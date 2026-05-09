#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "capturethread.h"
#include <stdio.h>

#pragma comment(lib,"../../MVCAMSDK.lib")
//SDK
int                     g_hCamera = -1;     //设备句柄
unsigned char           * g_pRgbBuffer=NULL;     //处理后数据缓存区
tSdkFrameHead           g_tFrameHead;       //图像帧头信息
tSdkCameraCapbility     g_tCapability;      //设备描述信息

int                     g_SaveParameter_num=0;    //保存参数组
int                     g_SaveImage_type=0;         //保存图像格式

int                     g_read_fps=0;       //统计读取帧率
int                     g_disply_fps=0;     //统计显示帧率

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),  m_scene(0), m_image_item(0)
{

    if(init_SDK()==-1){
        status =0;
        return ;
    }
    ui->setupUi(this);
    m_scene = new QGraphicsScene(this);
    ui->gvMain->setScene(m_scene);

    m_timer = new QTimer(this);
    connect(m_timer, SIGNAL(timeout()), this, SLOT(camera_statues()));
    m_timer->start(1000);


    m_thread = new CaptureThread(this);
    //start task thread
    connect(m_thread, SIGNAL(captured(QImage)),
            this, SLOT(Image_process(QImage)),
            Qt::BlockingQueuedConnection);


    m_camera_statuesFps = new QLabel(this);
    m_camera_statuesFps->setAlignment(Qt::AlignHCenter);
    ui->statusBar->addWidget(m_camera_statuesFps);






    ui->radioButton_A->setChecked(true);
    GUI_init_Resolution(g_hCamera,&g_tCapability);
    GUI_init_speed(g_hCamera,&g_tCapability);
    GUI_init_parameter(g_hCamera,&g_tCapability);

    m_thread->start();
    m_thread->stream();
    status =1;

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::changeEvent(QEvent *e)
{
    QMainWindow::changeEvent(e);
    switch (e->type()) {
    case QEvent::LanguageChange:
        ui->retranslateUi(this);
        break;
    default:
        break;
    }
}


/*
* 关闭程序处理
*
*
*/

void MainWindow::closeEvent(QCloseEvent * e)
{
    m_thread->stop();
    while (!m_thread->wait(1) )
    {
        QApplication::processEvents();
    }

    if(g_pRgbBuffer!=NULL){
        free(g_pRgbBuffer);
        g_pRgbBuffer=NULL;
    }

    if(g_hCamera>0){
        //相机反初始化。释放资源。
        CameraUnInit(g_hCamera);
        g_hCamera=-1;
    }

    QMainWindow::closeEvent(e);
}


/*
*  相机状态栏
*
*
*/

void MainWindow::camera_statues()
{
    m_camera_statuesFps->setText(QString("Capture fps: %1  Display fps :%2").arg(QString::number(g_read_fps, 'f', 2)).arg(QString::number(g_disply_fps, 'f', 2)));
    g_read_fps=0;
    g_disply_fps=0;
}



/*
*  图像显示刷新处理
*
*
*/

void MainWindow::Image_process(QImage img)
{
    if(m_image_item)
    {
        m_scene->removeItem(m_image_item);
        delete m_image_item;
        m_image_item = 0;
    }

    m_image_item = m_scene->addPixmap(QPixmap::fromImage(img));

    m_scene->setSceneRect(0, 0, img.width(), img.height());
    //m_scene->setSceneRect(0, 0, 800, 600);

    g_disply_fps++;
    
}

/*
*  SDK等初始化操作
*
*
*/
int MainWindow::init_SDK()
{

    int                     iCameraCounts = 4;
    int                     iStatus=-1;
    tSdkCameraDevInfo       tCameraEnumList[4];

    //sdk初始化  0 English 1中文
    CameraSdkInit(1);

    //枚举设备，并建立设备列表
    CameraEnumerateDevice(tCameraEnumList,&iCameraCounts);

    //没有连接设备
    if(iCameraCounts==0){
        return -1;
    }

    //相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    iStatus = CameraInit(&tCameraEnumList[0],-1,-1,&g_hCamera);

    //初始化失败
    if(iStatus!=CAMERA_STATUS_SUCCESS){
        return -1;
    }
    //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    CameraGetCapability(g_hCamera,&g_tCapability);

    g_pRgbBuffer = (unsigned char*)malloc(g_tCapability.sResolutionRange.iHeightMax*g_tCapability.sResolutionRange.iWidthMax
                                          *  (g_tCapability.sIspCapacity.bMonoSensor ? 1 : 3));

    // 设置分辨率
    //SetCameraResolution(0, 0, 640, 480);

    /*让SDK进入工作模式，开始接收来自相机发送的图像
    数据。如果当前相机是触发模式，则需要接收到
    触发帧以后才会更新图像。    */
    CameraPlay(g_hCamera);



    /*
        设置图像处理的输出格式，彩色黑白都支持RGB24位
    */
    if(g_tCapability.sIspCapacity.bMonoSensor){
        CameraSetIspOutFormat(g_hCamera,CAMERA_MEDIA_TYPE_MONO8);
    }else{
        CameraSetIspOutFormat(g_hCamera,CAMERA_MEDIA_TYPE_RGB8);
    }
    return 0;
}

// offsetx, offsety, width, height: 偏移宽高都选取为16的倍数兼容性最好（不同的相机对这个的要求不同，有的只需要2的倍数，有的可能需要16的倍数）
// offsetx, offsety, width, height: offset width and height are all chosen to be 16 times the best compatibility (different cameras have different requirements for this, some need only a multiple of 2, some may require a multiple of 16)
int MainWindow::SetCameraResolution(int offsetx, int offsety, int width, int height)
{
    tSdkImageResolution sRoiResolution = { 0 };

    // 设置成0xff表示自定义分辨率，设置成0到N表示选择预设分辨率
    // Set to 0xff for custom resolution, set to 0 to N for select preset resolution
    sRoiResolution.iIndex = 0xff;

    // iWidthFOV表示相机的视场宽度，iWidth表示相机实际输出宽度
    // 大部分情况下iWidthFOV=iWidth。有些特殊的分辨率模式如BIN2X2：iWidthFOV=2*iWidth，表示视场是实际输出宽度的2倍
    // iWidthFOV represents the camera's field of view width, iWidth represents the camera's actual output width
    // In most cases iWidthFOV=iWidth. Some special resolution modes such as BIN2X2:iWidthFOV=2*iWidth indicate that the field of view is twice the actual output width
    sRoiResolution.iWidth = width;
    sRoiResolution.iWidthFOV = width;

    // 高度，参考上面宽度的说明
    // height, refer to the description of the width above
    sRoiResolution.iHeight = height;
    sRoiResolution.iHeightFOV = height;

    // 视场偏移
    // Field of view offset
    sRoiResolution.iHOffsetFOV = offsetx;
    sRoiResolution.iVOffsetFOV = offsety;

    // ISP软件缩放宽高，都为0则表示不缩放
    // ISP software zoom width and height, all 0 means not zoom
    sRoiResolution.iWidthZoomSw = 0;
    sRoiResolution.iHeightZoomSw = 0;

    // BIN SKIP 模式设置（需要相机硬件支持）
    // BIN SKIP mode setting (requires camera hardware support)
    sRoiResolution.uBinAverageMode = 0;
    sRoiResolution.uBinSumMode = 0;
    sRoiResolution.uResampleMask = 0;
    sRoiResolution.uSkipMode = 0;

    return CameraSetImageResolution(g_hCamera, &sRoiResolution);
}

/*
*  QT界面初始化
*
*
*/


int MainWindow::GUI_init_parameter(int hCamera,tSdkCameraCapbility * pCameraInfo)
{
    GUI_Update_Resolution(hCamera, pCameraInfo);
    GUI_init_exposure(hCamera, pCameraInfo);
    if(!g_tCapability.sIspCapacity.bMonoSensor){
        GUI_init_WB(hCamera, pCameraInfo);
    }else{
        ui->frame_wb->setEnabled(false);
    }
    GUI_init_mmap(hCamera, pCameraInfo);
    GUI_init_isp(hCamera, pCameraInfo);
    GUI_set_speed(hCamera, pCameraInfo);
    GUI_init_Trigger(hCamera, pCameraInfo);

    ui->snap_path_lineEdit->setText(QString("./"));


    g_SaveImage_type=1;
    ui->radioButton_jpg->setChecked(true);

    return 0;
}

int  MainWindow::GUI_Update_Resolution(int hCamera,tSdkCameraCapbility * pCameraInfo)
{
    int iImageSizeDesc=pCameraInfo->iImageSizeDesc;// 预设分辨率选择

    //获得当前预览的分辨率。
    tSdkImageResolution sResolution = { 0 };
    CameraGetImageResolution(hCamera,&sResolution);
    if (sResolution.iIndex != 255)
    {
        ui->res_combobox->setCurrentIndex(sResolution.iIndex);
    }
    else
    {
        ui->res_combobox->setItemText(iImageSizeDesc,
                                      QString("%1X%2").arg(sResolution.iWidth).arg(sResolution.iHeight));
        ui->res_combobox->setCurrentIndex(iImageSizeDesc);
    }

    return 1;
}

int  MainWindow::GUI_init_Resolution(int hCamera,tSdkCameraCapbility * pCameraInfo)
{
    int                     i=0;
    tSdkImageResolution     *pImageSizeDesc=pCameraInfo->pImageSizeDesc;// 预设分辨率选择
    int                     iImageSizeDesc=pCameraInfo->iImageSizeDesc; // 预设分辨率的个数，即pImageSizeDesc数组的大小


	//windows 编码
	QTextCodec::setCodecForCStrings(QTextCodec::codecForName("GB2312"));

    for(i=0;i<iImageSizeDesc;i++){
        ui->res_combobox->addItem(QString("%1").arg(pImageSizeDesc[i].acDescription));

//		QTextCodec::setCodecForCStrings(QTextCodec::codecForName("UTF-8"));
    }
    ui->res_combobox->addItem("Customize");

    return 1;
}

int  MainWindow::GUI_init_Trigger(int hCamera,tSdkCameraCapbility * pCameraInfo)
{
    int  pbySnapMode;
    int StrobeMode=0;
    int  uPolarity=0;

//获得相机的触发模式。
    CameraGetTriggerMode(hCamera,&pbySnapMode);

//设置相机的触发模式。0表示连续采集模式；1表示软件触发模式；2表示硬件触发模式。
    switch(pbySnapMode){
        case 0:
            ui->radioButton_collect->setChecked(true);
            ui->flashlight->setEnabled(false);
            ui->software_trigger_once_button->setEnabled(false);

        break;
        case 1:
            ui->radioButton_software_trigger->setChecked(true);
            ui->software_trigger_once_button->setEnabled(true);
            ui->flashlight->setEnabled(false);

        break;
        case 2:
            ui->radioButton_trigger_hardware->setChecked(true);
            ui->flashlight->setEnabled(true);
            ui->software_trigger_once_button->setEnabled(false);

            CameraGetStrobeMode(hCamera,&StrobeMode);
            CameraGetStrobePolarity(hCamera,&uPolarity);

            if(StrobeMode){
                CameraSetStrobePolarity(hCamera,1);

                ui->flashlight_manual->setChecked(true);
                ui->flashlight_polarity->setEnabled(true);
                if(uPolarity){
                    ui->flashlight_h->setChecked(true);
                    CameraSetStrobePolarity(hCamera,1);
                }else{
                    ui->flashlight_l->setChecked(true);
                    CameraSetStrobePolarity(hCamera,0);
                }
            }else{
                CameraSetStrobePolarity(hCamera,0);
                ui->flashlight_auto->setChecked(true);
                ui->flashlight_polarity->setEnabled(false);
            }

        break;
        default:
            ui->radioButton_collect->setChecked(true);
            ui->flashlight->setEnabled(false);
            ui->software_trigger_once_button->setEnabled(false);
        break;
    }



    return 1;
}

int  MainWindow::GUI_init_exposure(int hCamera,tSdkCameraCapbility * pCameraInfo)
{

    BOOL            AEstate=FALSE;
    int             pbyAeTarget;
    double          pfExposureTime;
    int             pusAnalogGain;
    BOOL            FlickEnable=FALSE;
    int             piFrequencySel;
    double	        m_fExpLineTime=0;//当前的行曝光时间，单位为us
    tSdkExpose      *  SdkExpose =   &pCameraInfo->sExposeDesc;

    //获得相机当前的曝光模式。
    CameraGetAeState(hCamera,&AEstate);

    //获得自动曝光的亮度目标值。
    CameraGetAeTarget(hCamera,&pbyAeTarget);

    //获得自动曝光时抗频闪功能的使能状态。
    CameraGetAntiFlick(hCamera,&FlickEnable);

    //获得相机的曝光时间。
    CameraGetExposureTime(hCamera,&pfExposureTime);

    //获得图像信号的模拟增益值。
    CameraGetAnalogGain(hCamera,&pusAnalogGain);

    //获得自动曝光时，消频闪的频率选择。
    CameraGetLightFrequency(hCamera,&piFrequencySel);

/*
    获得一行的曝光时间。对于CMOS传感器，其曝光
    的单位是按照行来计算的，因此，曝光时间并不能在微秒
    级别连续可调。而是会按照整行来取舍。这个函数的
    作用就是返回CMOS相机曝光一行对应的时间。
*/
    CameraGetExposureLineTime(hCamera, &m_fExpLineTime);



    connect(ui->AE_spinBox,SIGNAL(valueChanged(int)),ui->AE_horizontalSlider,SLOT(setValue(int)));
    connect(ui->AE_horizontalSlider,SIGNAL(valueChanged(int)),ui->AE_spinBox,SLOT(setValue(int)));

    connect(ui->horizontalSlider_gain,SIGNAL(valueChanged(int)),ui->spinBox_gain,SLOT(setValue(int)));
    connect(ui->spinBox_gain,SIGNAL(valueChanged(int)),ui->horizontalSlider_gain,SLOT(setValue(int)));


    ui->AE_horizontalSlider->setMinimum(SdkExpose->uiTargetMin);
    ui->AE_horizontalSlider->setMaximum(SdkExpose->uiTargetMax);
    ui->AE_spinBox->setMinimum(SdkExpose->uiTargetMin);
    ui->AE_spinBox->setMaximum(SdkExpose->uiTargetMax);
    ui->AE_horizontalSlider->setValue(pbyAeTarget);



    ui->horizontalSlider_gain->setMinimum(SdkExpose->uiAnalogGainMin);
    ui->horizontalSlider_gain->setMaximum(SdkExpose->uiAnalogGainMax);
    ui->spinBox_gain->setMinimum(SdkExpose->uiExposeTimeMin);
    ui->spinBox_gain->setMaximum(SdkExpose->uiExposeTimeMax);
    ui->horizontalSlider_gain->setValue(pusAnalogGain);


    exposure_time_lineedit_status=false;

    ui->horizontalSlider_exposure_time->setMinimum(SdkExpose->uiExposeTimeMin);
    ui->horizontalSlider_exposure_time->setMaximum(SdkExpose->uiExposeTimeMax);
    ui->horizontalSlider_exposure_time->setValue(pfExposureTime/m_fExpLineTime);


    if(!AEstate){
        //设置相机曝光的模式。自动或者手动。关
        CameraSetAeState(hCamera,false);
        ui->exposure_manual->setEnabled(true);
        ui->exposure_auto->setEnabled(false);
        ui->exposure_mode_manual->setChecked(true);
    }else{
        //设置相机曝光的模式。自动或者手动。开
        CameraSetAeState(hCamera,true);
        ui->exposure_mode_auto->setChecked(true);
        ui->exposure_auto->setEnabled(true);
        ui->exposure_manual->setEnabled(false);

        if(FlickEnable){

            //设置自动曝光时抗频闪功能的使能状态。对于手动曝光模式下无效。
            CameraSetAntiFlick(hCamera,true);
            ui->flicker_checkBox->setChecked(true);

            if(piFrequencySel){
                //设置自动曝光时消频闪的频率。0:50HZ , 1:60HZ
                CameraSetLightFrequency(hCamera,piFrequencySel);
                ui->radioButton_60HZ->setChecked(true);
            }else{
                CameraSetLightFrequency(hCamera,piFrequencySel);
                ui->radioButton_50HZ->setChecked(true);
            }
        }else{
            CameraSetAntiFlick(hCamera,false);
            ui->radioButton_50HZ->setEnabled(false);
            ui->radioButton_60HZ->setEnabled(false);

            if(piFrequencySel){
                //设置自动曝光时消频闪的频率。0:50HZ , 1:60HZ
                CameraSetLightFrequency(hCamera,piFrequencySel);
                ui->radioButton_60HZ->setChecked(true);
            }else{
                CameraSetLightFrequency(hCamera,piFrequencySel);
                ui->radioButton_50HZ->setChecked(true);
            }
        }
    }

    return 1;
}

int  MainWindow::GUI_init_WB(int hCamera,tSdkCameraCapbility * pCameraInfo)
{
    int RPos,GPos,BPos,Saturation;


    ui->radioButton_AWB_auto->setChecked(true);

    CameraGetGain(hCamera,&RPos,&GPos,&BPos);
    CameraGetSaturation(hCamera,&Saturation);

    connect(ui->horizontalSlider_gain_r,SIGNAL(valueChanged(int)),ui->spinBox_gain_r,SLOT(setValue(int)));
    connect(ui->spinBox_gain_r,SIGNAL(valueChanged(int)),ui->horizontalSlider_gain_r,SLOT(setValue(int)));

    connect(ui->horizontalSlider_gain_g,SIGNAL(valueChanged(int)),ui->spinBox_gain_g,SLOT(setValue(int)));
    connect(ui->spinBox_gain_g,SIGNAL(valueChanged(int)),ui->horizontalSlider_gain_g,SLOT(setValue(int)));

    connect(ui->horizontalSlider_gain_b,SIGNAL(valueChanged(int)),ui->spinBox_gain_b,SLOT(setValue(int)));
    connect(ui->spinBox_gain_b,SIGNAL(valueChanged(int)),ui->horizontalSlider_gain_b,SLOT(setValue(int)));

    connect(ui->horizontalSlider_saturation,SIGNAL(valueChanged(int)),ui->spinBox_saturation,SLOT(setValue(int)));
    connect(ui->spinBox_saturation,SIGNAL(valueChanged(int)),ui->horizontalSlider_saturation,SLOT(setValue(int)));

    ui->horizontalSlider_gain_r->setValue(RPos);
    ui->horizontalSlider_gain_g->setValue(GPos);
    ui->horizontalSlider_gain_b->setValue(BPos);

    ui->horizontalSlider_saturation->setValue(Saturation);

    return 1;
}

int  MainWindow::GUI_init_mmap(int hCamera,tSdkCameraCapbility * pCameraInfo)
{
    int                 gamma=0;
    int                 contrast=0;
    tGammaRange         *  sGammaRange      =   &pCameraInfo->sGammaRange;
    tContrastRange      *  sContrastRange   =   &pCameraInfo->sContrastRange;

    //获得LUT动态生成模式下的Gamma值。
    CameraGetGamma(hCamera,&gamma);

    //获得LUT动态生成模式下的对比度值
    CameraGetContrast(hCamera,&contrast);

//设置最大最小值

    connect(ui->gamma_horizontalSlider,SIGNAL(valueChanged(int)),ui->spinBox_gamma,SLOT(setValue(int)));
    connect(ui->spinBox_gamma,SIGNAL(valueChanged(int)),ui->gamma_horizontalSlider,SLOT(setValue(int)));

    connect(ui->contrast_horizontalSlider,SIGNAL(valueChanged(int)),ui->spinBox_contrast,SLOT(setValue(int)));
    connect(ui->spinBox_contrast,SIGNAL(valueChanged(int)),ui->contrast_horizontalSlider,SLOT(setValue(int)));



    ui->gamma_horizontalSlider->setMinimum(sGammaRange->iMin);
    ui->gamma_horizontalSlider->setMaximum(sGammaRange->iMax);
    ui->spinBox_gamma->setMinimum(sGammaRange->iMin);
    ui->spinBox_gamma->setMaximum(sGammaRange->iMax);
    ui->gamma_horizontalSlider->setValue(gamma);


    ui->contrast_horizontalSlider->setMinimum(sContrastRange->iMin);
    ui->contrast_horizontalSlider->setMaximum(sContrastRange->iMax);
    ui->spinBox_contrast->setMinimum(sContrastRange->iMin);
    ui->spinBox_contrast->setMaximum(sContrastRange->iMax);
    ui->contrast_horizontalSlider->setValue(contrast);


    return 1;
}

int  MainWindow::GUI_init_isp(int hCamera,tSdkCameraCapbility * pCameraInfo)
{
    BOOL        m_bHflip=FALSE;
    BOOL        m_bVflip=FALSE;
    int         m_Sharpness=0;

    tSharpnessRange  *  SharpnessRange =   &pCameraInfo->sSharpnessRange;
    //获得图像的镜像状态。
    CameraGetMirror(hCamera, MIRROR_DIRECTION_HORIZONTAL, &m_bHflip);
    CameraGetMirror(hCamera, MIRROR_DIRECTION_VERTICAL,   &m_bVflip);
    //获取当前锐化设定值。
    CameraGetSharpness(hCamera, &m_Sharpness);


//设置最大最小值 +10
    connect(ui->horizontalSlider_isp_acutance,SIGNAL(valueChanged(int)),ui->spinBox_isp_acutance,SLOT(setValue(int)));
    connect(ui->spinBox_isp_acutance,SIGNAL(valueChanged(int)),ui->horizontalSlider_isp_acutance,SLOT(setValue(int)));


    ui->horizontalSlider_isp_acutance->setMinimum(SharpnessRange->iMin);
    ui->horizontalSlider_isp_acutance->setMaximum(SharpnessRange->iMax);
    ui->spinBox_isp_acutance->setMinimum(SharpnessRange->iMin);
    ui->spinBox_isp_acutance->setMaximum(SharpnessRange->iMax);
    ui->horizontalSlider_isp_acutance->setValue(m_Sharpness);



////设置选中内容
    if(m_bHflip){
        ui->checkBox_isp_h->setChecked(true);
    }else{
        ui->checkBox_isp_h->setChecked(false);
    }
    if(m_bVflip){
        ui->checkBox_isp_v->setChecked(true);
    }else{
        ui->checkBox_isp_v->setChecked(false);
    }


    return 1;
}

int  MainWindow::GUI_set_speed(int hCamera,tSdkCameraCapbility * pCameraInfo)
{
    int         pbyFrameSpeed=0;
    //获得相机输出图像的帧率选择索引号。
    CameraGetFrameSpeed(hCamera,&pbyFrameSpeed);

    radioButton_speed[pbyFrameSpeed]->setChecked(true);

    return 1;
}

int  MainWindow::GUI_init_speed(int hCamera,tSdkCameraCapbility * pCameraInfo)
{

    int         i,size=pCameraInfo->iFrameSpeedDesc;

	//windows 编码
	QTextCodec::setCodecForCStrings(QTextCodec::codecForName("GB2312"));
    for(i=0;i<size;i++){



        radioButton_speed[i] = new QRadioButton(QString("%1").arg(pCameraInfo->pFrameSpeedDesc[i].acDescription) ,ui->groupBox_speed);
        radioButton_speed[i]->setGeometry(QRect(10+(i*80), 5, 80, 30));
        // signals and slots
        connect(radioButton_speed[i], SIGNAL(clicked(bool)), this, SLOT(radioChange()));
    }

    return 1;
}

/*
*  QT界面按钮等操作
*
*
*/

//分辨率操作
void MainWindow::on_res_combobox_activated(int index)
{

    tSdkImageResolution   *pImageSizeDesc=g_tCapability.pImageSizeDesc;// 预设分辨率选择

    m_thread->pause();
    //设置预览的分辨率。
    CameraSetImageResolution(g_hCamera,&(pImageSizeDesc[index]));
    m_thread->stream();
    //ui->gvMain->scale(1/1.5,1/1.5);
}

//连续采集模式
void MainWindow::on_radioButton_collect_clicked(bool checked)
{
    ui->radioButton_collect->setChecked(true);
    if(checked)
    {
    //获得相机的触发模式。
        CameraSetTriggerMode(g_hCamera,0);


        ui->radioButton_collect->setChecked(true);
        ui->flashlight->setEnabled(false);
        ui->software_trigger_once_button->setEnabled(false);
    }
}
//软触发模式
void MainWindow::on_radioButton_software_trigger_clicked(bool checked)
{
    ui->radioButton_software_trigger->setChecked(true);
    if(checked)
    {
    //获得相机的触发模式。
        CameraSetTriggerMode(g_hCamera,1);

    //设置相机的触发模式。0表示连续采集模式；1表示软件触发模式；2表示硬件触发模式。

        ui->radioButton_software_trigger->setChecked(true);
        ui->software_trigger_once_button->setEnabled(true);
        ui->flashlight->setEnabled(false);
    }
}

//硬触发模式
void MainWindow::on_radioButton_trigger_hardware_clicked(bool checked)
{
    
    int StrobeMode=0;
    int  uPolarity=0;

    ui->radioButton_trigger_hardware->setChecked(true);
    if(checked)
    {
        //获得相机的触发模式。
        CameraSetTriggerMode(g_hCamera,2);

        //设置相机的触发模式。0表示连续采集模式；1表示软件触发模式；2表示硬件触发模式。


        ui->flashlight->setEnabled(true);
        ui->software_trigger_once_button->setEnabled(false);

        CameraGetStrobeMode(g_hCamera,&StrobeMode);
        CameraGetStrobePolarity(g_hCamera,&uPolarity);
        if(StrobeMode){
            CameraSetStrobePolarity(g_hCamera,1);

            ui->flashlight_manual->setChecked(true);

            ui->flashlight_polarity->setEnabled(true);
            if(uPolarity){
                ui->flashlight_h->setChecked(true);
                CameraSetStrobePolarity(g_hCamera,1);
            }else{
                ui->flashlight_l->setChecked(true);
                CameraSetStrobePolarity(g_hCamera,0);
            }
        }else{
            CameraSetStrobePolarity(g_hCamera,0);
            ui->flashlight_auto->setChecked(true);
            ui->flashlight_polarity->setEnabled(false);
        }
    }


}

//软触发一次操作
void MainWindow::on_software_trigger_once_button_clicked()
{
    //执行一次软触发。执行后，会触发由CameraSetTriggerCount指定的帧数。
    CameraSoftTrigger(g_hCamera);
}

//闪光灯自动模式
void MainWindow::on_flashlight_auto_clicked(bool checked)
{
    ui->flashlight_auto->setChecked(true);

    if(checked){
        CameraSetStrobeMode(g_hCamera,0);

        ui->flashlight_polarity->setEnabled(false);
    }
}

//闪光灯手动模式
void MainWindow::on_flashlight_manual_clicked(bool checked)
{

    int  uPolarity=0;


    if(checked)
    {
        CameraSetStrobeMode(g_hCamera,1);

        CameraGetStrobePolarity(g_hCamera,&uPolarity);


        ui->flashlight_polarity->setEnabled(true);
        if(uPolarity){
            ui->flashlight_h->setChecked(true);
            CameraSetStrobePolarity(g_hCamera,1);
        }else{
            ui->flashlight_l->setChecked(true);
            CameraSetStrobePolarity(g_hCamera,0);
        }
    }
    ui->flashlight_manual->setChecked(true);

}

//闪光灯 高电平有效
void MainWindow::on_flashlight_h_clicked(bool checked)
{
    ui->flashlight_h->setChecked(true);
    if(checked){
        CameraSetStrobePolarity(g_hCamera,1);
    }
}

//闪光灯 低电平有效
void MainWindow::on_flashlight_l_clicked(bool checked)
{
    ui->flashlight_l->setChecked(true);
    if(checked){
        CameraSetStrobePolarity(g_hCamera,0);
    }
}

//自动曝光调节操作
void MainWindow::on_AE_horizontalSlider_valueChanged(int value)
{

/*
    设定自动曝光的亮度目标值。设定范围由CameraGetCapability函数获得。
*/
    CameraSetAeTarget(g_hCamera,value);
}

//频闪操作
void MainWindow::on_flicker_checkBox_clicked(bool checked)
{
    if(checked){
        CameraSetAntiFlick(g_hCamera,true);
        ui->radioButton_50HZ->setEnabled(true);
        ui->radioButton_60HZ->setEnabled(true);

    }else{
        CameraSetAntiFlick(g_hCamera,false);
        ui->radioButton_50HZ->setEnabled(false);
        ui->radioButton_60HZ->setEnabled(false);
    }
}



//频闪 50hz
void MainWindow::on_radioButton_50HZ_clicked(bool checked)
{
    if(checked){
        //设置自动曝光时消频闪的频率。0: 50hz  1:60hz
        CameraSetLightFrequency(g_hCamera,0);
    }
    ui->radioButton_50HZ->setChecked(true);
}

//频闪 60hz
void MainWindow::on_radioButton_60HZ_clicked(bool checked)
{
    if(checked){
        //设置自动曝光时消频闪的频率。0: 50hz  1:60hz
        CameraSetLightFrequency(g_hCamera,1);
    }
    ui->radioButton_60HZ->setChecked(true);

}

//曝光模式 自动
void MainWindow::on_exposure_mode_auto_clicked(bool checked)
{
    ui->exposure_mode_auto->setChecked(true);
    if(checked){
        ui->exposure_auto->setEnabled(true);
        ui->exposure_manual->setEnabled(false);

        BOOL        FlickEnable=false;
        int         piFrequencySel;

        //获得自动曝光时抗频闪功能的使能状态。
        CameraGetAntiFlick(g_hCamera,&FlickEnable);
        //获得自动曝光时，消频闪的频率选择。
        CameraGetLightFrequency(g_hCamera,&piFrequencySel);

        CameraSetAeState(g_hCamera,true);

        if(FlickEnable){
            //设置自动曝光时抗频闪功能的使能状态。对于手动曝光模式下无效。
            CameraSetAntiFlick(g_hCamera,true);
            ui->flicker_checkBox->setChecked(true);

            if(piFrequencySel){
                //设置自动曝光时消频闪的频率。0:50HZ , 1:60HZ
                CameraSetLightFrequency(g_hCamera,piFrequencySel);
                ui->radioButton_60HZ->setChecked(true);
            }else{
                CameraSetLightFrequency(g_hCamera,piFrequencySel);
                ui->radioButton_50HZ->setChecked(true);
            }
        }else{
            CameraSetAntiFlick(g_hCamera,false);
            ui->radioButton_50HZ->setEnabled(false);
            ui->radioButton_50HZ->setEnabled(false);
        }
    }
}

//曝光模式 手动
void MainWindow::on_exposure_mode_manual_clicked(bool checked)
{

    double          pfExposureTime;
    int             pusAnalogGain;
    double	    m_fExpLineTime=0;//当前的行曝光时间，单位为us


    ui->exposure_mode_manual->setChecked(true);

    if(checked){


        //获得相机的曝光时间。
        CameraGetExposureTime(g_hCamera,&pfExposureTime);

        //获得图像信号的模拟增益值。
        CameraGetAnalogGain(g_hCamera,&pusAnalogGain);

    /*
        获得一行的曝光时间。对于CMOS传感器，其曝光
        的单位是按照行来计算的，因此，曝光时间并不能在微秒
        级别连续可调。而是会按照整行来取舍。这个函数的
        作用就是返回CMOS相机曝光一行对应的时间。
    */
        CameraGetExposureLineTime(g_hCamera, &m_fExpLineTime);


        ui->horizontalSlider_gain->setValue(pusAnalogGain);

        ui->horizontalSlider_exposure_time->setValue(pfExposureTime/m_fExpLineTime);

        CameraSetAeState(g_hCamera,false);

        ui->exposure_auto->setEnabled(false);
        ui->exposure_manual->setEnabled(true);
    }

}

//手动曝光 增益
void MainWindow::on_horizontalSlider_gain_valueChanged(int value)
{
    /*
        设置相机的图像模拟增益值。该值乘以CameraGetCapability获得
        的相机属性结构体中sExposeDesc.fAnalogGainStep，就
        得到实际的图像信号放大倍数。
    */

    CameraSetAnalogGain(g_hCamera,value);
}
//手动曝光 曝光值
void MainWindow::on_horizontalSlider_exposure_time_valueChanged(int value)
{

    double          m_fExpLineTime=0;//当前的行曝光时间，单位为us
    char            buffer[16];

    /*
        获得一行的曝光时间。对于CMOS传感器，其曝光
        的单位是按照行来计算的，因此，曝光时间并不能在微秒M
        级别连续可调。而是会按照整行来取舍。这个函数的
        作用就是返回CMOS相机曝光一行对应的时间。
    */
    CameraGetExposureLineTime(g_hCamera, &m_fExpLineTime);

    /*
        设置曝光时间。单位为微秒。对于CMOS传感器，其曝光
        的单位是按照行来计算的，因此，曝光时间并不能在微秒
        级别连续可调。而是会按照整行来取舍。在调用
        本函数设定曝光时间后，建议再调用CameraGetExposureTime
        来获得实际设定的值。
    */
    CameraSetExposureTime(g_hCamera,value*m_fExpLineTime);

    if(!exposure_time_lineedit_status){
        //除以1000是换算成毫秒
        sprintf( buffer,"%0.3f",((value*m_fExpLineTime)/1000));
        ui->lineEdit_exposure_time->setText(QString(buffer));
    }

}


void MainWindow::on_lineEdit_exposure_time_returnPressed()
{

    double          m_fExpLineTime=0;//当前的行曝光时间，单位为us
    int             exposure;

    /*
        获得一行的曝光时间。对于CMOS传感器，其曝光
        的单位是按照行来计算的，因此，曝光时间并不能在微秒
        级别连续可调。而是会按照整行来取舍。这个函数的
        作用就是返回CMOS相机曝光一行对应的时间。
    */
    CameraGetExposureLineTime(g_hCamera, &m_fExpLineTime);


    QString value =ui->lineEdit_exposure_time->text();
    double tmp=value.toDouble();

    exposure=(int)tmp*1000/m_fExpLineTime;
    exposure_time_lineedit_status=true;
    ui->horizontalSlider_exposure_time->setValue(exposure);
    exposure_time_lineedit_status=false;
    /*
        设置曝光时间。单位为微秒。对于CMOS传感器，其曝光
        的单位是按照行来计算的，因此，曝光时间并不能在微秒
        级别连续可调。而是会按照整行来取舍。在调用
        本函数设定曝光时间后，建议再调用CameraGetExposureTime
        来获得实际设定的值。
    */
    CameraSetExposureTime(g_hCamera,exposure*m_fExpLineTime);


}

//参数操作A按钮
void MainWindow::on_radioButton_A_clicked(bool checked)
{
    ui->radioButton_A->setChecked(true);

    if(checked){

        /*
        存当前相机参数到指定的参数组中。相机提供了A,B,C,D
        A,B,C,D四组空间来进行参数的保存。
        */
        CameraSaveParameter(g_hCamera,g_SaveParameter_num);
        g_SaveParameter_num=0;

        //加载指定组的参数到相机中。
        CameraLoadParameter(g_hCamera,g_SaveParameter_num);
        m_thread->pause();
        GUI_init_parameter(g_hCamera,&g_tCapability);
        m_thread->stream();

    }
}

//参数操作B按钮
void MainWindow::on_radioButton_B_clicked(bool checked)
{
    ui->radioButton_B->setChecked(true);

    if(checked){


        /*
        存当前相机参数到指定的参数组中。相机提供了A,B,C,D
        A,B,C,D四组空间来进行参数的保存。
        */
        CameraSaveParameter(g_hCamera,g_SaveParameter_num);
        g_SaveParameter_num=1;

        //加载指定组的参数到相机中。
        CameraLoadParameter(g_hCamera,g_SaveParameter_num);

        m_thread->pause();
        GUI_init_parameter(g_hCamera,&g_tCapability);
        m_thread->stream();
    }

}

//参数操作C按钮
void MainWindow::on_radioButton_C_clicked(bool checked)
{
    ui->radioButton_C->setChecked(true);

    if(checked){

        /*
        存当前相机参数到指定的参数组中。相机提供了A,B,C,D
        A,B,C,D四组空间来进行参数的保存。
        */
        CameraSaveParameter(g_hCamera,g_SaveParameter_num);
        g_SaveParameter_num=2;

        //加载指定组的参数到相机中。
        CameraLoadParameter(g_hCamera,g_SaveParameter_num);

        m_thread->pause();
        GUI_init_parameter(g_hCamera,&g_tCapability);
        m_thread->stream();

    }

}

//参数操作D按钮
void MainWindow::on_radioButton_D_clicked(bool checked)
{
    ui->radioButton_D->setChecked(true);

    if(checked){

        /*
        存当前相机参数到指定的参数组中。相机提供了A,B,C,D
        A,B,C,D四组空间来进行参数的保存。
        */
        CameraSaveParameter(g_hCamera,g_SaveParameter_num);
        g_SaveParameter_num=3;

        //加载指定组的参数到相机中。
        CameraLoadParameter(g_hCamera,g_SaveParameter_num);

        m_thread->pause();
        GUI_init_parameter(g_hCamera,&g_tCapability);
        m_thread->stream();

    }

}

//默认参数
void MainWindow::on_pushButton_para_acquiesce_released()
{
    //加载指定组的参数到相机中。
    CameraLoadParameter(g_hCamera,PARAMETER_TEAM_DEFAULT);

    m_thread->pause();
    GUI_init_parameter(g_hCamera,&g_tCapability);
    m_thread->stream();
}

//保存参数
void MainWindow::on_pushButton_para_save_released()
{
    /*
    存当前相机参数到指定的参数组中。相机提供了A,B,C,D
    A,B,C,D四组空间来进行参数的保存。
    */
    CameraSaveParameter(g_hCamera,g_SaveParameter_num);
}

//加载参数文件
void MainWindow::on_pushButton_para_load_released()
{
    QFileDialog *fileDialog = new QFileDialog(this);
    fileDialog->setWindowTitle(tr("Select file"));
    fileDialog->setFilter(tr("config Files(*.config )"));
    if(fileDialog->exec() == QDialog::Accepted) {
        QString path = fileDialog->selectedFiles()[0];
        //QMessageBox::information(NULL, tr("Path"), tr("You selected ") + path);

        char*  filename;
        QByteArray tmp = path.toLatin1();
        filename=tmp.data();

        CameraReadParameterFromFile(g_hCamera,filename);

        m_thread->pause();
        GUI_init_parameter(g_hCamera,&g_tCapability);
        m_thread->stream();

    }
    delete fileDialog;
}

//白平衡按钮操作
void MainWindow::on_AWB_once_button_released()
{   
    int RPos,GPos,BPos;

    CameraSetOnceWB(g_hCamera);

    CameraGetGain(g_hCamera,&RPos,&GPos,&BPos);


    ui->horizontalSlider_gain_r->setValue(RPos);
    ui->horizontalSlider_gain_g->setValue(GPos);
    ui->horizontalSlider_gain_b->setValue(BPos);

}




//白平衡 增益R
void MainWindow::on_horizontalSlider_gain_r_valueChanged(int value)
{
    int RPos,GPos,BPos;

//设置选中项
    //获得当前预览的分辨率。
    CameraGetGain(g_hCamera,&RPos,&GPos,&BPos);

    CameraSetGain(g_hCamera,value,GPos,BPos);
}

//白平衡 增益G
void MainWindow::on_horizontalSlider_gain_g_valueChanged(int value)
{
    int RPos,GPos,BPos;

//设置选中项
    //获得当前预览的分辨率。
    CameraGetGain(g_hCamera,&RPos,&GPos,&BPos);

    CameraSetGain(g_hCamera,RPos,value,BPos);

}

//白平衡 增益B
void MainWindow::on_horizontalSlider_gain_b_valueChanged(int value)
{
    int RPos,GPos,BPos;

//设置选中项
    //获得当前预览的分辨率。
    CameraGetGain(g_hCamera,&RPos,&GPos,&BPos);

    CameraSetGain(g_hCamera,RPos,GPos,value);
}

//白平衡 饱和度
void MainWindow::on_horizontalSlider_saturation_valueChanged(int value)
{
    CameraSetSaturation(g_hCamera,value);
}

//GAMMA
void MainWindow::on_gamma_horizontalSlider_valueChanged(int value)
{
    CameraSetGamma(g_hCamera,value);
}

//对比度
void MainWindow::on_contrast_horizontalSlider_valueChanged(int value)
{
    /*
        设定LUT动态生成模式下的对比度值。设定的值会
        马上保存在SDK内部，但是只有当相机处于动态
        参数生成的LUT模式时，才会生效。
    */
    CameraSetContrast(g_hCamera,value);
}

//水平镜像
void MainWindow::on_checkBox_isp_h_clicked(bool checked)
{
    if(checked){
        //设置图像镜像操作。镜像操作分为水平和垂直两个方向。水平
        CameraSetMirror(g_hCamera, MIRROR_DIRECTION_HORIZONTAL, true);
    }else{
        //设置图像镜像操作。镜像操作分为水平和垂直两个方向。水平
        CameraSetMirror(g_hCamera, MIRROR_DIRECTION_HORIZONTAL, false);
    }

}

//垂直镜像
void MainWindow::on_checkBox_isp_v_clicked(bool checked)
{
    if(checked){
        //设置图像镜像操作。镜像操作分为水平和垂直两个方向。垂直
        CameraSetMirror(g_hCamera, MIRROR_DIRECTION_VERTICAL, true);
    }else{
        //设置图像镜像操作。镜像操作分为水平和垂直两个方向。垂直
        CameraSetMirror(g_hCamera, MIRROR_DIRECTION_VERTICAL, false);
    }
}

//锐化
void MainWindow::on_horizontalSlider_isp_acutance_valueChanged(int value)
{
    CameraSetSharpness(g_hCamera,value);
}


//保存图片路径设置
void MainWindow::on_pushButton_snap_path_released()
{
    QFileDialog* openFilePath = new QFileDialog( this, "Select Folder", "");     //打开一个目录选择对话框
    openFilePath-> setFileMode( QFileDialog::Directory );
    if ( openFilePath->exec() == QDialog::Accepted )
    {
        QString path = openFilePath->selectedFiles()[0];

        ui->snap_path_lineEdit->setText(path);
    }
    delete openFilePath;

}

//保存图片按钮确认
void MainWindow::on_pushButton_snap_catch_released()
{
    tSdkFrameHead	tFrameHead;
    BYTE			*pbyBuffer;
    bool            mono = (g_tCapability.sIspCapacity.bMonoSensor != 0);
    char            filename[512]={0};
    QString         path = ui->snap_path_lineEdit->text();

    char* dir;
    QByteArray tmp = path.toLatin1();
    dir=tmp.data();

    static int s_snap_no = 0;
    sprintf(filename,"%ssnap%d",dir,++s_snap_no);

    //CameraSnapToBuffer抓拍一张图像保存到buffer中
    if(CameraSnapToBuffer(g_hCamera,&tFrameHead,&pbyBuffer,1000) == CAMERA_STATUS_SUCCESS)
    {
        if (g_SaveImage_type == 4)
        {
            CameraSaveImage(g_hCamera, filename,pbyBuffer, &tFrameHead, FILE_RAW, 100);
            CameraReleaseImageBuffer(g_hCamera, pbyBuffer);
        }
        else
        {
            BYTE* pbImgBuffer = (unsigned char*)malloc(tFrameHead.iWidth * tFrameHead.iHeight
                                                 * (mono ? 1 : 3) );
            if (pbImgBuffer == NULL)
            {
                CameraReleaseImageBuffer(g_hCamera, pbyBuffer);
                QMessageBox::information(this, "Error", "No Memory");
                return;
            }
            CameraImageProcess(g_hCamera, pbyBuffer,pbImgBuffer,&tFrameHead);
            CameraReleaseImageBuffer(g_hCamera, pbyBuffer);

            switch(g_SaveImage_type)
            {
                case 1: // jpg
                CameraSaveImage(g_hCamera, filename,pbImgBuffer, &tFrameHead, FILE_JPG, 80);
                break;

                case 2: // png
                CameraSaveImage(g_hCamera, filename,pbImgBuffer, &tFrameHead, FILE_PNG, 100);
                break;

                case 3: // bmp
                CameraSaveImage(g_hCamera, filename,pbImgBuffer, &tFrameHead, mono ? FILE_BMP_8BIT : FILE_BMP, 100);
                break;

                default :
                break;
            }
            free(pbImgBuffer);
        }
    }
    else
    {
        QMessageBox::information(this, "Error", "Snap fail");
    }
}

void MainWindow::on_radioButton_jpg_clicked()
{
    g_SaveImage_type=1;
}

void MainWindow::on_radioButton_png_clicked()
{
    g_SaveImage_type=2;
}

//保存图片格式 bmp
void MainWindow::on_radioButton_bmp_clicked()
{
    g_SaveImage_type=3;
}

//保存图片格式 raw
void MainWindow::on_radioButton_raw_clicked()
{
    g_SaveImage_type=4;
}

//速度选择
void MainWindow::radioChange()
{

    if (sender() == radioButton_speed[0])
    {
        /*
            设定相机输出图像的帧率。相机可供选择的帧率模式由
            CameraGetCapability获得的信息结构体中iFrameSpeedDesc
            表示最大帧率选择模式个数。
        */
        CameraSetFrameSpeed(g_hCamera,0);
    }
    else if (sender() == radioButton_speed[1])
    {
        /*
            设定相机输出图像的帧率。相机可供选择的帧率模式由
            CameraGetCapability获得的信息结构体中iFrameSpeedDesc
            表示最大帧率选择模式个数。
        */
        CameraSetFrameSpeed(g_hCamera,1);

    }
    else if (sender() == radioButton_speed[2])
    {
        /*
            设定相机输出图像的帧率。相机可供选择的帧率模式由
            CameraGetCapability获得的信息结构体中iFrameSpeedDesc
            表示最大帧率选择模式个数。
        */
        CameraSetFrameSpeed(g_hCamera,2);
    }

}





