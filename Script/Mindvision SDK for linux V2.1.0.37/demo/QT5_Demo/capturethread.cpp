#include "capturethread.h"
#include "mainwindow.h"
#include <QDebug>

#include "CameraApi.h"

//SDK使用
extern int                  g_hCamera;          //设备句柄
extern unsigned char        * g_pRgbBuffer;     //处理后数据缓存区
extern tSdkFrameHead        g_tFrameHead;       //图像帧头信息
extern tSdkCameraCapbility  g_tCapability;      //设备描述信息

extern int                  g_read_fps;         //统计帧率
extern int                  g_SaveImage_type;   //保存图像格式

CaptureThread::CaptureThread(QObject *parent) :
    QThread(parent)
{
    pause_status = true;
    quit = false;

    for(int i = 0; i < 256; i++)
    {
       grayColourTable.append(qRgb(i, i, i));
    }
}

void CaptureThread::run()
{
    BYTE* pRawBuffer;

    forever
    {
        if(!pause_status)
        {
            if(quit) break;
            if (CameraGetImageBuffer(g_hCamera,&g_tFrameHead,&pRawBuffer,200) == CAMERA_STATUS_SUCCESS)
            {
                CameraImageProcess(g_hCamera,pRawBuffer,g_pRgbBuffer,&g_tFrameHead);
                CameraReleaseImageBuffer(g_hCamera,pRawBuffer);

                if(g_tFrameHead.uiMediaType==CAMERA_MEDIA_TYPE_MONO8){
                    QImage img(g_pRgbBuffer, g_tFrameHead.iWidth, g_tFrameHead.iHeight,QImage::Format_Indexed8);
                    img.setColorTable(grayColourTable);
                    emit captured(img);

                }else{
                    QImage img(g_pRgbBuffer, g_tFrameHead.iWidth, g_tFrameHead.iHeight,QImage::Format_RGB888);
                    emit captured(img);
                }

                g_read_fps++;//统计抓取帧率

            }else{
                //printf("timeout \n");
                usleep(1000);
            }


        } else usleep(1000);
        if(quit) break;
    }
}

void CaptureThread::stream()
{
    pause_status = false;
}

void CaptureThread::pause()
{
    pause_status = true;
}

void CaptureThread::stop()
{
    pause_status = false;
    quit = true;
}

