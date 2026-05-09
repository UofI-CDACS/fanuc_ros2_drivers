#include <iostream>
#include <gtk/gtk.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>

#include "CameraApi.h"
#include "interface.h"
#include "Demo.h"
using namespace std;

#define     GLADE_NAME      "multi_camera_demo.glade"


//SDK
int                     g_hCamera[DISPLAY_MAX] = {-1,-1};        //设备句柄
unsigned char           * g_pRawBuffer[DISPLAY_MAX];        //raw数据
unsigned char           * g_pRgbBuffer[DISPLAY_MAX];        //处理后数据缓存区
tSdkFrameHead           g_tFrameHead[DISPLAY_MAX];          //图像帧头信息
tSdkCameraCapbility     g_tCapability[DISPLAY_MAX];         //设备描述信息





//gtk demo使用
GtkBuilder              *Demo_builder =NULL;            //glade进行界面设计
GtkWidget               *main_window = NULL;            //主窗体
GtkWidget               *display_drawingarea[DISPLAY_MAX]={NULL,NULL};    //画板

Width_Height            g_W_H_INFO[DISPLAY_MAX];            //显示画板到大小和图像大小
BYTE                    *g_readBuf[DISPLAY_MAX]={NULL,NULL};            //画板显示数据区
pthread_t               g_thread_id[DISPLAY_MAX];           //线程

int                     g_display_state[DISPLAY_MAX]={0,0};      //显示状态  1读取数据 0退出
int                     g_read_fps[DISPLAY_MAX]={0,0};           //统计读取帧率
int                     g_disply_fps[DISPLAY_MAX]={0,0};         //统计显示帧率
int                     g_display_max=0;                        //最大显示


typedef void *(*preview_thread_Func)(void* arg);

int main (int argc, char **argv)
{
    int                     iCameraCounts = 4;
    int                     iStatus1=-1;
    int                     iStatus2=-1;
    int                     i=0;
    tSdkCameraDevInfo       tCameraEnumList[iCameraCounts];
    void                    (* preview_thread[DISPLAY_MAX])(void* arg);
    preview_thread_Func     thread_func[DISPLAY_MAX];
    GtkWidget                   *camera_label[2];

    thread_func[0]=preview_thread1;
    thread_func[1]=preview_thread2;

	if(!g_thread_supported())
	{
	g_thread_init(NULL);
	}
	gdk_threads_init();

	gtk_init(&argc,&argv);

	Demo_builder = gtk_builder_new();
    gtk_builder_add_from_file(Demo_builder, GLADE_NAME, NULL);




    //sdk初始化  0 English 1中文
    CameraSdkInit(1);

    //枚举设备，并建立设备列表
    CameraEnumerateDevice(tCameraEnumList,&iCameraCounts);

    //没有连接设备
    if(iCameraCounts==0){
        return -1;
    }

    g_display_max=iCameraCounts;
    if(iCameraCounts>DISPLAY_MAX){
        g_display_max=DISPLAY_MAX;
    }

    //创建窗体构建
    main_window=create_gtkdemo_window();

    camera_label[0]  = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "camera_label1"));
    camera_label[1]  = GTK_WIDGET(gtk_builder_get_object(Demo_builder, "camera_label2"));


    for(i=0;i<g_display_max;i++){
        //相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
        iStatus1 = CameraInit(&tCameraEnumList[i],-1,-1,&g_hCamera[i]);

        gtk_label_set_text((GtkLabel      *)camera_label[i],tCameraEnumList[i].acProductName);

        //初始化失败
        if(iStatus1!=CAMERA_STATUS_SUCCESS){
             continue;
        }
        g_display_state[i]=1;
        //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
        CameraGetCapability(g_hCamera[i],&g_tCapability[i]);

        g_pRgbBuffer[i] = (unsigned char*)malloc(g_tCapability[i].sResolutionRange.iHeightMax*g_tCapability[i].sResolutionRange.iWidthMax*3);
        g_readBuf[i] = (unsigned char*)malloc(g_tCapability[i].sResolutionRange.iHeightMax*g_tCapability[i].sResolutionRange.iWidthMax*3);

        /*让SDK进入工作模式，开始接收来自相机发送的图像
        数据。如果当前相机是触发模式，则需要接收到
        触发帧以后才会更新图像。    */
        CameraPlay(g_hCamera[i] );
    /*
        设置图像处理的输出格式，彩色黑白都支持RGB24位
    */

        if(g_tCapability[i].sIspCapacity.bMonoSensor){
            CameraSetIspOutFormat(g_hCamera[i],CAMERA_MEDIA_TYPE_MONO8);
        }else{
            CameraSetIspOutFormat(g_hCamera[i],CAMERA_MEDIA_TYPE_RGB8);
        }
        CameraSetAeState(g_hCamera[i],FALSE);
        //根据设备设置参数初始化界面
        Gtk_SetResolution(display_drawingarea[i] ,&g_W_H_INFO[i] ,g_hCamera[i] ,&g_tCapability[i] );

        //创建preview 读取相机每帧数据
        pthread_create(&g_thread_id[i], NULL, thread_func[i], NULL);
    }

    gtk_widget_show (main_window);

	gdk_threads_enter();
	gtk_main();
	gdk_threads_leave();

    return 0;
}



