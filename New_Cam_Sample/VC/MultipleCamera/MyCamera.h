#ifndef _MY_CAMERA_H_
#define _MY_CAMERA_H_

#include "Mv3dRgbdApi.h"
#include "Mv3dRgbdDefine.h"
#include "Mv3dRgbdImgProc.h"
#include "BasicDemo.h"


#define EXPOSURE_TIME                    "ExposureTime"
#define GAIN                            "Gain"
#define ACQUISITION_FRAME_RATE            "AcquisitionFrameRate"
#define ACQUISITION_FRAME_RATE_ENABLE    "AcquisitionFrameRateEnable"

#define WORKING_MODE "CameraWorkingMode"
#define IMG_MODE "ImageMode"

#define CHUNK_MODE_ACTIVE "ChunkModeActive" 
#define CHUNK_ENABLE "ChunkEnable"
#define CHUNK_SELECTOR "ChunkSelector"

#define Camera_Width             "Width"
#define Camera_Height            "Height"

typedef struct _MV3D_RGBD_DRAW_PARAM_
{
    HDC hDC;
    unsigned char *pData;

    int nImageWidth;
    int nImageHeight;

    int nWndRectWidth;
    int nWndRectHeight;
    int nDstX;
    int nDstY;
}MV3D_RGBD_DRAW_PARAM;


class CMyCamera
{
public:
    CMyCamera();
    ~CMyCamera();


    //图像模式
    enum MV3D_IMG_MODE
    {
        PROFILE_DATA = 0,//轮廓数据
        DEPTH_DATA,//深度数据
        INTENSITY_DATA,//亮度数据
    };

    enum Mv3dLpImageMode
    {
        MV3D_RGBD_Origin_Image                     =   1,
        MV3D_RGBD_Point_Cloud_Image                =   4,
        MV3D_RGBD_Range_Image                      =   7,
    };


    int     Open(char* chSerialNumber);
    int     Close();
    int     StartGrabbing();
    int     StopGrabbing();
    int        SoftTrigger();
    int     Process(HWND hDisplay);
    int     SaveRAW();

    int        Draw(MV3D_RGBD_DRAW_PARAM* pstParam);
    int        Display(void* handle, void* hWnd, MV3D_RGBD_IMAGE_DATA* pstDisplayImage);
private:
    static void*  __stdcall WINAPI ProcessThread(void* pUser);            // 图像显示线程
public:
    void*           m_handle;
    char*            m_chSerialNumber;
    int        m_nImgMode;                //图像模式
private:
    BOOL            m_bStartJob;              // 是否工作线程已开启
    HANDLE          m_hProcessThread;         // 取流线程
    HWND            m_hWndDisplay;
    int             m_MaxImageSize;                     // 图像最大尺寸
    unsigned char*  m_pcDataBuf;                        // 存储图像数据
    CCriticalSection        m_criSection;                       // 临界区
    MV3D_RGBD_IMAGE_DATA           m_stImageInfo;
    BITMAPINFO*             m_bBitmapInfo;
};


#endif