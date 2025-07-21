#include "stdafx.h"
#include "MyCamera.h"
#include <string.h>
#include "afxwin.h"

void    DebugInfo(char *szFormat, ...)
{
#ifdef WIN32

    char szInfo[8192];
    va_list ArgumentList;

    va_start(ArgumentList, szFormat); 
    vsprintf_s(szInfo, 8192, szFormat, ArgumentList);
    va_end(ArgumentList);

    OutputDebugStringA(szInfo);
#endif
}

// 渲染线程
void*  __stdcall CMyCamera::ProcessThread(void* pUser)
{
    int nRet = MV3D_RGBD_OK;

    CMyCamera* pThis = (CMyCamera*)pUser;
    if (NULL == pThis)
    {
        return NULL;
    }

    MV3D_RGBD_FRAME_DATA stFrameData = {0};
    while (pThis->m_bStartJob)
    {
        // 获取图像数据
        nRet = MV3D_RGBD_FetchFrame(pThis->m_handle, &stFrameData, 5000);
        if (MV3D_RGBD_OK ==  nRet)
        {
            try
            {
                nRet = pThis->Display(pThis->m_handle,pThis->m_hWndDisplay, stFrameData.stImageData);
                //nRet = pThis->Display(pThis->m_handle,pThis->m_hWndDisplay, &stFrameData.stImageData[1]); // RGB图
                if (MV3D_RGBD_OK != nRet)
                {
                    throw nRet;
                }
            }
            catch (...)
            {
                printf("ERROR  !\r\n");
            }
        }
        else
        {
            Sleep(1);
            continue;
        }
    }
    printf("stop recv  !\r\n");
    return 0;
}


CMyCamera::CMyCamera()
{
    m_handle = NULL;
    m_MaxImageSize = 0;
    m_pcDataBuf = NULL;
    m_hWndDisplay = NULL;
    m_bStartJob = FALSE;
    m_bBitmapInfo = NULL;
    memset((void *)&m_stImageInfo, 0, sizeof(MV3D_RGBD_IMAGE_DATA));

    m_chSerialNumber = (char*)malloc(sizeof(char)*16);
    memset(m_chSerialNumber,0,(sizeof(char)*16));
}

CMyCamera::~CMyCamera()
{
    if (m_handle)
    {
        Close();
        delete m_handle;
        m_handle = NULL;
    }
}

int CMyCamera::Open(char* chSerialNumber)
{
    if (NULL ==  chSerialNumber)
    {
        return MV3D_RGBD_E_PARAMETER;
    }
    else
    {
        memcpy(m_chSerialNumber,chSerialNumber,sizeof(char)*16);
    }

    int nRet = MV3D_RGBD_OK;
    CString cstrInfo;

    if(m_handle == NULL)
    {
        nRet = MV3D_RGBD_OpenDeviceBySerialNumber(&m_handle,chSerialNumber);
        if (MV3D_RGBD_OK != nRet)
        {
            return nRet;
        }
    }

    return MV3D_RGBD_OK;
}

int CMyCamera::Close()
{
    int nRet = MV3D_RGBD_OK;
    CString cstrInfo;

    if (TRUE == m_bStartJob)
    {
        m_bStartJob = FALSE;

        // 销毁取流线程
        if (NULL != m_hProcessThread)
        {
            //等待线程结束，关闭释放线程
            WaitForSingleObject(m_hProcessThread, 1000);
            CloseHandle(m_hProcessThread);
            m_hProcessThread = NULL;
        }

        nRet = MV3D_RGBD_Stop(m_handle);
        if (MV3D_RGBD_OK != nRet)
        {
            return nRet;
        }
    }

    nRet = MV3D_RGBD_CloseDevice(&m_handle);
    if (MV3D_RGBD_OK == nRet)
    {
        m_handle = NULL;
    }

    return nRet;
}

int CMyCamera::StartGrabbing()
{
    int nRet = MV3D_RGBD_OK;
    m_bStartJob = TRUE;

    nRet = MV3D_RGBD_Start(m_handle);
    return nRet;
}

int CMyCamera::StopGrabbing()
{
    int nRet = MV3D_RGBD_OK; 
    m_bStartJob = FALSE;
    // 销毁取流线程
    if (NULL != m_hProcessThread)
    {
        //等待线程结束，关闭释放线程
        WaitForSingleObject(m_hProcessThread, 1000);
        CloseHandle(m_hProcessThread);
        m_hProcessThread = NULL;
    }

    nRet = MV3D_RGBD_Stop(m_handle);
    return nRet;

}

int    CMyCamera::SoftTrigger()
{
    int nRet = MV3D_RGBD_OK; 
    nRet = MV3D_RGBD_SoftTrigger(m_handle);
    return nRet;
}

int    CMyCamera::SaveRAW()
{
    int nRet = MV3D_RGBD_OK;
    CString cstrInfo;

    // 判断是否开始取流
    if (!m_bStartJob)
    {
        return MV3D_RGBD_E_PARAMETER;
    }

    // 判断是否有有效数据
    if (NULL == m_pcDataBuf)
    {
        return MV3D_RGBD_E_PARAMETER;
    }

    if (0 == m_stImageInfo.nFrameNum)
    {
        return MV3D_RGBD_E_NODATA;             
    }


    // 保存RAW图像
    FILE* pfile;
    char filename[256] = {0};
    CTime currTime;                                     // 获取系统时间作为保存图片文件名
    currTime = CTime::GetCurrentTime(); 
    sprintf(filename,("%s_%.4d%.2d%.2d%.2d%.2d%.2d.raw"),m_chSerialNumber, currTime.GetYear(), currTime.GetMonth(),
        currTime.GetDay(), currTime.GetHour(), currTime.GetMinute(), currTime.GetSecond());
    pfile = fopen(filename,"wb");
    if(pfile == NULL)
    {
        return MV3D_RGBD_E_PARAMETER;
    }

    {
        m_criSection.Lock();
        fwrite(m_pcDataBuf, 1, m_stImageInfo.nDataLen, pfile);
        m_criSection.Unlock();
    }

    fclose (pfile);
    pfile = NULL;

    return nRet;
}

int    CMyCamera::Process(HWND hDisplay)
{
    m_hWndDisplay = hDisplay;

    // 创建接收 处理线程
    m_hProcessThread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)ProcessThread, this, 0, NULL);
    if (NULL == m_hProcessThread)
    {
        return  MV3D_RGBD_E_RESOURCE;
    }
    
    return MV3D_RGBD_OK;
}


int  CMyCamera::Draw(MV3D_RGBD_DRAW_PARAM* pstParam)
{
    if (NULL == pstParam)
    {
        return MV3D_RGBD_E_PARAMETER;
    }   

    int nImageWidth = pstParam->nImageWidth;
    int nImageHeight = pstParam->nImageHeight;
    int nDstWidth  = (int)(pstParam->nWndRectWidth);
    int nDstHeight = (int)(pstParam->nWndRectHeight);
    int nSrcX      = 0;
    int nSrcY      = 0;
    int nSrcWidth  = (int)(nImageWidth);
    int nSrcHeight = (int)(nImageHeight);

    if (NULL == m_bBitmapInfo)
    {
        m_bBitmapInfo = (PBITMAPINFO)malloc(sizeof(BITMAPINFO) + 256 * sizeof(RGBQUAD));
        memset(m_bBitmapInfo, 0, sizeof(sizeof(BITMAPINFO) + 256 * sizeof(RGBQUAD)));
    }
    // 位图信息头
    m_bBitmapInfo->bmiHeader.biSize = sizeof(BITMAPINFOHEADER);             // BITMAPINFOHEADER结构长度
    m_bBitmapInfo->bmiHeader.biWidth = nImageWidth;                         // 图像宽度
    m_bBitmapInfo->bmiHeader.biPlanes = 1;                                  // 位面数
    m_bBitmapInfo->bmiHeader.biBitCount = 8;                                // 比特数/像素的颜色深度,2^8=256
    m_bBitmapInfo->bmiHeader.biCompression = BI_RGB;                        // 图像数据压缩类型,BI_RGB表示不压缩
    m_bBitmapInfo->bmiHeader.biSizeImage = nImageWidth * nImageHeight;      // 图像大小
    m_bBitmapInfo->bmiHeader.biHeight = - nImageHeight;                     // 图像高度

    for(int i = 0; i < 256; i++)
    {
        m_bBitmapInfo->bmiColors[i].rgbBlue = m_bBitmapInfo->bmiColors[i].rgbRed = m_bBitmapInfo->bmiColors[i].rgbGreen = i;
        m_bBitmapInfo->bmiColors[i].rgbReserved = 0;
    }

    int nRet = StretchDIBits(pstParam->hDC,
        pstParam->nDstX, pstParam->nDstY, nDstWidth, nDstHeight,
        nSrcX, nSrcY, nSrcWidth, nSrcHeight, pstParam->pData, m_bBitmapInfo, DIB_RGB_COLORS, SRCCOPY);

    return MV3D_RGBD_OK;
}


int  CMyCamera::Display(void* handle, void* hWnd, MV3D_RGBD_IMAGE_DATA* pstDisplayImage)
{
    int nRet = MV3D_RGBD_OK;

    if (NULL == pstDisplayImage)
    {
        return MV3D_RGBD_E_PARAMETER;
    }

    if (pstDisplayImage->nWidth == 0 || pstDisplayImage->nHeight == 0 || NULL == hWnd)
    {
        return MV3D_RGBD_E_PARAMETER;
    }

    // 显示图像
    HDC hDC  = ::GetDC((HWND)hWnd);
    SetStretchBltMode(hDC, COLORONCOLOR);
    RECT wndRect;
    ::GetClientRect((HWND)hWnd, &wndRect);

    {
        // 缓存下来，后期保存图片
        m_criSection.Lock();
        if (m_MaxImageSize <  pstDisplayImage->nDataLen)
        {
            if (NULL != m_pcDataBuf)
            {
                free(m_pcDataBuf);
                m_pcDataBuf = NULL;
            }

            m_MaxImageSize =  pstDisplayImage->nDataLen;
            m_pcDataBuf =  (unsigned char*)malloc(m_MaxImageSize);
            if (NULL == m_pcDataBuf)
            {
                nRet = MV3D_RGBD_E_RESOURCE;
                return nRet;
            }
            memset(m_pcDataBuf, 0, m_MaxImageSize);
        }

        memset(m_pcDataBuf, 0, m_MaxImageSize);
        memset((void *)&m_stImageInfo, 0, sizeof(MV3D_RGBD_IMAGE_DATA));

        // 保存图片信息 & 图片
        memcpy((void *)&m_stImageInfo, pstDisplayImage, sizeof(MV3D_RGBD_IMAGE_DATA));
        if (NULL != pstDisplayImage->pData)
        {
            memcpy(m_pcDataBuf, pstDisplayImage->pData, pstDisplayImage->nDataLen);
        }
        m_criSection.Unlock();
    }

    if (ImageType_Mono8 == pstDisplayImage->enImageType)
    {
        // mono8 直接渲染
        MV3D_RGBD_DRAW_PARAM stParam;    // 自己构建的结构体

        int nWndRectWidth  = wndRect.right  - wndRect.left;
        int nWndRectHeight = wndRect.bottom - wndRect.top;
        int nDstWidth  = (int)(nWndRectWidth);
        int nDstHeight = (int)(nWndRectHeight);
        int nDstX      = wndRect.left;
        int nDstY      = wndRect.top; 

        int nImageWidth = pstDisplayImage->nWidth;
        int nImageHeight = pstDisplayImage->nHeight;
        int nSrcX      = 0;
        int nSrcY      = 0;
        int nSrcWidth  = (int)(nImageWidth);
        int nSrcHeight = (int)(nImageHeight);

        // 给结构体赋值
        stParam.hDC = hDC;
        stParam.nDstX = nDstX;
        stParam.nDstY = nDstY;
        stParam.nImageHeight = nImageHeight;
        stParam.nImageWidth = nImageWidth;
        stParam.nWndRectHeight = nWndRectHeight;
        stParam.nWndRectWidth = nWndRectWidth;

        stParam.pData =  pstDisplayImage->pData;
        nRet = Draw(&stParam);
        if (MV3D_RGBD_OK != nRet)
        {
            return MV3D_RGBD_E_PARAMETER;
        }

    }
    else if (ImageType_Depth ==  pstDisplayImage->enImageType || ImageType_RGB8_Planar == pstDisplayImage->enImageType || 
        ImageType_YUV422 == pstDisplayImage->enImageType || ImageType_YUV420SP_NV12 == pstDisplayImage->enImageType || 
        ImageType_YUV420SP_NV21 == pstDisplayImage->enImageType)
    {
        nRet = MV3D_RGBD_DisplayImage(handle, pstDisplayImage, hWnd);
        if (0 != nRet)
        {
            printf("DisplayDepthOrRgbMap failed, errcode (%#x)!\r\n",nRet);
            return nRet;
        }
    }
    else
    {
        // 不支持
    }

    ::ReleaseDC((HWND)hWnd, hDC);
    return nRet;
}

