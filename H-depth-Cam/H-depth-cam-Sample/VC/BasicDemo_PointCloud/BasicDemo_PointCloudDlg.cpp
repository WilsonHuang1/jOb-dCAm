#include "stdafx.h"
#include "BasicDemo_PointCloud.h"
#include "BasicDemo_PointCloudDlg.h"
#include <string.h> 
#include <comdef.h>
#include <gdiplus.h>
#include "include/RenderImage.hpp"
#define GLFW_EXPOSE_NATIVE_WIN32
#include <GLFW/glfw3native.h>

using namespace Gdiplus;
#pragma comment( lib, "gdiplus.lib" ) 
#include <WinGDI.h> 


class CAboutDlg : public CDialog
{
public:
    CAboutDlg();

    enum { IDD = IDD_ABOUTBOX };

    protected:
    virtual void DoDataExchange(CDataExchange* pDX);  

protected:
    DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialog(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
    CDialog::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialog)
END_MESSAGE_MAP()

CBasicDemo_PointCloudDlg::CBasicDemo_PointCloudDlg(CWnd* pParent /*=NULL*/)
    : CDialog(CBasicDemo_PointCloudDlg::IDD, pParent)
    , m_bConnect(0)
    , m_bStartJob(0)
    ,m_pcDataBuf(NULL)
    ,m_bBitmapInfo(NULL)
    ,m_bCalibInfo(false)
{
    memset(&m_stDeviceInfoList, 0, sizeof(m_stDeviceInfoList));
    m_nDevNum = 0;
    m_nPointCloudType = 0;

    m_handle = NULL;
 
    m_MaxImageSize = 0;
    m_pcDataBuf = NULL;

    m_MaxRgbTransImageSize = 0;
    m_pcRgbTransDataBuf = NULL;
    
    hProcessThread = NULL;    

    m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CBasicDemo_PointCloudDlg::DoDataExchange(CDataExchange* pDX)
{
    CDialog::DoDataExchange(pDX);
    DDX_Control(pDX, IDC_OPEN_BUTTON, m_ctrlOpenButton);
    DDX_Control(pDX, IDC_CLOSE_BUTTON, m_ctrlCloseButton);
    DDX_Control(pDX, IDC_START_GRABBING_BUTTON, m_ctrlStartGrabbingButton);
    DDX_Control(pDX, IDC_STOP_GRABBING_BUTTON, m_ctrlStopGrabbingButton);
    DDX_Control(pDX, IDC_EXPOSURE_EDIT, m_ctrlExposureEdit);
    DDX_Control(pDX, IDC_GAIN_EDIT, m_ctrlGainEdit);
    DDX_Control(pDX, IDC_GET_PARAMETER_BUTTON, m_ctrlGetParameterButton);
    DDX_Control(pDX, IDC_SET_PARAMETER_BUTTON, m_ctrlSetParameterButton);
    DDX_Control(pDX, IDC_DEVICE_COMBO, m_ctrlDeviceCombo);
    DDX_Control(pDX, IDC_DEVICE_COMBO2, m_ctrlImageAlignCombo);
    DDX_Control(pDX, IDC_DEVICE_COMBO3, m_ctrlPointCloudType);
}

BEGIN_MESSAGE_MAP(CBasicDemo_PointCloudDlg, CDialog)
    ON_WM_SYSCOMMAND()
    ON_WM_PAINT()
    ON_WM_QUERYDRAGICON()
    ON_WM_MOUSEWHEEL()
    ON_WM_MOUSEMOVE()
    ON_WM_LBUTTONDOWN()
    ON_WM_LBUTTONUP()
    ON_BN_CLICKED(IDC_ENUM_BUTTON, &CBasicDemo_PointCloudDlg::OnBnClickedEnumButton)
    ON_BN_CLICKED(IDC_OPEN_BUTTON, &CBasicDemo_PointCloudDlg::OnBnClickedOpenButton)
    ON_BN_CLICKED(IDC_START_GRABBING_BUTTON, &CBasicDemo_PointCloudDlg::OnBnClickedStartGrabbingButton)
    ON_BN_CLICKED(IDC_BTN_Start, &CBasicDemo_PointCloudDlg::OnBnClickedStartGrabbingButton)
    ON_BN_CLICKED(IDC_GET_PARAMETER_BUTTON, &CBasicDemo_PointCloudDlg::OnBnClickedGetParameterButton)
    ON_BN_CLICKED(IDC_SET_PARAMETER_BUTTON, &CBasicDemo_PointCloudDlg::OnBnClickedSetParameterButton)
    ON_BN_CLICKED(IDC_STOP_GRABBING_BUTTON, &CBasicDemo_PointCloudDlg::OnBnClickedStopGrabbingButton)
    ON_BN_CLICKED(IDC_CLOSE_BUTTON, &CBasicDemo_PointCloudDlg::OnBnClickedCloseButton)
    ON_WM_CLOSE()
    ON_BN_CLICKED(IDC_SAVE_PLY_BTN, &CBasicDemo_PointCloudDlg::OnBnClickedSavePlyBtn)
    ON_BN_CLICKED(IDC_SAVE_PLY_BINARY_BTN, &CBasicDemo_PointCloudDlg::OnBnClickedSavePlyBinaryBtn)
    ON_BN_CLICKED(IDC_SAVE_PCD_BTN, &CBasicDemo_PointCloudDlg::OnBnClickedSavePcdBtn)
    ON_BN_CLICKED(IDC_SAVE_PCD_BINARY_BTN, &CBasicDemo_PointCloudDlg::OnBnClickedSavePcdBinaryBtn)
    ON_CBN_SELCHANGE(IDC_DEVICE_COMBO3, &CBasicDemo_PointCloudDlg::OnCbnSelchangeDeviceCombo3)
END_MESSAGE_MAP()

BOOL CBasicDemo_PointCloudDlg::OnInitDialog()
{
    CDialog::OnInitDialog();

    ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
    ASSERT(IDM_ABOUTBOX < 0xF000);

    CMenu* pSysMenu = GetSystemMenu(FALSE);
    if (pSysMenu != NULL)
    {
        BOOL bNameValid;
        CString strAboutMenu;
        bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
        ASSERT(bNameValid);
        if (!strAboutMenu.IsEmpty())
        {
            pSysMenu->AppendMenu(MF_SEPARATOR);
            pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
        }
    }

    SetIcon(m_hIcon, TRUE);         //ch:���ô�ͼ�� | en:Set big icon
    SetIcon(m_hIcon, FALSE);        //ch:����Сͼ�� | en:Set small icon

    GdiplusStartupInput gdiplusStartupInput;
    ULONG_PTR gdiplusToken;
    // ch:��ʼ��GDI+ | en:GDI+ Initialization
    GdiplusStartup(&gdiplusToken, &gdiplusStartupInput, NULL);
    SetCtrlWhenInit();
    return TRUE; 
}

int CBasicDemo_PointCloudDlg::InitResources()
{
    int nRet = MV3D_RGBD_OK;
    CString cstrInfo;

    try
    {
        int nSensorWidth = 0;
        int nSensorHight = 0;

        MV3D_RGBD_PARAM pstValue;
        memset(&pstValue, 0, sizeof(MV3D_RGBD_PARAM));
        
        pstValue.enParamType = ParamType_Int;
        nRet = MV3D_RGBD_GetParam(m_handle, Camera_Width, &pstValue);
        if (MV3D_RGBD_OK != nRet)
        {
            cstrInfo.Format(_T("Get width failed! err code:%#x"), nRet);
            MessageBox(cstrInfo);

            nRet = MV3D_RGBD_E_UNKNOW;
            throw nRet;
        }
        nSensorWidth = pstValue.ParamInfo.stIntParam.nMax;

        memset(&pstValue, 0, sizeof(MV3D_RGBD_PARAM));
        pstValue.enParamType = ParamType_Int;
        nRet = MV3D_RGBD_GetParam(m_handle, Camera_Height, &pstValue);
        if (MV3D_RGBD_OK != nRet)
        {
            cstrInfo.Format(_T("Get hight failed! err code:%#x"), nRet);
            MessageBox(cstrInfo);
            return nRet;
        }
        nSensorHight = pstValue.ParamInfo.stIntParam.nMax;
  
        m_MaxImageSize = nSensorWidth * nSensorHight * 3 * sizeof(float) + 4096;

        CString cstrInfo;
        m_pcDataBuf =  (unsigned char*)malloc(m_MaxImageSize);
        if (NULL == m_pcDataBuf)
        {
            nRet = MV3D_RGBD_E_RESOURCE;
            throw nRet;
        }
        memset(m_pcDataBuf, 0, m_MaxImageSize);
        memset((void *)&m_stImageInfo, 0, sizeof(MV3D_RGBD_IMAGE_DATA));

    }
    catch (...)
    {
        DeInitResources();
        return nRet;
    }

    return nRet;
}

void CBasicDemo_PointCloudDlg::DeInitResources()
{
    if (NULL != m_pcDataBuf)
    {
        free(m_pcDataBuf);
        m_pcDataBuf = NULL;
    }

    if (NULL != m_pcRgbTransDataBuf)
    {
        free(m_pcRgbTransDataBuf);
        m_pcRgbTransDataBuf = NULL;
    }
}

void CBasicDemo_PointCloudDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
    if ((nID & 0xFFF0) == IDM_ABOUTBOX)
    {
        CAboutDlg dlgAbout;
        dlgAbout.DoModal();
    }
    else
    {
        CDialog::OnSysCommand(nID, lParam);
    }
}
      
void CBasicDemo_PointCloudDlg::OnPaint()
{
    if (IsIconic())
    {
        CPaintDC dc(this); // ch:���ڻ��Ƶ��豸������ | en:device context for painting

        SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

        // ch:ʹͼ���ڹ����������о��� | en:Center icon in client rectangle
        int cxIcon = GetSystemMetrics(SM_CXICON);
        int cyIcon = GetSystemMetrics(SM_CYICON);
        CRect rect;
        GetClientRect(&rect);
        int x = (rect.Width() - cxIcon + 1) / 2;
        int y = (rect.Height() - cyIcon + 1) / 2;

        // ch:����ͼ�� | en:Draw the icon
        dc.DrawIcon(x, y, m_hIcon);
    }
    else
    {
        CDialog::OnPaint();
    }
}

HCURSOR CBasicDemo_PointCloudDlg::OnQueryDragIcon()
{
    return static_cast<HCURSOR>(m_hIcon);
}

BOOL CBasicDemo_PointCloudDlg::PreTranslateMessage(MSG* pMsg)
{
    // ch:����ESC��ENTER���� | en:Shielding ESC and ENTER key
    if (pMsg->message == WM_KEYDOWN && pMsg->wParam == VK_ESCAPE)
    {
        return TRUE;
    }
    if (pMsg->message == WM_KEYDOWN && pMsg->wParam == VK_RETURN) 
    {
        return TRUE;
    }
    else
    {
        return CDialog::PreTranslateMessage(pMsg);
    }
}

// ch:��Ⱦ�߳� | en:Display thread
void*  __stdcall CBasicDemo_PointCloudDlg::ProcessThread(void* pUser)
{
    int nRet = MV3D_RGBD_OK;

    CBasicDemo_PointCloudDlg* pThis = (CBasicDemo_PointCloudDlg*)pUser;
    if (NULL == pThis)
    {
        return NULL;
    }

    MV3D_RGBD_FRAME_DATA stFrameData = { 0 };
    MV3D_RGBD_CALIB_INFO stCalibInfo = { 0 };
    // ch:�������ھ�� | en: Creat Display Window
    CWnd *pWnd = pThis->GetDlgItem(IDC_DISPLAY_STATIC);
    if (NULL == pWnd)
    {
        return NULL;
    }

    CRect rect;
    pWnd->GetWindowRect(&rect);
    RenderImgWnd pointCloudViewer(rect.Width(), rect.Height(), "pointcloud");

    //ȡ��glfw���ھ��������Ƕ�븸����
    HWND hwndGLFW = glfwGetWin32Window(pointCloudViewer.win);
    SetWindowLong(hwndGLFW, GWL_STYLE, WS_VISIBLE);
    ::MoveWindow(hwndGLFW, 0, 0, rect.Width(), rect.Height(), TRUE);
    ::SetParent(hwndGLFW, pWnd->GetSafeHwnd());
    pointCloudViewer.Init3DRender();
    glfw_state stAppState;
    while (pThis->m_bStartJob)
    {
        // ch:��ȡͼ������ | en:Get image data
        nRet = MV3D_RGBD_FetchFrame(pThis->m_handle, &stFrameData, 5000);
        if (MV3D_RGBD_OK ==  nRet)
        {
            BOOL bUvMap = FALSE;
            BOOL bTexture = FALSE;
            RIFrameInfo pointCloud = { 0 };
            RIFrameTexInfo TexInfo = { 0 };
            MV3D_RGBD_IMAGE_DATA* pstPointCloudImage = NULL;
            try
            {
                for(int i=0; i < stFrameData.nImageCount;i++)
                {
                    // ch:������������ | en: Parse point cloud data
                    if (ImageType_PointCloud == stFrameData.stImageData[i].enImageType)
                    {
                        {
                            // ch:�������������ڱ���ͼƬ | en: Copy the image buffer for saving image
                            pThis->m_criSection.Lock();
                            if (NULL == pThis->m_pcDataBuf || pThis->m_MaxImageSize <  stFrameData.stImageData[i].nDataLen)
                            {
                                if (NULL != pThis->m_pcDataBuf)
                                {
                                    free(pThis->m_pcDataBuf);
                                    pThis->m_pcDataBuf = NULL;
                                }

                                pThis->m_MaxImageSize = stFrameData.stImageData[i].nDataLen;
                                pThis->m_pcDataBuf = (unsigned char*)malloc(pThis->m_MaxImageSize);
                                if (NULL == pThis->m_pcDataBuf)
                                {
                                    nRet = MV3D_RGBD_E_RESOURCE;
                                    throw nRet;
                                }
                                memset(pThis->m_pcDataBuf, 0, pThis->m_MaxImageSize);
                            }

                            memset(pThis->m_pcDataBuf, 0, pThis->m_MaxImageSize);
                            memset((void *)&(pThis->m_stImageInfo), 0, sizeof(MV3D_RGBD_IMAGE_DATA));

                            // ch:����ͼƬ��Ϣ & ͼƬ | en: Copy image infomation and buffer
                            memcpy((void *)&(pThis->m_stImageInfo), &stFrameData.stImageData[i], sizeof(MV3D_RGBD_IMAGE_DATA));
                            if (NULL != stFrameData.stImageData[i].pData)
                            {
                                memcpy(pThis->m_pcDataBuf, stFrameData.stImageData[i].pData, stFrameData.stImageData[i].nDataLen);
                            }
                            pThis->m_stImageInfo.pData = pThis->m_pcDataBuf;
                            pThis->m_criSection.Unlock();
                        }

                        pstPointCloudImage = &stFrameData.stImageData[i];
                        pointCloud.enPixelType = (RenderImage::RIPixelType)pstPointCloudImage->enImageType;
                        pointCloud.nFrameNum = pstPointCloudImage->nFrameNum;
                        pointCloud.nHeight = pstPointCloudImage->nHeight;
                        pointCloud.nWidth = pstPointCloudImage->nWidth;
                        pointCloud.nFrameLength = pstPointCloudImage->nDataLen;
                        pointCloud.pData = pstPointCloudImage->pData;
                        
                        if (pThis->m_nPointCloudType == 0)
                        {
                            pointCloudViewer.RenderPointCloud(pointCloud);
                        }
                        else
                        {
                            // ch:��ȡ�궨��Ϣ | en:Get calib info
                            nRet = MV3D_RGBD_GetCalibInfo(pThis->m_handle, stFrameData.stImageData[i].enCoordinateType, &stCalibInfo);
                            if (MV3D_RGBD_OK != nRet)
                            {
                                throw false;
                            }
                            // ch:��ȡuv���� | en:Get uv coordinate
                            MV3D_RGBD_UV_DATA stUvMap;
                            nRet = MV3D_RGBD_MapPointCloudToUV(&stFrameData.stImageData[i], &stCalibInfo, &stUvMap);
                            if (MV3D_RGBD_OK == nRet)
                            {
                                TexInfo.pTexData = (float*)stUvMap.pData;
                                bUvMap = TRUE;
                            }
                            else
                            {
                                throw false;
                            }
                        }    
                    }
                    // ch:������ɫͼ�������� | en: Parse color texture data
                    if (ImageType_RGB8_Planar == stFrameData.stImageData[i].enImageType)
                    {
                        TexInfo.pRgbData = stFrameData.stImageData[i].pData;
                        TexInfo.nWidth = stFrameData.stImageData[i].nWidth;
                        TexInfo.nHeight = stFrameData.stImageData[i].nHeight;
                        TexInfo.enPixelType = (RIPixelType)ImageType_RGB8_Planar;
                        TexInfo.nFrameNum = stFrameData.stImageData[i].nFrameNum;
                        bTexture = TRUE;
                    }
                    if (ImageType_YUV422 == stFrameData.stImageData[i].enImageType)
                    {
                        int nWidth = stFrameData.stImageData[i].nWidth;
                        int nHeight = stFrameData.stImageData[i].nHeight;
                        const long len = nWidth * nHeight;
                        if (NULL == pThis->m_pcRgbTransDataBuf || pThis->m_MaxRgbTransImageSize <  len * 3)
                        {
                            if (NULL != pThis->m_pcRgbTransDataBuf)
                            {
                                free(pThis->m_pcRgbTransDataBuf);
                                pThis->m_pcRgbTransDataBuf = NULL;
                            }

                            pThis->m_MaxRgbTransImageSize = len * 3;
                            pThis->m_pcRgbTransDataBuf = (unsigned char*)malloc(pThis->m_MaxRgbTransImageSize);
                            if (NULL == pThis->m_pcRgbTransDataBuf)
                            {
                                nRet = MV3D_RGBD_E_RESOURCE;
                                throw nRet;
                            }
                            memset(pThis->m_pcRgbTransDataBuf, 0, pThis->m_MaxRgbTransImageSize);
                        }
                        memset(pThis->m_pcRgbTransDataBuf, 0, pThis->m_MaxRgbTransImageSize);

                        unsigned char * pData = stFrameData.stImageData[i].pData;

                        int bgr[3];
                        int yIdx, uIdx, vIdx;
                        for (int y = 0; y < nHeight; y++)
                        {
                            for (int x = 0; x < nWidth; x++) 
                            {
                                yIdx = 2 * ((y * nWidth) + x);
                                uIdx = 4 * (((y * nWidth) + x) >> 1) + 1;
                                vIdx = 4 * (((y * nWidth) + x) >> 1) + 3;

                                bgr[0] = (int)(pData[yIdx] + 1.732446 * (pData[uIdx] - 128));
                                bgr[1] = (int)(pData[yIdx] - 0.698001 * (pData[uIdx] - 128) - 0.703125 * (pData[vIdx] - 128));
                                bgr[2] = (int)(pData[yIdx] + 1.370705  * (pData[vIdx] - 128));

                                for (int k = 0; k < 3; k++) 
                                {
                                    if (bgr[k] >= 0 && bgr[k] <= 255)
                                        bgr[k] = bgr[k];
                                    else
                                        bgr[k] = (bgr[k] < 0) ? 0 : 255;
                                }

                                pThis->m_pcRgbTransDataBuf[y * nWidth + x] = bgr[2];
                                pThis->m_pcRgbTransDataBuf[y * nWidth + x + len] = bgr[1];
                                pThis->m_pcRgbTransDataBuf[y * nWidth + x + len * 2] = bgr[0];
                                
                            }
                        }

                        TexInfo.pRgbData = pThis->m_pcRgbTransDataBuf;
                        TexInfo.nWidth = stFrameData.stImageData[i].nWidth;
                        TexInfo.nHeight = stFrameData.stImageData[i].nHeight;
                        TexInfo.enPixelType = (RIPixelType)ImageType_RGB8_Planar;
                        TexInfo.nFrameNum = stFrameData.stImageData[i].nFrameNum;
                        bTexture = TRUE;
                    }
                }

                if (pThis->m_nPointCloudType == 1)
                {
                    // ch:��Ⱦ������� | en: Display textured point cloud
                    if (bUvMap && bTexture)
                    {
                        // ����������Ϣ
                        stAppState.tex.upload(&TexInfo);
                        // ��Ⱦ�������
                        pointCloudViewer.RenderUVPointCloud(pointCloud, TexInfo, stAppState);
                    }
                }
            }
            catch (...)
            {
                printf("ERROR  !\r\n");
            }
        }
        else
        {
            if (!pointCloudViewer)
            {
                break;
            }
            Sleep(1);
            continue;
        }
    }
    printf("stop recv  !\r\n");
    return NULL;
}

int  CBasicDemo_PointCloudDlg::Draw(MV3D_RGBD_DRAW_PARAM* pstParam)
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
    // ch:λͼ��Ϣͷ | en:Butmap header
    m_bBitmapInfo->bmiHeader.biSize = sizeof(BITMAPINFOHEADER);             // ch:BITMAPINFOHEADER�ṹ���� | en:| en:Butmap header size
    m_bBitmapInfo->bmiHeader.biWidth = nImageWidth;                         // ch:ͼ���� | en:Image width
    m_bBitmapInfo->bmiHeader.biPlanes = 1;                                  // ch:λ���� | en: Plane numbers
    m_bBitmapInfo->bmiHeader.biBitCount = 8;                                // ch:������/���ص���ɫ���,2^8=256 | en:Bit count
    m_bBitmapInfo->bmiHeader.biCompression = BI_RGB;                        // ch:ͼ������ѹ������,BI_RGB��ʾ��ѹ�� | en:Image data compression type, BI_RGB indicates no compression
    m_bBitmapInfo->bmiHeader.biSizeImage = nImageWidth * nImageHeight;      // ch:ͼ���С | en: Image data size
    m_bBitmapInfo->bmiHeader.biHeight = - nImageHeight;                     // ch:ͼ��߶� | en: Image height

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

void CBasicDemo_PointCloudDlg::OnBnClickedEnumButton()
{
    m_ctrlDeviceCombo.ResetContent();
    
    int nRet = MV3D_RGBD_OK;
    CString cstrInfo;
    
    nRet = MV3D_RGBD_GetDeviceNumber(DeviceType_Ethernet | DeviceType_USB | DeviceType_Ethernet_Vir | DeviceType_USB_Vir, &m_nDevNum);
    if (MV3D_RGBD_OK != nRet)
    {
        cstrInfo.Format(_T("MV3D_RGBD_GetDeviceNumber failed! err code:%#x"), nRet);
        MessageBox(cstrInfo);
        return;
    }

    // ch:�����豸 | en: Search device
    memset(&m_stDeviceInfoList, 0, sizeof(m_stDeviceInfoList));

    nRet = MV3D_RGBD_GetDeviceList(DeviceType_Ethernet | DeviceType_USB | DeviceType_Ethernet_Vir | DeviceType_USB_Vir,&m_stDeviceInfoList[0], 20, &m_nDevNum);  //ch:���20 | en:Max number 20
    if (MV3D_RGBD_OK != nRet)
    {
        cstrInfo.Format(_T("MV3D_RGBD_GetDeviceList failed! err code:%#x"), nRet);
        MessageBox(cstrInfo);
        return;
    }
    
    // ch:��ֵ���뵽��Ϣ�б���в���ʾ���� | en:Add value to the information list box and display
    for (unsigned int i = 0; i < m_nDevNum; i++)
    { 
        if (DeviceType_Ethernet == m_stDeviceInfoList[i].enDeviceType || DeviceType_Ethernet_Vir == m_stDeviceInfoList[i].enDeviceType)
        {
            char  pCurrentIP[16] = ""; 
            memcpy(pCurrentIP,m_stDeviceInfoList[i].SpecialInfo.stNetInfo.chCurrentIp,strlen((const char*)m_stDeviceInfoList[i].SpecialInfo.stNetInfo.chCurrentIp));

            cstrInfo.Format(_T("Index[%d] Name[%s] IP[%s] SerialNum[%s]"), i, CStringW(m_stDeviceInfoList[i].chModelName), CStringW(pCurrentIP), CStringW(m_stDeviceInfoList[i].chSerialNumber));
        }
        else if (DeviceType_USB == m_stDeviceInfoList[i].enDeviceType || DeviceType_USB_Vir == m_stDeviceInfoList[i].enDeviceType)
        {
            cstrInfo.Format(_T("Index[%d] Name[%s] UsbProtocol[%d] SerialNum[%s]"), i, CStringW(m_stDeviceInfoList[i].chModelName), m_stDeviceInfoList[i].SpecialInfo.stUsbInfo.enUsbProtocol, CStringW(m_stDeviceInfoList[i].chSerialNumber));
        }

        m_ctrlDeviceCombo.AddString(cstrInfo);
    }

    m_ctrlDeviceCombo.SetCurSel(0);
    UpdateData(FALSE);

    GetDlgItem(IDC_OPEN_BUTTON)->EnableWindow(TRUE);
}

void CBasicDemo_PointCloudDlg::OnBnClickedOpenButton()
{
    int nRet = MV3D_RGBD_OK;
    CString cstrInfo;

    UpdateData(TRUE);

    if (true == m_bConnect)
    {
        cstrInfo.Format(_T("The camera is already connect"));
        MessageBox(cstrInfo);
        return ;
    }

    if (0 == m_nDevNum)
    {
        cstrInfo.Format(_T("Please discovery device first"));
        MessageBox(cstrInfo);
        return ;
    }

    if (m_handle)
    {
        MV3D_RGBD_CloseDevice(&m_handle);
        m_handle = NULL;
    }

    m_ctrlImageAlignCombo.ResetContent();
    m_ctrlPointCloudType.ResetContent();

    m_ctrlImageAlignCombo.AddString(CStringW("Depth"));
    m_ctrlImageAlignCombo.AddString(CStringW("RGB"));

    m_ctrlPointCloudType.AddString(CStringW("point cloud"));
    m_ctrlPointCloudType.AddString(CStringW("textured point cloud"));
             
    // ch:��ȡ��ǰѡ����豸��Ϣ | en:Get the currently selected device information
    int nIndex = m_ctrlDeviceCombo.GetCurSel();

    // ch:���豸 | en:Open device
    nRet = MV3D_RGBD_OpenDevice(&m_handle, &m_stDeviceInfoList[nIndex]);
    if (MV3D_RGBD_OK != nRet)
    {
        cstrInfo.Format(_T("Create handle failed! err code:%#x"), nRet);
        MessageBox(cstrInfo);
        return ;
    }

    m_ctrlImageAlignCombo.SetCurSel(0);
    m_ctrlPointCloudType.SetCurSel(0);
    m_nPointCloudType = 0;

    // ch:���õ�������ڵ� | en:Set point cloud output node
    MV3D_RGBD_PARAM stParam;
    stParam.enParamType = ParamType_Enum;
    stParam.ParamInfo.stEnumParam.nCurValue = PointCloudType_Common;
    nRet = MV3D_RGBD_SetParam(m_handle, MV3D_RGBD_ENUM_POINT_CLOUD_OUTPUT, &stParam);

    if (MV3D_RGBD_OK != nRet)
    {
        cstrInfo.Format(_T("Set point cloud output failed! err code:%#x"), nRet);
        MessageBox(cstrInfo);
        return ;
    }
    
    // ch:��ȡ���� | en:Get parameter
    OnBnClickedGetParameterButton();
    m_bConnect = true;

    // ch:��ʼ����Դ | en:Init resources
    InitResources();
    SetCtrlWhenOpen();
}

void CBasicDemo_PointCloudDlg::OnBnClickedCloseButton()
{
    int nRet = MV3D_RGBD_OK;
    CString cstrInfo;

    // ch:�����豸��� | en:Destory device handle 
    if (NULL != m_handle)
    {
        // ch:ֹͣ�������� | en:Stop grabbing
        if (true ==  m_bStartJob)
        {
            m_bStartJob = false;

            nRet = MV3D_RGBD_Stop(m_handle);
            if (MV3D_RGBD_OK != nRet)
            {
                cstrInfo.Format(_T("Stop grabbing failed! err code:%#x"), nRet);
                MessageBox(cstrInfo);
                m_bStartJob = false;
                return;
            }

            // ch:�ȴ���Ⱦ�߳���ȫֹͣ en:Wait for the display thread complete stop
            if (NULL != hProcessThread)
            {
                WaitForSingleObject(hProcessThread, INFINITE);
                // ch:���رվ�� | en: Close the display thread handle
                CloseHandle(hProcessThread);
                hProcessThread = NULL;
            }
        }

        // ch:��ղ���ͼƬ | en:Clear diaplay image 
        GetDlgItem(IDC_DISPLAY_STATIC)->ShowWindow(FALSE);
        GetDlgItem(IDC_DISPLAY_STATIC)->ShowWindow(TRUE);

        if (NULL != m_handle)
        {
            nRet = MV3D_RGBD_CloseDevice(&m_handle);
            if (MV3D_RGBD_OK != nRet)
            {
                cstrInfo.Format(_T("Destroy handle failed! err code:%#x"), nRet);
                MessageBox(cstrInfo);
                return ;
            }
            m_handle = NULL;
        }

        m_ctrlImageAlignCombo.ResetContent();
        m_ctrlPointCloudType.ResetContent();
    }

    // ch:������Դ | en:Deinit resource
    DeInitResources();
    SetCtrlWhenClose();
}

void CBasicDemo_PointCloudDlg::OnBnClickedStartGrabbingButton()
{
    int nRet = MV3D_RGBD_OK;
    CString cstrInfo;

    // ch:�ȴ���Ⱦ�߳���ȫֹͣ en:Wait for the display thread complete stop
    if (NULL != hProcessThread)
    {
        m_bStartJob = false;
        WaitForSingleObject(hProcessThread,INFINITE);
        // ch:���رվ�� | en: Close the display thread handle
        CloseHandle(hProcessThread);
        hProcessThread = NULL;
    }

    int nImageAlignIndex = m_ctrlImageAlignCombo.GetCurSel();
    MV3D_RGBD_PARAM pstValue;
    memset(&pstValue, 0, sizeof(MV3D_RGBD_PARAM));
    pstValue.enParamType = ParamType_Int;
    pstValue.ParamInfo.stIntParam.nCurValue = nImageAlignIndex;
    nRet = MV3D_RGBD_SetParam(m_handle, MV3D_RGBD_INT_IMAGEALIGN, &pstValue);
    if (MV3D_RGBD_OK != nRet)
    {
        cstrInfo.Format(_T("Set IMAGEALIGN failed! err code:%#x"), nRet);
        MessageBox(cstrInfo);
        return ;
    }

    // ch:��ʼ�������� | en: Start grabbing
    nRet = MV3D_RGBD_Start(m_handle);
    if (MV3D_RGBD_OK != nRet)
    {
        cstrInfo.Format(_T("Start grabbing failed! err code:%#x"), nRet);
        MessageBox(cstrInfo);
        return ;
    }

    m_bStartJob = true;
    m_bCalibInfo = false;

    // ch:�����߳� | en:Creat handle
    hProcessThread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)ProcessThread, this, 0, NULL);
    if (NULL == hProcessThread)
    {
        cstrInfo.Format(_T("Create proccess Thread failed "), 0);
        MessageBox(cstrInfo);
    }

    SetCtrlWhenStart();
}

void CBasicDemo_PointCloudDlg::OnBnClickedStopGrabbingButton()
{
    int nRet = MV3D_RGBD_OK;
    CString cstrInfo;

    m_bStartJob = false;
    // ch:ֹͣ�������� | en:Stop grabbing
    nRet = MV3D_RGBD_Stop(m_handle);
    if (MV3D_RGBD_OK != nRet)
    {
        cstrInfo.Format(_T("Stop grabbing failed! err code:%#x"), nRet);
        MessageBox(cstrInfo);
        m_bStartJob = false;
        return ;
    }
    // ch:�ȴ���Ⱦ�߳���ȫֹͣ en:Wait for the display thread complete stop
    if (NULL != hProcessThread)
    {
        WaitForSingleObject(hProcessThread, INFINITE);
        // ch:���رվ�� | en: Close the display thread handle
        CloseHandle(hProcessThread);
        hProcessThread = NULL;
    }
    SetCtrlWhenStop();
}

void CBasicDemo_PointCloudDlg::OnBnClickedGetParameterButton()
{
    int nRet = MV3D_RGBD_OK;
    CString cstrInfo;
    MV3D_RGBD_PARAM pstValue;
    bool bHasError = false;
    // ch:��ȡ�ع�ʱ�� | en: Get exposure time
    float fExposureTime = 0.0f;
    memset(&pstValue, 0, sizeof(MV3D_RGBD_PARAM));
    pstValue.enParamType = ParamType_Float;
    nRet = MV3D_RGBD_GetParam(m_handle, EXPOSURE_TIME, &pstValue);
    if (MV3D_RGBD_OK != nRet)
    {
        bHasError = true;
        cstrInfo.Format(_T("Get exposure time failed! err code:%#x"), nRet);
        MessageBox(cstrInfo);
    }
    else
    {
        fExposureTime = pstValue.ParamInfo.stFloatParam.fCurValue;
        cstrInfo.Format(_T("%0.2f"), fExposureTime);
        m_ctrlExposureEdit.SetWindowText(cstrInfo);
    }

    // ch:��ȡ���� | en: Get gain
    float fGain= 0.0f;
    memset(&pstValue, 0, sizeof(MV3D_RGBD_PARAM));
    pstValue.enParamType = ParamType_Float;
     nRet = MV3D_RGBD_GetParam(m_handle, GAIN, &pstValue);
    if (MV3D_RGBD_OK != nRet)
    {
        bHasError = true;
        cstrInfo.Format(_T("Get gain failed! err code:%#x"), nRet);
        MessageBox(cstrInfo);
    }
    else
    {
        fGain = pstValue.ParamInfo.stFloatParam.fCurValue;
        cstrInfo.Format(_T("%0.2f"), fGain);
        m_ctrlGainEdit.SetWindowText(cstrInfo);
    }

    if (m_bConnect && !bHasError)
    {
        cstrInfo.Format(_T("Get Para Success!"));
        MessageBox(cstrInfo);
    }

    UpdateData(FALSE);
}


void CBasicDemo_PointCloudDlg::OnBnClickedSetParameterButton()
{
    UpdateData(TRUE);
    bool bHasError = false;

    int nRet = MV3D_RGBD_OK;
    CString cstrInfo;
    MV3D_RGBD_PARAM pstValue;

    // ch:�����ع�ʱ�� | en:Set exposure time
    float fExposureTime = 0.0f;
    m_ctrlExposureEdit.GetWindowText(cstrInfo);
    fExposureTime = atof(CStringA(cstrInfo));
    memset(&pstValue, 0, sizeof(MV3D_RGBD_PARAM));
    pstValue.enParamType = ParamType_Float;
    pstValue.ParamInfo.stFloatParam.fCurValue = fExposureTime;
    nRet = MV3D_RGBD_SetParam(m_handle, EXPOSURE_TIME, &pstValue);
    if (MV3D_RGBD_OK != nRet)
    {
        bHasError = true;
        cstrInfo.Format(_T("Set exposure time failed! err code:%#x"), nRet);
        MessageBox(cstrInfo);
    }

    // ch:�������� | en:Set gain
    float fGain= 0.0f;
    m_ctrlGainEdit.GetWindowText(cstrInfo);
    fGain = atof(CStringA(cstrInfo));
    memset(&pstValue, 0, sizeof(MV3D_RGBD_PARAM));
    pstValue.enParamType = ParamType_Float;
    pstValue.ParamInfo.stFloatParam.fCurValue = fGain;
    nRet = MV3D_RGBD_SetParam(m_handle, GAIN, &pstValue);
    if (MV3D_RGBD_OK != nRet)
    {
        bHasError = true;
        cstrInfo.Format(_T("Set gain failed! err code:%#x"), nRet);
        MessageBox(cstrInfo);
    }

    if (false == bHasError)
    {
        cstrInfo.Format(_T("Set Para Success!"));
        MessageBox(cstrInfo);
    }
}

void CBasicDemo_PointCloudDlg::OnClose()
{
    // ch:�رճ���ִ�жϿ���������پ������ | en: Close the program, perform close device and deinit resources
    PostQuitMessage(0);
    CloseDevice();

    DeInitResources();
    
    CDialog::OnClose();
}

int  CBasicDemo_PointCloudDlg::CloseDevice(void)
{
    if (m_handle)
    {
        MV3D_RGBD_CloseDevice(&m_handle);
        m_handle = NULL;
    }

    m_bConnect = FALSE;
    m_bStartJob = FALSE;

    return MV3D_RGBD_OK;
}

void    CBasicDemo_PointCloudDlg::SetCtrlWhenInit()
{
    GetDlgItem(IDC_OPEN_BUTTON)->EnableWindow(FALSE);
    GetDlgItem(IDC_CLOSE_BUTTON)->EnableWindow(FALSE);
    GetDlgItem(IDC_START_GRABBING_BUTTON)->EnableWindow(FALSE);
    GetDlgItem(IDC_STOP_GRABBING_BUTTON)->EnableWindow(FALSE);
    GetDlgItem(IDC_EXPOSURE_EDIT)->EnableWindow(FALSE);
    GetDlgItem(IDC_GAIN_EDIT)->EnableWindow(FALSE);
    GetDlgItem(IDC_GET_PARAMETER_BUTTON)->EnableWindow(FALSE);
    GetDlgItem(IDC_SET_PARAMETER_BUTTON)->EnableWindow(FALSE);

    GetDlgItem(IDC_SAVE_PLY_BTN)->EnableWindow(TRUE);
    GetDlgItem(IDC_SAVE_PLY_BINARY_BTN)->EnableWindow(TRUE);
    GetDlgItem(IDC_SAVE_PCD_BTN)->EnableWindow(TRUE);
    GetDlgItem(IDC_SAVE_PCD_BINARY_BTN)->EnableWindow(TRUE);
}

void    CBasicDemo_PointCloudDlg::SetCtrlWhenOpen()
{
    GetDlgItem(IDC_OPEN_BUTTON)->EnableWindow(FALSE);
    GetDlgItem(IDC_CLOSE_BUTTON)->EnableWindow(TRUE);
    GetDlgItem(IDC_START_GRABBING_BUTTON)->EnableWindow(TRUE);
    GetDlgItem(IDC_EXPOSURE_EDIT)->EnableWindow(TRUE);
    GetDlgItem(IDC_GAIN_EDIT)->EnableWindow(TRUE);
    GetDlgItem(IDC_GET_PARAMETER_BUTTON)->EnableWindow(TRUE);
    GetDlgItem(IDC_SET_PARAMETER_BUTTON)->EnableWindow(TRUE);
    GetDlgItem(IDC_ENUM_BUTTON)->EnableWindow(FALSE);  
}

void    CBasicDemo_PointCloudDlg::SetCtrlWhenClose()
{
    m_ctrlExposureEdit.SetWindowText(NULL);
    m_ctrlGainEdit.SetWindowText(NULL);

    m_bStartJob = false;
    m_bConnect = false;
    GetDlgItem(IDC_OPEN_BUTTON)->EnableWindow(TRUE);
    GetDlgItem(IDC_CLOSE_BUTTON)->EnableWindow(FALSE);
    GetDlgItem(IDC_START_GRABBING_BUTTON)->EnableWindow(FALSE);
    GetDlgItem(IDC_EXPOSURE_EDIT)->EnableWindow(FALSE);
    GetDlgItem(IDC_GAIN_EDIT)->EnableWindow(FALSE);
    GetDlgItem(IDC_GET_PARAMETER_BUTTON)->EnableWindow(FALSE);
    GetDlgItem(IDC_SET_PARAMETER_BUTTON)->EnableWindow(FALSE);

    GetDlgItem(IDC_STOP_GRABBING_BUTTON)->EnableWindow(FALSE);
    GetDlgItem(IDC_ENUM_BUTTON)->EnableWindow(TRUE);
    GetDlgItem(IDC_DEVICE_COMBO2)->EnableWindow(TRUE);
}

void    CBasicDemo_PointCloudDlg::SetCtrlWhenStart()
{
    GetDlgItem(IDC_START_GRABBING_BUTTON)->EnableWindow(FALSE);
    GetDlgItem(IDC_STOP_GRABBING_BUTTON)->EnableWindow(TRUE);

    GetDlgItem(IDC_SAVE_PLY_BTN)->EnableWindow(TRUE);
    GetDlgItem(IDC_SAVE_PLY_BINARY_BTN)->EnableWindow(TRUE);
    GetDlgItem(IDC_SAVE_PCD_BTN)->EnableWindow(TRUE);
    GetDlgItem(IDC_SAVE_PCD_BINARY_BTN)->EnableWindow(TRUE);
    GetDlgItem(IDC_DEVICE_COMBO2)->EnableWindow(FALSE);
}

void    CBasicDemo_PointCloudDlg::SetCtrlWhenStop()
{

    GetDlgItem(IDC_START_GRABBING_BUTTON)->EnableWindow(TRUE);
    GetDlgItem(IDC_STOP_GRABBING_BUTTON)->EnableWindow(FALSE);

    GetDlgItem(IDC_SAVE_PLY_BTN)->EnableWindow(FALSE);
    GetDlgItem(IDC_SAVE_PLY_BINARY_BTN)->EnableWindow(FALSE);
    GetDlgItem(IDC_SAVE_PCD_BTN)->EnableWindow(FALSE);
    GetDlgItem(IDC_SAVE_PCD_BINARY_BTN)->EnableWindow(FALSE);

    GetDlgItem(IDC_DEVICE_COMBO2)->EnableWindow(TRUE);
}

int        CBasicDemo_PointCloudDlg::SaveImage(Mv3dRgbdPointCloudFileType enFileType)
{
    int nRet = MV3D_RGBD_OK;
    CString cstrInfo;

    // ch:�ж��Ƿ�ʼȡ�� | en:Judge whether to start grabbing
    if (!m_bConnect)
    {
        cstrInfo.Format(_T("No camera Connected! "));
        MessageBox(cstrInfo);
        return MV3D_RGBD_E_CALLORDER;
    }

    if (!m_bStartJob)
    {
        cstrInfo.Format(_T("The camera is not startJob!"));
        MessageBox(cstrInfo);
        return MV3D_RGBD_E_CALLORDER;
    }

    //  ch:�ж��Ƿ�����Ч���� | en:Judge whether the image buffer is null
    if (NULL == m_pcDataBuf)
    {
        cstrInfo.Format(_T("No data��Save Image failed!"));
        MessageBox(cstrInfo);
        return MV3D_RGBD_E_CALLORDER;
    }

    if (0 == m_stImageInfo.nDataLen)
    {
        cstrInfo.Format(_T("NO Data, save nothing !"));
        MessageBox(cstrInfo);
        return MV3D_RGBD_E_PARAMETER;
    }

    nRet = MV3D_RGBD_SavePointCloudImage(m_handle, &m_stImageInfo, enFileType, "");
    if (MV3D_RGBD_OK != nRet)
    {
        cstrInfo.Format(_T("SaveImage failed"));
        MessageBox(cstrInfo);
    }

    return nRet;
}

void CBasicDemo_PointCloudDlg::OnBnClickedSavePlyBtn()
{
    // TODO:  �ڴ���ӿؼ�֪ͨ����������
    int nRet = MV3D_RGBD_OK;
    CString cstrInfo;

    nRet = SaveImage(PointCloudFileType_PLY);
    if (MV3D_RGBD_OK != nRet)
    {
        return;
    }

    cstrInfo.Format(_T("Save PLY point image success!"));
    MessageBox(cstrInfo);
}


void CBasicDemo_PointCloudDlg::OnBnClickedSavePlyBinaryBtn()
{
    // TODO:  �ڴ���ӿؼ�֪ͨ����������
    // TODO:  �ڴ���ӿؼ�֪ͨ����������
    int nRet = MV3D_RGBD_OK;
    CString cstrInfo;

    nRet = SaveImage(PointCloudFileType_PLY_Binary);
    if (MV3D_RGBD_OK != nRet)
    {
        return;
    }

    cstrInfo.Format(_T("Save PLY_Binary point image success!"));
    MessageBox(cstrInfo);
}


void CBasicDemo_PointCloudDlg::OnBnClickedSavePcdBtn()
{
    // TODO:  �ڴ���ӿؼ�֪ͨ����������
    int nRet = MV3D_RGBD_OK;
    CString cstrInfo;

    nRet = SaveImage(PointCloudFileType_PCD_ASCII);
    if (MV3D_RGBD_OK != nRet)
    {
        return;
    }

    cstrInfo.Format(_T("Save PCD point image success!"));
    MessageBox(cstrInfo);
}

void CBasicDemo_PointCloudDlg::OnBnClickedSavePcdBinaryBtn()
{
    // TODO:  �ڴ���ӿؼ�֪ͨ����������
    int nRet = MV3D_RGBD_OK;
    CString cstrInfo;

    nRet = SaveImage(PointCloudFileType_PCD_Binary);
    if (MV3D_RGBD_OK != nRet)
    {
        return;
    }

    cstrInfo.Format(_T("Save PCD_Binary point image success!"));
    MessageBox(cstrInfo);
}


void CBasicDemo_PointCloudDlg::OnCbnSelchangeDeviceCombo3()
{
    // TODO:  �ڴ���ӿؼ�֪ͨ����������
    m_nPointCloudType = m_ctrlPointCloudType.GetCurSel();
}
