#include "stdafx.h"
#include "BasicDemo.h"
#include "BasicDemoDlg.h"
#include <string.h> 
#include <comdef.h>
#include <gdiplus.h>

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

void __stdcall  VirtualImageCallBack(MV3D_RGBD_FRAME_DATA* pstFrameData, void* pUser)
{
    CBasicDemoDlg * pThis = (CBasicDemoDlg *)pUser;
    if (pThis)
    {
        pThis->CallBackFunc(pstFrameData,pUser);
    }
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialog)
END_MESSAGE_MAP()

CBasicDemoDlg::CBasicDemoDlg(CWnd* pParent /*=NULL*/)
    : CDialog(CBasicDemoDlg::IDD, pParent)
    , m_bConnect(0)
    , m_bStartJob(0)
    ,m_pcDataBuf(NULL)
    ,m_bBitmapInfo(NULL)
{
    memset(&m_stDeviceInfoList, 0, sizeof(m_stDeviceInfoList));

    m_nDevNum = 0;

    m_handle = NULL;
    m_hWndDisplay = NULL;
    m_MaxImageSize = 0;

    m_pcDataBuf = NULL;

    m_hProcessThread = NULL;    
    m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);

    m_pDisplayMapData = NULL;
    m_nDisplayMapDataLen = 5*1024*1024;
}

void CBasicDemoDlg::DoDataExchange(CDataExchange* pDX)
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

}

BEGIN_MESSAGE_MAP(CBasicDemoDlg, CDialog)
    ON_WM_SYSCOMMAND()
    ON_WM_PAINT()
    ON_WM_QUERYDRAGICON()
    ON_WM_MOUSEWHEEL()
    ON_WM_MOUSEMOVE()
    //ON_WM_NCHITTEST()
    ON_WM_LBUTTONDOWN()
    ON_WM_LBUTTONUP()
    //}}AFX_MSG_MAP
    ON_BN_CLICKED(IDC_ENUM_BUTTON, &CBasicDemoDlg::OnBnClickedEnumButton)
    ON_BN_CLICKED(IDC_OPEN_BUTTON, &CBasicDemoDlg::OnBnClickedOpenButton)
    ON_BN_CLICKED(IDC_START_GRABBING_BUTTON, &CBasicDemoDlg::OnBnClickedStartGrabbingButton)
    ON_BN_CLICKED(IDC_GET_PARAMETER_BUTTON, &CBasicDemoDlg::OnBnClickedGetParameterButton)
    ON_BN_CLICKED(IDC_SET_PARAMETER_BUTTON, &CBasicDemoDlg::OnBnClickedSetParameterButton)
    ON_BN_CLICKED(IDC_STOP_GRABBING_BUTTON, &CBasicDemoDlg::OnBnClickedStopGrabbingButton)
    ON_BN_CLICKED(IDC_CLOSE_BUTTON, &CBasicDemoDlg::OnBnClickedCloseButton)
    ON_WM_CLOSE()
    ON_BN_CLICKED(IDC_SAVE_TIFF_BTN, &CBasicDemoDlg::OnBnClickedSaveTiffBtn)
    ON_BN_CLICKED(IDC_SAVE_BMP_BTN, &CBasicDemoDlg::OnBnClickedSaveBmpBtn)
    ON_BN_CLICKED(IDC_SAVE_JPG_BTN, &CBasicDemoDlg::OnBnClickedSaveJpgBtn)
    ON_BN_CLICKED(IDC_SAVE_RAW_BTN, &CBasicDemoDlg::OnBnClickedSaveRawBtn)

	ON_STN_CLICKED(IDC_DISPLAY_STATIC, &CBasicDemoDlg::OnStnClickedDisplayStatic)
END_MESSAGE_MAP()

BOOL CBasicDemoDlg::OnInitDialog()
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

    SetIcon(m_hIcon, TRUE);            // ch:设置大图标 | en:Set big icon
    SetIcon(m_hIcon, FALSE);        // ch:设置小图标 | en:Set small icon

    // ch:创建窗口句柄 | en: Creat Display Window
    CWnd *pWnd = GetDlgItem(IDC_DISPLAY_STATIC);
    if (NULL == pWnd)
    {
        return MV3D_RGBD_E_RESOURCE;
    }
    m_hWndDisplay = pWnd->GetSafeHwnd();
    if (NULL == m_hWndDisplay)
    {
        return MV3D_RGBD_E_RESOURCE;
    }

    GdiplusStartupInput gdiplusStartupInput;
    ULONG_PTR gdiplusToken;
    // ch:初始化GDI+ | en:GDI+ Initialization
    GdiplusStartup(&gdiplusToken, &gdiplusStartupInput, NULL);
    SetCtrlWhenInit();
    return TRUE;  
}

int CBasicDemoDlg::InitResources()
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
  
        m_MaxImageSize = nSensorWidth * nSensorHight + 4096;

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

void CBasicDemoDlg::DeInitResources()
{
    if (NULL != m_pcDataBuf)
    {
        free(m_pcDataBuf);
        m_pcDataBuf = NULL;
    }
}

void CBasicDemoDlg::OnSysCommand(UINT nID, LPARAM lParam)
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
      
void CBasicDemoDlg::OnPaint()
{
    if (IsIconic())
    {
        CPaintDC dc(this); // ch:用于绘制的设备上下文 | en:device context for painting

        SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

        // ch:使图标在工作区矩形中居中 | en:Center icon in client rectangle
        int cxIcon = GetSystemMetrics(SM_CXICON);
        int cyIcon = GetSystemMetrics(SM_CYICON);
        CRect rect;
        GetClientRect(&rect);
        int x = (rect.Width() - cxIcon + 1) / 2;
        int y = (rect.Height() - cyIcon + 1) / 2;

        // ch:绘制图标 | en:Draw the icon
        dc.DrawIcon(x, y, m_hIcon);
    }
    else
    {
        CDialog::OnPaint();
    }
}

HCURSOR CBasicDemoDlg::OnQueryDragIcon()
{
    return static_cast<HCURSOR>(m_hIcon);
}

BOOL CBasicDemoDlg::PreTranslateMessage(MSG* pMsg)
{
    // ch:屏蔽ESC和ENTER按键 | en:Shielding ESC and ENTER key
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

int  CBasicDemoDlg::Draw(MV3D_RGBD_DRAW_PARAM* pstParam)
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
    // ch:位图信息头 | en:Butmap header
    m_bBitmapInfo->bmiHeader.biSize = sizeof(BITMAPINFOHEADER);             // ch:BITMAPINFOHEADER结构长度 | en:| en:Butmap header size
    m_bBitmapInfo->bmiHeader.biWidth = nImageWidth;                         // ch:图像宽度 | en:Image width
    m_bBitmapInfo->bmiHeader.biPlanes = 1;                                  // ch:位面数 | en: Plane numbers
    m_bBitmapInfo->bmiHeader.biBitCount = 8;                                // ch:比特数/像素的颜色深度,2^8=256 | en:Bit count
    m_bBitmapInfo->bmiHeader.biCompression = BI_RGB;                        // ch:图像数据压缩类型,BI_RGB表示不压缩 | en:Image data compression type, BI_RGB indicates no compression
    m_bBitmapInfo->bmiHeader.biSizeImage = nImageWidth * nImageHeight;      // ch:图像大小 | en: Image data size
    m_bBitmapInfo->bmiHeader.biHeight = - nImageHeight;                     // ch:图像高度 | en: Image height

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


int CBasicDemoDlg::Display(void* handle, void* hWnd, MV3D_RGBD_IMAGE_DATA* pstDisplayImage)
{
    int nRet = MV3D_RGBD_OK;

    if (NULL == pstDisplayImage)
    {
        return MV3D_RGBD_E_PARAMETER;
    }

    if (pstDisplayImage->nWidth == 0 || pstDisplayImage->nHeight == 0 )
    {
        return MV3D_RGBD_E_PARAMETER;
    }

     // ch:显示图像 | en: Display image
    HDC hDC  = ::GetDC((HWND)hWnd);
    SetStretchBltMode(hDC, COLORONCOLOR);
    RECT wndRect;
    ::GetClientRect((HWND)hWnd, &wndRect);

    {
        // ch:缓存下来，后期保存图片 | en: Copy the image buffer for saving image
        m_criSection.Lock();
        if (NULL == m_pcDataBuf || m_MaxImageSize <  pstDisplayImage->nDataLen)
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

        // ch:保存图片信息 & 图片 | en: Copy image infomation and buffer
        memcpy((void *)&m_stImageInfo, pstDisplayImage, sizeof(MV3D_RGBD_IMAGE_DATA));
        if (NULL != pstDisplayImage->pData)
        {
            memcpy(m_pcDataBuf, pstDisplayImage->pData, pstDisplayImage->nDataLen);
        }
        m_stImageInfo.pData = m_pcDataBuf;
        m_criSection.Unlock();
    }

    if (ImageType_Mono8 == pstDisplayImage->enImageType)
    {
        MV3D_RGBD_DRAW_PARAM stParam;   

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
        // ch:不支持 | en: Not support
    }

    ::ReleaseDC((HWND)hWnd, hDC);
    return nRet;
}

void CBasicDemoDlg::OnBnClickedEnumButton()
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

    // ch:查找设备 | en: Search device
    memset(&m_stDeviceInfoList, 0, sizeof(m_stDeviceInfoList));

    nRet = MV3D_RGBD_GetDeviceList(DeviceType_Ethernet | DeviceType_USB | DeviceType_Ethernet_Vir | DeviceType_USB_Vir,&m_stDeviceInfoList[0], 20, &m_nDevNum); //最大20
    if (MV3D_RGBD_OK != nRet)
    {
        cstrInfo.Format(_T("MV3D_RGBD_GetDeviceList failed! err code:%#x"), nRet);
        MessageBox(cstrInfo);
        return;
    }
    
    // ch:将值加入到信息列表框中并显示出来 | en:Add value to the information list box and display
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

void CBasicDemoDlg::OnBnClickedOpenButton()
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

    // ch:获取当前选择的设备信息 | en:Get the currently selected device information
    int nIndex = m_ctrlDeviceCombo.GetCurSel();

    // ch:打开设备 | en:Open device
    nRet = MV3D_RGBD_OpenDevice(&m_handle, &m_stDeviceInfoList[nIndex]);
    if (MV3D_RGBD_OK != nRet)
    {
        cstrInfo.Format(_T("Create handle failed! err code:%#x"), nRet);
        MessageBox(cstrInfo);
        return ;
    }

    nRet = MV3D_RGBD_RegisterFrameCallBack(m_handle, VirtualImageCallBack, this);
    if (MV3D_RGBD_OK != nRet)
    {
        cstrInfo.Format(_T("Register Image CallBack fail! err code:%#x"), nRet);
        MessageBox(cstrInfo);
        return ;
    }

    // ch:获取参数 | en:Get parameter
    OnBnClickedGetParameterButton();
    m_bConnect = true;

    // ch:初始化资源 | en:Init resources
    InitResources();
    SetCtrlWhenOpen();
}

void CBasicDemoDlg::OnBnClickedCloseButton()
{
    int nRet = MV3D_RGBD_OK;
    CString cstrInfo;

    // ch:销毁设备句柄 | en:Destory device handle
    if (NULL != m_handle)
    {
        // ch:停止工作流程 | en:Stop grabbing
        if (true ==  m_bStartJob)
        {
            m_bStartJob = false;
            nRet = MV3D_RGBD_Stop(m_handle);
            if (MV3D_RGBD_OK != nRet)
            {
                cstrInfo.Format(_T("Stop grabbing failed! err code:%#x"), nRet);
                MessageBox(cstrInfo);
                m_bStartJob = false;
                return ;
            }

            // ch:等待渲染线程完全停止 en:Wait for the display thread complete stop 
            if (NULL != m_hProcessThread)
            {
                WaitForSingleObject(m_hProcessThread,INFINITE);
                // ch:最后关闭句柄 | en: Close the display thread handle
                CloseHandle(m_hProcessThread);
                m_hProcessThread = NULL;
            }
        }

        // ch:清空残留图片 | en:Clear diaplay image 
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
    }

    // ch:销毁资源 | en:Deinit resource
    DeInitResources();
    SetCtrlWhenClose();
}

void CBasicDemoDlg::OnBnClickedStartGrabbingButton()
{
    int nRet = MV3D_RGBD_OK;
    CString cstrInfo;

    // ch:等待渲染线程完全停止 en:Wait for the display thread complete stop
    if (NULL != m_hProcessThread)
    {
        m_bStartJob = false;
        WaitForSingleObject(m_hProcessThread,INFINITE);
         // ch:最后关闭句柄 | en: Close the display thread handle
        CloseHandle(m_hProcessThread);
        m_hProcessThread = NULL;
    }
    
    m_bStartJob = TRUE;

    // ch:创建线程 | en:Creat handle
    nRet = MV3D_RGBD_Start(m_handle);
    if (MV3D_RGBD_OK != nRet)
    {
        cstrInfo.Format(_T("Start grabbing failed! err code:%#x"), nRet);
        MessageBox(cstrInfo);
        return ;
    }

    SetCtrlWhenStart();
}

void CBasicDemoDlg::OnBnClickedStopGrabbingButton()
{
    int nRet = MV3D_RGBD_OK;
    CString cstrInfo;

    m_bStartJob = false;
    // ch:停止工作流程 | en:Stop grabbing
    nRet = MV3D_RGBD_Stop(m_handle);
    if (MV3D_RGBD_OK != nRet)
    {
        cstrInfo.Format(_T("Stop grabbing failed! err code:%#x"), nRet);
        MessageBox(cstrInfo);
        m_bStartJob = false;
        return ;
    }
    SetCtrlWhenStop();
}

void CBasicDemoDlg::OnBnClickedGetParameterButton()
{
    int nRet = MV3D_RGBD_OK;
    CString cstrInfo;
    MV3D_RGBD_PARAM pstValue;

    // ch:获取曝光时间 | en: Get exposure time
    float fExposureTime = 0.0f;
    memset(&pstValue, 0, sizeof(MV3D_RGBD_PARAM));
    pstValue.enParamType = ParamType_Float;
    nRet = MV3D_RGBD_GetParam(m_handle, EXPOSURE_TIME, &pstValue);
    if (MV3D_RGBD_OK != nRet)
    {
        cstrInfo.Format(_T("Get exposure time failed! err code:%#x"), nRet);
        MessageBox(cstrInfo);
    }
    else
    {
        fExposureTime = pstValue.ParamInfo.stFloatParam.fCurValue;
        cstrInfo.Format(_T("%0.2f"), fExposureTime);
        m_ctrlExposureEdit.SetWindowText(cstrInfo);
    }

    // ch:获取增益 | en: Get gain
    float fGain= 0.0f;
    memset(&pstValue, 0, sizeof(MV3D_RGBD_PARAM));
    pstValue.enParamType = ParamType_Float;
     nRet = MV3D_RGBD_GetParam(m_handle, GAIN, &pstValue);
    if (MV3D_RGBD_OK != nRet)
    {
        cstrInfo.Format(_T("Get gain failed! err code:%#x"), nRet);
        MessageBox(cstrInfo);
    }
    else
    {
        fGain = pstValue.ParamInfo.stFloatParam.fCurValue;
        cstrInfo.Format(_T("%0.2f"), fGain);
        m_ctrlGainEdit.SetWindowText(cstrInfo);
    }

    UpdateData(FALSE);
}


void CBasicDemoDlg::OnBnClickedSetParameterButton()
{
    UpdateData(TRUE);
    bool bHasError = false;

    int nRet = MV3D_RGBD_OK;
    CString cstrInfo;
    MV3D_RGBD_PARAM pstValue;

    // ch:设置曝光时间 | en:Set exposure time
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

    // ch:设置增益 | en:Set gain
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

void CBasicDemoDlg::OnClose()
{
    // ch:关闭程序，执行断开相机、销毁句柄操作 | en: Close the program, perform close device and deinit resources
    PostQuitMessage(0);
    CloseDevice();

    DeInitResources();
    
    CDialog::OnClose();
}

int        CBasicDemoDlg::CloseDevice(void)
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

void    CBasicDemoDlg::SetCtrlWhenInit()
{
    GetDlgItem(IDC_OPEN_BUTTON)->EnableWindow(FALSE);
    GetDlgItem(IDC_CLOSE_BUTTON)->EnableWindow(FALSE);
    GetDlgItem(IDC_START_GRABBING_BUTTON)->EnableWindow(FALSE);
    GetDlgItem(IDC_STOP_GRABBING_BUTTON)->EnableWindow(FALSE);
    GetDlgItem(IDC_EXPOSURE_EDIT)->EnableWindow(FALSE);
    GetDlgItem(IDC_GAIN_EDIT)->EnableWindow(FALSE);
    GetDlgItem(IDC_GET_PARAMETER_BUTTON)->EnableWindow(FALSE);
    GetDlgItem(IDC_SET_PARAMETER_BUTTON)->EnableWindow(FALSE);

    GetDlgItem(IDC_SAVE_RAW_BTN)->EnableWindow(FALSE);
    GetDlgItem(IDC_SAVE_TIFF_BTN)->EnableWindow(FALSE);
    GetDlgItem(IDC_SAVE_JPG_BTN)->EnableWindow(FALSE);
    GetDlgItem(IDC_SAVE_BMP_BTN)->EnableWindow(FALSE);

}

void    CBasicDemoDlg::SetCtrlWhenOpen()
{
    GetDlgItem(IDC_OPEN_BUTTON)->EnableWindow(FALSE);
    GetDlgItem(IDC_CLOSE_BUTTON)->EnableWindow(TRUE);
    GetDlgItem(IDC_START_GRABBING_BUTTON)->EnableWindow(TRUE);
    GetDlgItem(IDC_EXPOSURE_EDIT)->EnableWindow(TRUE);
    GetDlgItem(IDC_GAIN_EDIT)->EnableWindow(TRUE);
    GetDlgItem(IDC_GET_PARAMETER_BUTTON)->EnableWindow(TRUE);
    GetDlgItem(IDC_SET_PARAMETER_BUTTON)->EnableWindow(TRUE);
    GetDlgItem(IDC_ENUM_BUTTON)->EnableWindow(FALSE);  


    GetDlgItem(IDC_SAVE_RAW_BTN)->EnableWindow(TRUE);
    GetDlgItem(IDC_SAVE_TIFF_BTN)->EnableWindow(TRUE);
    GetDlgItem(IDC_SAVE_JPG_BTN)->EnableWindow(TRUE);
    GetDlgItem(IDC_SAVE_BMP_BTN)->EnableWindow(TRUE);

}

void    CBasicDemoDlg::SetCtrlWhenClose()
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

    GetDlgItem(IDC_SAVE_RAW_BTN)->EnableWindow(FALSE);
    GetDlgItem(IDC_SAVE_TIFF_BTN)->EnableWindow(FALSE);
    GetDlgItem(IDC_SAVE_JPG_BTN)->EnableWindow(FALSE);
    GetDlgItem(IDC_SAVE_BMP_BTN)->EnableWindow(FALSE);

    GetDlgItem(IDC_STOP_GRABBING_BUTTON)->EnableWindow(FALSE);
    GetDlgItem(IDC_ENUM_BUTTON)->EnableWindow(TRUE);   
}

void    CBasicDemoDlg::SetCtrlWhenStart()
{
    GetDlgItem(IDC_START_GRABBING_BUTTON)->EnableWindow(FALSE);
    GetDlgItem(IDC_STOP_GRABBING_BUTTON)->EnableWindow(TRUE);

    GetDlgItem(IDC_SAVE_RAW_BTN)->EnableWindow(TRUE);
    GetDlgItem(IDC_SAVE_TIFF_BTN)->EnableWindow(TRUE);

    GetDlgItem(IDC_SAVE_JPG_BTN)->EnableWindow(TRUE);
    GetDlgItem(IDC_SAVE_BMP_BTN)->EnableWindow(TRUE);
}

void    CBasicDemoDlg::SetCtrlWhenStop()
{

    GetDlgItem(IDC_START_GRABBING_BUTTON)->EnableWindow(TRUE);
    GetDlgItem(IDC_STOP_GRABBING_BUTTON)->EnableWindow(FALSE);

    GetDlgItem(IDC_SAVE_RAW_BTN)->EnableWindow(FALSE);
    GetDlgItem(IDC_SAVE_TIFF_BTN)->EnableWindow(FALSE);

    GetDlgItem(IDC_SAVE_JPG_BTN)->EnableWindow(FALSE);
    GetDlgItem(IDC_SAVE_BMP_BTN)->EnableWindow(FALSE);
}


int        CBasicDemoDlg::SaveImage(Mv3dRgbdFileType enFileType)
{
    int nRet = MV3D_RGBD_OK;
    CString cstrInfo;

    // ch:判断是否开始取流 | en:Judge whether to start grabbing
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

    //  ch:判断是否有有效数据 | en:Judge whether the image buffer is null
    if (NULL == m_pcDataBuf)
    {
        cstrInfo.Format(_T("No data，Save Image failed!"));
        MessageBox(cstrInfo);
        return MV3D_RGBD_E_CALLORDER;
    }

    if (0 == m_stImageInfo.nDataLen)
    {
        cstrInfo.Format(_T("NO Data, save nothing !"));
        MessageBox(cstrInfo);
        return MV3D_RGBD_E_PARAMETER;
    }

    nRet = MV3D_RGBD_SaveImage(m_handle, &m_stImageInfo, enFileType, "");
    if (MV3D_RGBD_OK != nRet)
    {
        cstrInfo.Format(_T("SaveImage failed"));
        MessageBox(cstrInfo);
    }

    return nRet;
}

void    CBasicDemoDlg::OnBnClickedSaveTiffBtn()
{
    int nRet = MV3D_RGBD_OK;
    CString cstrInfo;

    nRet = SaveImage(FileType_TIFF);
    if (MV3D_RGBD_OK != nRet)
    {
        return;
    }

    cstrInfo.Format(_T("Save tiff image success!"));
    MessageBox(cstrInfo);
}

void    CBasicDemoDlg::OnBnClickedSaveBmpBtn()
{
    int nRet = MV3D_RGBD_OK;
    CString cstrInfo;

    nRet = SaveImage(FileType_BMP);
    if (MV3D_RGBD_OK != nRet)
    {
        return;
    }

    cstrInfo.Format(_T("Save bmp image success!"));
    MessageBox(cstrInfo);
}

void    CBasicDemoDlg::OnBnClickedSaveJpgBtn()
{
    int nRet = MV3D_RGBD_OK;
    CString cstrInfo;

    nRet = SaveImage(FileType_JPG);
    if (MV3D_RGBD_OK != nRet)
    {
        return;
    }

    cstrInfo.Format(_T("Save jpg image success!"));
    MessageBox(cstrInfo);
}

void    CBasicDemoDlg::OnBnClickedSaveRawBtn()
{
    int nRet = MV3D_RGBD_OK;
    CString cstrInfo;

    // ch:判断是否开始取流 | en:Judge whether to start grabbing
    if (!m_bConnect)
    {
        cstrInfo.Format(_T("No camera Connected! "));
        MessageBox(cstrInfo);
        return;
    }

    if (!m_bStartJob)
    {
        cstrInfo.Format(_T("The camera is not startJob!"));
        MessageBox(cstrInfo);
        return;
    }

    //  ch:判断是否有有效数据 | en:Judge whether the image buffer is null
    if (NULL == m_pcDataBuf)
    {
        cstrInfo.Format(_T("No valid image data，Save RAW failed!"));
        MessageBox(cstrInfo);
        return;
    }

    if (0 == m_stImageInfo.nDataLen)
    {
        cstrInfo.Format(_T("NO Data, save nothing !"));
        MessageBox(cstrInfo);
        return;             
    }

    // ch:保存RAW图像 | en:Save as raw image
    FILE* pfile;
    char filename[256] = {0};
    CTime currTime;                                     // ch:获取系统时间作为保存图片文件名 | en:Get the system current time as image name
    currTime = CTime::GetCurrentTime(); 
    sprintf(filename,("%.4d%.2d%.2d%.2d%.2d%.2d.raw"), currTime.GetYear(), currTime.GetMonth(),
        currTime.GetDay(), currTime.GetHour(), currTime.GetMinute(), currTime.GetSecond());
    pfile = fopen(filename,"wb");
    if(pfile == NULL)
    {
        cstrInfo.Format(_T("Open file failed!"));
        MessageBox(cstrInfo);
        return ;
    }

    {
        m_criSection.Lock();
        fwrite(m_pcDataBuf, 1, m_stImageInfo.nDataLen, pfile);
        m_criSection.Unlock();
    }

    cstrInfo.Format(_T("Save raw image success!"));
    MessageBox(cstrInfo);
    fclose (pfile);
    pfile = NULL;
}
 
void CBasicDemoDlg::CallBackFunc(MV3D_RGBD_FRAME_DATA* pstFrameData, void* pUser)
{
    int nRet = MV3D_RGBD_OK;
    CBasicDemoDlg* pThis = (CBasicDemoDlg*)pUser;
    if (NULL != pThis)
    {
        try
        {              
            nRet = pThis->Display(pThis->m_handle,pThis->m_hWndDisplay, &pstFrameData->stImageData[0]);
            //nRet = pThis->Display(pThis->m_handle,pThis->m_hWndDisplay, &pstFrameData->stImageData[1]); // ch:RGB图 | en:RGB image
            if (MV3D_RGBD_OK != nRet)
            {
                throw nRet;
            }
        }
        catch (...)
        {
            printf("ERROR  !\r\n");
        }
        return ;
    }
    else
    {
        return;
    }



    printf("stop recv  !\r\n");
    return ;
}



void CBasicDemoDlg::OnStnClickedDisplayStatic()
{
	// TODO:  在此添加控件通知处理程序代码
}
