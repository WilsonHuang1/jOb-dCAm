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
    m_nConvertColorMapMode = 0;

    m_handle = NULL;
    m_hWndDisplay = NULL;
    m_hWndDisplay2 = NULL;

    m_MaxImageSize = 0;
    m_pcDataBuf = NULL;

    m_hProcessThread = NULL;    

    m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
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
    DDX_Control(pDX, IDC_WIDTH, m_ctrlWidthEdit);
    DDX_Control(pDX, IDC_HEIGHT, m_ctrlHeightEdit);
    DDX_Control(pDX, IDC_DEPTH, m_ctrlDepthEdit);
    DDX_Control(pDX, IDC_GET_PARAMETER_BUTTON, m_ctrlGetParameterButton);
    DDX_Control(pDX, IDC_SET_PARAMETER_BUTTON, m_ctrlSetParameterButton);
    DDX_Control(pDX, IDC_DEVICE_COMBO, m_ctrlDeviceCombo);
    DDX_Control(pDX, IDC_DEVICE_COMBO2, m_ctrlImageAlignCombo);
    DDX_Control(pDX, IDC_DEVICE_COMBO3, m_ctrlConvertColorMapCombo);
}

BEGIN_MESSAGE_MAP(CBasicDemoDlg, CDialog)
    ON_WM_SYSCOMMAND()
    ON_WM_PAINT()
    ON_WM_QUERYDRAGICON()
    ON_WM_MOUSEWHEEL()
    ON_WM_MOUSEMOVE()
    ON_WM_LBUTTONDOWN()
    ON_WM_LBUTTONUP()
    ON_BN_CLICKED(IDC_ENUM_BUTTON, &CBasicDemoDlg::OnBnClickedEnumButton)
    ON_BN_CLICKED(IDC_OPEN_BUTTON, &CBasicDemoDlg::OnBnClickedOpenButton)
    ON_BN_CLICKED(IDC_START_GRABBING_BUTTON, &CBasicDemoDlg::OnBnClickedStartGrabbingButton)
    ON_BN_CLICKED(IDC_BTN_Start, &CBasicDemoDlg::OnBnClickedStartGrabbingButton)
    ON_BN_CLICKED(IDC_GET_PARAMETER_BUTTON, &CBasicDemoDlg::OnBnClickedGetParameterButton)
    ON_BN_CLICKED(IDC_SET_PARAMETER_BUTTON, &CBasicDemoDlg::OnBnClickedSetParameterButton)
    ON_BN_CLICKED(IDC_STOP_GRABBING_BUTTON, &CBasicDemoDlg::OnBnClickedStopGrabbingButton)
    ON_BN_CLICKED(IDC_CLOSE_BUTTON, &CBasicDemoDlg::OnBnClickedCloseButton)
    ON_WM_CLOSE()
    ON_BN_CLICKED(IDC_SAVE_TIFF_BTN, &CBasicDemoDlg::OnBnClickedSaveTiffBtn)
    ON_BN_CLICKED(IDC_SAVE_BMP_BTN, &CBasicDemoDlg::OnBnClickedSaveBmpBtn)
    ON_BN_CLICKED(IDC_SAVE_JPG_BTN, &CBasicDemoDlg::OnBnClickedSaveJpgBtn)
    ON_BN_CLICKED(IDC_SAVE_RAW_BTN, &CBasicDemoDlg::OnBnClickedSaveRawBtn)
    ON_CBN_SELCHANGE(IDC_DEVICE_COMBO3, &CBasicDemoDlg::OnCbnSelchangeDeviceCombo3)
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

    SetIcon(m_hIcon, TRUE);         //ch:���ô�ͼ�� | en:Set big icon
    SetIcon(m_hIcon, FALSE);        //ch:����Сͼ�� | en:Set small icon

    // ch:�������ھ�� | en: Creat Display Window
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

    // ch:�������ھ�� | en: Creat Display Window
    CWnd *pWnd2 = GetDlgItem(IDC_DISPLAY_STATIC2);
    if (NULL == pWnd)
    {
        return MV3D_RGBD_E_RESOURCE;
    }
    m_hWndDisplay2 = pWnd2->GetSafeHwnd();
    if (NULL == m_hWndDisplay2)
    {
        return MV3D_RGBD_E_RESOURCE;
    }

    GdiplusStartupInput gdiplusStartupInput;
    ULONG_PTR gdiplusToken;
    // ch:��ʼ��GDI+ | en:GDI+ Initialization
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

HCURSOR CBasicDemoDlg::OnQueryDragIcon()
{
    return static_cast<HCURSOR>(m_hIcon);
}

BOOL CBasicDemoDlg::PreTranslateMessage(MSG* pMsg)
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
void*  __stdcall CBasicDemoDlg::ProcessThread(void* pUser)
{
    int nRet = MV3D_RGBD_OK;

    CBasicDemoDlg* pThis = (CBasicDemoDlg*)pUser;
    if (NULL == pThis)
    {
        return NULL;
    }

    MV3D_RGBD_FRAME_DATA stFrameData = {0};
    while (pThis->m_bStartJob)
    {
        // ch:��ȡͼ������ | en:Get image data
        nRet = MV3D_RGBD_FetchFrame(pThis->m_handle, &stFrameData, 5000);
        if (MV3D_RGBD_OK ==  nRet && !stFrameData.nValidInfo)
        {
            try
            {
                for(int i=0; i < stFrameData.nImageCount;i++)
                {
                    if(stFrameData.stImageData[i].enImageType == ImageType_Depth)
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

                                pThis->m_MaxImageSize =  stFrameData.stImageData[i].nDataLen;
                                pThis->m_pcDataBuf =  (unsigned char*)malloc(pThis->m_MaxImageSize);
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
                                memcpy(pThis->m_pcDataBuf,stFrameData.stImageData[i].pData, stFrameData.stImageData[i].nDataLen);
                            }
                            pThis->m_stImageInfo.pData = pThis->m_pcDataBuf;
                            pThis->m_criSection.Unlock();
                        }

                        
                        if (pThis->m_nConvertColorMapMode == 0)
                        {
                            nRet = pThis->Display(pThis->m_handle,pThis->m_hWndDisplay, &stFrameData.stImageData[i]);
                            if (MV3D_RGBD_OK != nRet)
                            {
                                throw nRet;
                            }
                        }
                        else
                        {
                            // ch:����α��ͼ���� | en:Config parameters of depth convert to color 
                            MV3D_RGBD_CONVERT_COLOR_PAPRAM stConvertParam;
                            memset(&stConvertParam, 0, sizeof(MV3D_RGBD_CONVERT_COLOR_PAPRAM));
                            stConvertParam.enConvertColorMapMode = (Mv3dRgbdConvertColorMapMode)pThis->m_nConvertColorMapMode;
                            MV3D_RGBD_IMAGE_DATA stConvertColorImage;
                            nRet = MV3D_RGBD_MapDepthToColor(&stFrameData.stImageData[i], &stConvertParam, &stConvertColorImage);
                            if (MV3D_RGBD_OK == nRet)
                            {
                                nRet = pThis->Display(pThis->m_handle,pThis->m_hWndDisplay, &stConvertColorImage);
                                if (MV3D_RGBD_OK != nRet)
                                {
                                    throw nRet;
                                }
                            }
                        }    
                    }
                    else if(stFrameData.stImageData[i].enImageType == ImageType_YUV422 || stFrameData.stImageData[i].enImageType == ImageType_YUV420SP_NV12 ||
                        stFrameData.stImageData[i].enImageType == ImageType_YUV420SP_NV21 || stFrameData.stImageData[i].enImageType ==ImageType_RGB8_Planar || 
                        stFrameData.stImageData[i].enImageType == ImageType_Jpeg)
                    {
                        nRet = pThis->Display(pThis->m_handle,pThis->m_hWndDisplay2, &stFrameData.stImageData[i]);
                        if (MV3D_RGBD_OK != nRet)
                        {
                            throw nRet;
                        }
                    }
                    else
                    {
                        continue;
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
            Sleep(1);
            continue;
        }
    }
    printf("stop recv  !\r\n");
    return NULL;
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


int CBasicDemoDlg::Display(void* handle, void* hWnd, MV3D_RGBD_IMAGE_DATA* pstDisplayImage)
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

    // ch:��ʾͼ�� | en: Display image
    HDC hDC  = ::GetDC((HWND)hWnd);
    SetStretchBltMode(hDC, COLORONCOLOR);
    RECT wndRect;
    ::GetClientRect((HWND)hWnd, &wndRect);

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
        // ch:��֧�� | en: Not support
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

    // ch:�����豸 | en: Search device
    memset(&m_stDeviceInfoList, 0, sizeof(m_stDeviceInfoList));

    nRet = MV3D_RGBD_GetDeviceList(DeviceType_Ethernet | DeviceType_USB | DeviceType_Ethernet_Vir | DeviceType_USB_Vir,&m_stDeviceInfoList[0], 50, &m_nDevNum);  //ch:���20 | en:Max number 20
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

    m_ctrlImageAlignCombo.ResetContent();
    m_ctrlConvertColorMapCombo.ResetContent();

    m_ctrlImageAlignCombo.AddString(CStringW("Depth"));
    m_ctrlImageAlignCombo.AddString(CStringW("RGB"));

    m_ctrlConvertColorMapCombo.AddString(CStringW("Custom"));
    m_ctrlConvertColorMapCombo.AddString(CStringW("Rainbow"));
    m_ctrlConvertColorMapCombo.AddString(CStringW("DarkRainbow"));
    m_ctrlConvertColorMapCombo.AddString(CStringW("DarkGreen"));
    m_ctrlConvertColorMapCombo.AddString(CStringW("PinkishRed"));          
    m_ctrlConvertColorMapCombo.AddString(CStringW("Yellow"));                 
    m_ctrlConvertColorMapCombo.AddString(CStringW("GrayScale"));            

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
    m_ctrlConvertColorMapCombo.SetCurSel(0);
    m_nConvertColorMapMode = 0;
    // ch:��ȡ���� | en:Get parameter
    OnBnClickedGetParameterButton();
    m_bConnect = true;

    // ch:��ʼ����Դ | en:Init resources
    InitResources();
    SetCtrlWhenOpen();
}

void CBasicDemoDlg::OnBnClickedCloseButton()
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
            // ch:�ȴ���Ⱦ�߳���ȫֹͣ en:Wait for the display thread complete stop
            if (NULL != m_hProcessThread)
            {
                WaitForSingleObject(m_hProcessThread, INFINITE);
                // ch:���رվ�� | en: Close the display thread handle
                CloseHandle(m_hProcessThread);
                m_hProcessThread = NULL;
            }

            nRet = MV3D_RGBD_Stop(m_handle);
            if (MV3D_RGBD_OK != nRet)
            {
                cstrInfo.Format(_T("Stop grabbing failed! err code:%#x"), nRet);
                MessageBox(cstrInfo);
                m_bStartJob = false;
                return ;
            }
        }

        // ch:��ղ���ͼƬ | en:Clear diaplay image 
        GetDlgItem(IDC_DISPLAY_STATIC)->ShowWindow(FALSE);
        GetDlgItem(IDC_DISPLAY_STATIC)->ShowWindow(TRUE);

        GetDlgItem(IDC_DISPLAY_STATIC2)->ShowWindow(FALSE);
        GetDlgItem(IDC_DISPLAY_STATIC2)->ShowWindow(TRUE);

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
        m_ctrlConvertColorMapCombo.ResetContent();
    }

    // ch:������Դ | en:Deinit resource
    DeInitResources();
    SetCtrlWhenClose();
}

void CBasicDemoDlg::OnBnClickedStartGrabbingButton()
{
    int nRet = MV3D_RGBD_OK;
    CString cstrInfo;

    // ch:�ȴ���Ⱦ�߳���ȫֹͣ en:Wait for the display thread complete stop
    if (NULL != m_hProcessThread)
    {
        m_bStartJob = false;
        WaitForSingleObject(m_hProcessThread,INFINITE);
        // ch:���رվ�� | en: Close the display thread handle
        CloseHandle(m_hProcessThread);
        m_hProcessThread = NULL;
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

    // ch:�����߳� | en:Creat handle
    m_hProcessThread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)ProcessThread, this, 0, NULL);
    if (NULL == m_hProcessThread)
    {
        cstrInfo.Format(_T("Create proccess Thread failed "), 0);
        MessageBox(cstrInfo);
    }

    SetCtrlWhenStart();
}

void CBasicDemoDlg::OnBnClickedStopGrabbingButton()
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

    SetCtrlWhenStop();
}

void CBasicDemoDlg::OnBnClickedGetParameterButton()
{
    int nRet = MV3D_RGBD_OK;
    CString cstrInfo;
    MV3D_RGBD_PARAM pstValue;

    // ch:��ȡ�ع�ʱ�� | en: Get exposure time
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

    // ch:��ȡ���� | en: Get gain
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

void CBasicDemoDlg::OnClose()
{
    // ch:�رճ���ִ�жϿ���������پ������ | en: Close the program, perform close device and deinit resources
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
    m_ctrlWidthEdit.SetWindowText(NULL);
    m_ctrlHeightEdit.SetWindowText(NULL);
    m_ctrlDepthEdit.SetWindowText(NULL);

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
    GetDlgItem(IDC_DEVICE_COMBO2)->EnableWindow(TRUE);
}

void    CBasicDemoDlg::SetCtrlWhenStart()
{
    GetDlgItem(IDC_START_GRABBING_BUTTON)->EnableWindow(FALSE);
    GetDlgItem(IDC_STOP_GRABBING_BUTTON)->EnableWindow(TRUE);

    GetDlgItem(IDC_SAVE_RAW_BTN)->EnableWindow(TRUE);
    GetDlgItem(IDC_SAVE_TIFF_BTN)->EnableWindow(TRUE);
    GetDlgItem(IDC_SAVE_JPG_BTN)->EnableWindow(TRUE);
    GetDlgItem(IDC_SAVE_BMP_BTN)->EnableWindow(TRUE);
    GetDlgItem(IDC_DEVICE_COMBO2)->EnableWindow(FALSE);
}

void    CBasicDemoDlg::SetCtrlWhenStop()
{

    GetDlgItem(IDC_START_GRABBING_BUTTON)->EnableWindow(TRUE);
    GetDlgItem(IDC_STOP_GRABBING_BUTTON)->EnableWindow(FALSE);

    GetDlgItem(IDC_SAVE_RAW_BTN)->EnableWindow(FALSE);
    GetDlgItem(IDC_SAVE_TIFF_BTN)->EnableWindow(FALSE);
    GetDlgItem(IDC_SAVE_JPG_BTN)->EnableWindow(FALSE);
    GetDlgItem(IDC_SAVE_BMP_BTN)->EnableWindow(FALSE);

    GetDlgItem(IDC_DEVICE_COMBO2)->EnableWindow(TRUE);
}

int        CBasicDemoDlg::SaveImage(Mv3dRgbdFileType enFileType)
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

    // ch:�ж��Ƿ�ʼȡ�� | en:Judge whether to start grabbing
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

    //  ch:�ж��Ƿ�����Ч���� | en:Judge whether the image buffer is null
    if (NULL == m_pcDataBuf)
    {
        cstrInfo.Format(_T("No valid image data��Save RAW failed!"));
        MessageBox(cstrInfo);
        return;
    }

    if (0 == m_stImageInfo.nDataLen)
    {
        cstrInfo.Format(_T("NO Data, save nothing !"));
        MessageBox(cstrInfo);
        return;             
    }

    // ch:����RAWͼ�� | en:Save as raw image
    FILE* pfile;
    char filename[256] = {0};
    CTime currTime;                                    // ch:��ȡϵͳʱ����Ϊ����ͼƬ�ļ��� | en:Get the system current time as image name
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

void CBasicDemoDlg::OnLButtonDown(UINT nFlags, CPoint point)
{
    // TODO: �ڴ������Ϣ�����������/�����Ĭ��ֵ
    CRect rect;
    CWnd *pWnd = GetDlgItem(IDC_DISPLAY_STATIC);  //ͼ��ؼ����ID
    pWnd->GetWindowRect(&rect);
    ScreenToClient(&rect);
 
    //�Ȼ�ȡ����������Ļ�ľ�������
    GetCursorPos(&point);
    int temp_x = point.x;   //Ϊ�˻�ȡ�Ҷ�ֵʹ��
    int temp_y = point.y;
    //Ȼ��õ�static�ؼ���rect����
    CRect pRect;
    pWnd->GetClientRect(&pRect);
    //���ѵ�ǰ��������ת��Ϊ�����rect�����꣨������꣩
    pWnd->ScreenToClient(&point);
    int x = point.x;     //�������
    int y = point.y;

    if(x > rect.Width()|| y > rect.Height())
    {
        return;
    }

    {
        m_criSection.Lock();
        int nImageWidth = m_stImageInfo.nWidth;
        int nImageHeight = m_stImageInfo.nHeight;

        int nRectWidth = rect.Width();
        int nRectHeight = rect.Height();

        int nImageX = x * nImageWidth / rect.Width();
        int nImageY =  y * nImageHeight / rect.Height();

        if(m_pcDataBuf == NULL)
        {
            m_criSection.Unlock();
            return;
        }

        unsigned short sValue = (unsigned short)(m_pcDataBuf[2 * (nImageWidth * nImageY + nImageX) - 2] | (m_pcDataBuf[2 * (nImageWidth * nImageY + nImageX) - 1] << 8));

        CString cstrInfo;
        cstrInfo.Format(_T("%d"), nImageX);
        m_ctrlWidthEdit.SetWindowText(cstrInfo);

        cstrInfo.Format(_T("%d"), nImageY);
        m_ctrlHeightEdit.SetWindowText(cstrInfo);

        cstrInfo.Format(_T("%d mm"), sValue);
        m_ctrlDepthEdit.SetWindowText(cstrInfo);
        m_criSection.Unlock();
    }
    
}


void CBasicDemoDlg::OnCbnSelchangeDeviceCombo3()
{
    // TODO:  �ڴ���ӿؼ�֪ͨ����������
    m_nConvertColorMapMode = m_ctrlConvertColorMapCombo.GetCurSel();
}
