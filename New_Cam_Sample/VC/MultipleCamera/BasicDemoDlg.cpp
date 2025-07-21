#include "stdafx.h"
#include "BasicDemoDlg.h"

using namespace Gdiplus;
#pragma comment( lib, "gdiplus.lib" ) 


#ifdef _DEBUG
#define new DEBUG_NEW
#endif

// 用于应用程序“关于”菜单项的 CAboutDlg 对话框

class CAboutDlg : public CDialog
{
public:
    CAboutDlg();

// 对话框数据
    enum { IDD = IDD_ABOUTBOX };

    protected:
    virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

// 实现
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


//对话框
CBasicDemoDlg::CBasicDemoDlg(CWnd* pParent /*=NULL*/)
    : CDialog(CBasicDemoDlg::IDD, pParent),
    m_nDeviceNumber(0),
    m_nUseCbxNum(0)
{
    m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);

    for (int i = 0; i < MAX_DEVICE_NUM; i++)
    {
        m_hwndDisplay[i] = NULL;
        m_cwmdDisplay[i] = NULL;
        m_pcMyCamera[i] = NULL;
        m_nCurDevIndex[i] = 0;
    }
}

void CBasicDemoDlg::DoDataExchange(CDataExchange* pDX)
{
    CDialog::DoDataExchange(pDX);
    DDX_Control(pDX, IDC_VERSION_INFO_EDIT, m_ctrlVersionEdit);
    DDX_Control(pDX, IDC_DEV_COUNT_EDIT   , m_ctrlDevCountEdit);
    DDX_Control(pDX, IDC_DEVICE_COMBOX   , m_ctrlDeviceCombo);
    DDX_Control(pDX, IDC_CAM_COMBOX_1   , m_ctrlCamCb1);
    DDX_Control(pDX, IDC_CAM_COMBOX_2   , m_ctrlCamCb2);
    DDX_Control(pDX, IDC_CAM_COMBOX_3   , m_ctrlCamCb3);
    DDX_Control(pDX, IDC_CAM_COMBOX_4   , m_ctrlCamCb4);
}

BEGIN_MESSAGE_MAP(CBasicDemoDlg, CDialog)
    ON_WM_SYSCOMMAND()
    ON_WM_PAINT()
    ON_WM_QUERYDRAGICON()
    //}}AFX_MSG_MAP
    ON_BN_CLICKED(IDC_ENUM_BUTTON, &CBasicDemoDlg::OnBnClickedEnumButton)
    ON_BN_CLICKED(IDC_OPEN_DEV_BTN, &CBasicDemoDlg::OnBnClickedOpenDevBtn)
    ON_BN_CLICKED(IDC_CLOSE_DEV_BTN, &CBasicDemoDlg::OnBnClickedCloseDevBtn)
    ON_BN_CLICKED(IDC_OPEN_MEASURE_BTN, &CBasicDemoDlg::OnBnClickedOpenGrabBtn)
    ON_BN_CLICKED(IDC_CLOSE_MEASURE_BTN, &CBasicDemoDlg::OnBnClickedCloseGrabBtn)
    ON_BN_CLICKED(IDC_SOFT_TRIGGER_BTN, &CBasicDemoDlg::OnBnClickedSoftTriggerBtn)
    ON_BN_CLICKED(IDC_SAVE_RAW_BUTTON, &CBasicDemoDlg::OnBnClickedSaveRawButton)
END_MESSAGE_MAP()


//消息处理程序
BOOL CBasicDemoDlg::OnInitDialog()
{
    CDialog::OnInitDialog();

    // 将“关于...”菜单项添加到系统菜单中。

    // IDM_ABOUTBOX 必须在系统命令范围内。
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

    // 设置此对话框的图标。当应用程序主窗口不是对话框时，框架将自动
    //  执行此操作
    SetIcon(m_hIcon, TRUE);            // 设置大图标
    SetIcon(m_hIcon, FALSE);        // 设置小图标

    // TODO: 在此添加额外的初始化代码
    InitResources();

    GdiplusStartupInput gdiplusStartupInput;
    ULONG_PTR gdiplusToken;
    //初始化GDI+
    GdiplusStartup(&gdiplusToken, &gdiplusStartupInput, NULL);

    return TRUE;  // 除非将焦点设置到控件，否则返回 TRUE
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

// 如果向对话框添加最小化按钮，则需要下面的代码
//  来绘制该图标。对于使用文档/视图模型的 MFC 应用程序，
//  这将由框架自动完成。

void CBasicDemoDlg::OnPaint()
{
    if (IsIconic())
    {
        CPaintDC dc(this); // 用于绘制的设备上下文

        SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

        // 使图标在工作区矩形中居中
        int cxIcon = GetSystemMetrics(SM_CXICON);
        int cyIcon = GetSystemMetrics(SM_CYICON);
        CRect rect;
        GetClientRect(&rect);
        int x = (rect.Width() - cxIcon + 1) / 2;
        int y = (rect.Height() - cyIcon + 1) / 2;

        // 绘制图标
        dc.DrawIcon(x, y, m_hIcon);
    }
    else
    {
        CDialog::OnPaint();
    }
}

//当用户拖动最小化窗口时系统调用此函数取得光标
//显示。
HCURSOR CBasicDemoDlg::OnQueryDragIcon()
{
    return static_cast<HCURSOR>(m_hIcon);
}

void CBasicDemoDlg::OnBnClickedEnumButton()
{
    m_ctrlDeviceCombo.ResetContent();
    m_ctrlCamCb1.ResetContent();
    m_ctrlCamCb2.ResetContent();
    m_ctrlCamCb3.ResetContent();
    m_ctrlCamCb4.ResetContent();

    int nRet = MV3D_RGBD_OK;
    CString cstrInfo;
    CString cstrCamSNInfo;

    nRet = MV3D_RGBD_GetDeviceNumber(DeviceType_Ethernet | DeviceType_USB | DeviceType_Ethernet_Vir | DeviceType_USB_Vir, &m_nDeviceNumber);
    if (MV3D_RGBD_OK != nRet)
    {
        cstrInfo.Format(_T("MV3D_RGBD_GetDeviceNumber failed! err code:%#x"), nRet);
        MessageBox(cstrInfo);
        return;
    }
    else
    {
        cstrInfo.Format(_T("%d"),m_nDeviceNumber);
        m_ctrlDevCountEdit.SetWindowText(cstrInfo);
    }

    // 查找设备
    memset(&m_stDeviceInfoList, 0, sizeof(m_stDeviceInfoList));

    nRet = MV3D_RGBD_GetDeviceList(DeviceType_Ethernet | DeviceType_USB | DeviceType_Ethernet_Vir | DeviceType_USB_Vir,&m_stDeviceInfoList[0], 20, &m_nDeviceNumber);  //最大20
    if (MV3D_RGBD_OK != nRet)
    {
        cstrInfo.Format(_T("MV3D_RGBD_GetDeviceList failed! err code:%#x"), nRet);
        MessageBox(cstrInfo);
        return;
    }

    m_ctrlCamCb1.InsertString(0,_T("None"));
    m_ctrlCamCb2.InsertString(0,_T("None"));
    m_ctrlCamCb3.InsertString(0,_T("None"));
    m_ctrlCamCb4.InsertString(0,_T("None"));

    // 显示查找到的设备信息
    for (unsigned int i = 0; i < m_nDeviceNumber; i++)
    { 
        char  pCurrentIP[16] = ""; 
        memcpy(pCurrentIP,m_stDeviceInfoList[i].SpecialInfo.stNetInfo.chCurrentIp,strlen((const char*)m_stDeviceInfoList[i].SpecialInfo.stNetInfo.chCurrentIp));

        char  pCurrentSN[16] = ""; 
        memcpy(pCurrentSN,m_stDeviceInfoList[i].chSerialNumber,strlen((const char*)m_stDeviceInfoList[i].chSerialNumber));

        if (DeviceType_Ethernet == m_stDeviceInfoList[i].enDeviceType || DeviceType_Ethernet_Vir == m_stDeviceInfoList[i].enDeviceType)
        {
            char  pCurrentIP[16] = ""; 
            memcpy(pCurrentIP,m_stDeviceInfoList[i].SpecialInfo.stNetInfo.chCurrentIp,strlen((const char*)m_stDeviceInfoList[i].SpecialInfo.stNetInfo.chCurrentIp));

            cstrInfo.Format(_T("Name[%s] IP[%s] SerialNum[%s]"),  CStringW(m_stDeviceInfoList[i].chModelName), CStringW(pCurrentIP), CStringW(m_stDeviceInfoList[i].chSerialNumber));
        }
        else if (DeviceType_USB == m_stDeviceInfoList[i].enDeviceType || DeviceType_USB_Vir == m_stDeviceInfoList[i].enDeviceType)
        {
            cstrInfo.Format(_T("Name[%s] UsbProtocol[%d] SerialNum[%s]"), CStringW(m_stDeviceInfoList[i].chModelName), m_stDeviceInfoList[i].SpecialInfo.stUsbInfo.enUsbProtocol, CStringW(m_stDeviceInfoList[i].chSerialNumber));
        }

        m_ctrlDeviceCombo.AddString(cstrInfo);

        cstrCamSNInfo.Format(_T("[%d]:[%s]"),i,CStringW(pCurrentSN));
        m_ctrlCamCb1.AddString(cstrCamSNInfo);
        m_ctrlCamCb2.AddString(cstrCamSNInfo);
        m_ctrlCamCb3.AddString(cstrCamSNInfo);
        m_ctrlCamCb4.AddString(cstrCamSNInfo);
    }

    UpdateData(FALSE);
    if(0 < m_nDeviceNumber)
    {
        m_ctrlDeviceCombo.SetCurSel(0);
        m_ctrlCamCb1.SetCurSel(0);
        m_ctrlCamCb2.SetCurSel(0);
        m_ctrlCamCb3.SetCurSel(0);
        m_ctrlCamCb4.SetCurSel(0);
        GetDlgItem(IDC_CAM_COMBOX_1)->EnableWindow(TRUE);
        GetDlgItem(IDC_CAM_COMBOX_2)->EnableWindow(TRUE);
        GetDlgItem(IDC_CAM_COMBOX_3)->EnableWindow(TRUE);
        GetDlgItem(IDC_CAM_COMBOX_4)->EnableWindow(TRUE);
        GetDlgItem(IDC_OPEN_DEV_BTN)->EnableWindow(TRUE);
    }

}

void CBasicDemoDlg::OnBnClickedOpenDevBtn()
{
    int nRet = MV3D_RGBD_OK; 
    CString cstrInfo;

    UpdateData(TRUE);

    bool bIsRepeat = IsRepeatIndex();
    if (true == bIsRepeat)
    {
        cstrInfo.Format(_T("Please select right camera"));
        MessageBox(cstrInfo);
        return ;
    }

    int nCanOpenDeviceNum = 0;
    int nCurDevIndex = 0;
    for (unsigned int i = 0; i < MAX_DEVICE_NUM; i++)
    {
        nCurDevIndex = m_nCurDevIndex[i];
        if (0 == nCurDevIndex)
        {
            continue;
        }

        m_pcMyCamera[i] = new  CMyCamera;
        if (NULL == m_pcMyCamera[i])
        {
            cstrInfo.Format(_T("Please create camera failed!"));
            MessageBox(cstrInfo);
            return ;
        }

        nRet |= m_pcMyCamera[i]->Open(m_stDeviceInfoList[nCurDevIndex - 1].chSerialNumber);
        if (MV3D_RGBD_OK != nRet)
        {
            delete m_pcMyCamera[i];
            m_pcMyCamera[i] = NULL;
            cstrInfo.Format(_T("Camera %d Open failed! error code:%#x"),i + 1,nRet);
            MessageBox(cstrInfo);           
            continue;
            //return ;
        }
        else
        {
            nCanOpenDeviceNum++;
        }
    }

    m_nUseCbxNum = nCanOpenDeviceNum;
    if ((0 != m_nUseCbxNum) && (MV3D_RGBD_OK == nRet))
    {
        EnableCtrlBySwitch(TRUE);
    }
    else
    {
        cstrInfo.Format(_T("Please select right camera"));
        MessageBox(cstrInfo);
    }
}

void CBasicDemoDlg::OnBnClickedCloseDevBtn()
{
    int nRet = MV3D_RGBD_OK;
    CString cstrInfo;

    int nCurDevIndex = 0;
    for (int i = 0; i < MAX_DEVICE_NUM; i++)
    {
        nCurDevIndex = m_nCurDevIndex[i];
        if (0 == nCurDevIndex)
        {
            continue;
        }

        if (m_pcMyCamera[i])
        {
            nRet = m_pcMyCamera[i]->StopGrabbing();

            m_cwmdDisplay[i]->ShowWindow(FALSE);
            m_cwmdDisplay[i]->ShowWindow(TRUE);

            nRet = m_pcMyCamera[i]->Close();
            if (MV3D_RGBD_OK != nRet)
            {
                cstrInfo.Format(_T("Camera %d Open failed! error code:%#x"),i,nRet);
                MessageBox(cstrInfo);
            }

            delete m_pcMyCamera[i];
            m_pcMyCamera[i] = NULL;
        }
    }

    if (0 != m_nUseCbxNum)
    {        
        EnableCtrlBySwitch(FALSE);
    }

    m_nUseCbxNum = 0;
}

void CBasicDemoDlg::OnBnClickedOpenGrabBtn()
{
    int nRet = MV3D_RGBD_OK;
    CString cstrInfo;

    int nCurDevIndex = 0;
    for (int i = 0; i < MAX_DEVICE_NUM; i++)
    {
        nCurDevIndex = m_nCurDevIndex[i];
        if (0 == nCurDevIndex)
        {
            continue;
        }

        nRet = m_pcMyCamera[i]->StartGrabbing();
        if (MV3D_RGBD_OK == nRet)
        {
            nRet = m_pcMyCamera[i]->Process(m_hwndDisplay[i]);
            if (MV3D_RGBD_OK != nRet)
            {
                cstrInfo.Format(_T("Camera %d Create Thread failed! error code:%#x"),i,nRet);
                MessageBox(cstrInfo);
            }
        }
        else
        {
            cstrInfo.Format(_T("Camera %d Start Grabbing failed! error code:%#x"),i,nRet);
            MessageBox(cstrInfo);
        }
        UpdateData(FALSE);
    }

    EnableCtrlByGrabStat(TRUE);

}

void CBasicDemoDlg::OnBnClickedCloseGrabBtn()
{
    int nRet = MV3D_RGBD_OK; 
    CString cstrInfo;

    int nCurDevIndex = 0;
    for (int i = 0; i < MAX_DEVICE_NUM; i++)
    {
        nCurDevIndex = m_nCurDevIndex[i];
        if (0 == nCurDevIndex)
        {
            continue;
        }

        nRet = m_pcMyCamera[i]->StopGrabbing();
        if (MV3D_RGBD_OK != nRet)
        {
            cstrInfo.Format(_T("Camera %d Stop Grabbing failed! error code:%#x"),i,nRet);
            MessageBox(cstrInfo);
            return ;
        }
    }

    EnableCtrlByGrabStat(FALSE);
}

void CBasicDemoDlg::OnBnClickedSoftTriggerBtn()
{
    int nRet = MV3D_RGBD_OK;
    CString cstrInfo;

    int nCurDevIndex = 0;
    for (int i = 0; i < MAX_DEVICE_NUM; i++)
    {
        nCurDevIndex = m_nCurDevIndex[i];
        if (0 == nCurDevIndex)
        {
            continue;
        }

         nRet = m_pcMyCamera[i]->SoftTrigger();
         if (MV3D_RGBD_OK != nRet)
         {
            cstrInfo.Format(_T("Camera %d SoftTrigger failed! error code:%#x"),i,nRet);
            MessageBox(cstrInfo);
            return ;
         }
    }
}

void CBasicDemoDlg::OnBnClickedSaveRawButton()
{
    int nRet = MV3D_RGBD_OK;
    CString cstrInfo;

    int nCurDevIndex = 0;
    for (int i = 0; i < MAX_DEVICE_NUM; i++)
    {
        nCurDevIndex = m_nCurDevIndex[i];
        if (0 == nCurDevIndex)
        {
            continue;
        }

        nRet = m_pcMyCamera[i]->SaveRAW();
        if (MV3D_RGBD_OK != nRet)
        {
            cstrInfo.Format(_T("Camera %d Save Raw failed! error code:%#x"),i+1,nRet);
            MessageBox(cstrInfo);
            return ;
        }
    }    
}

void CBasicDemoDlg::OnClose()
{
    // 关闭程序，执行断开相机、销毁句柄操作
    PostQuitMessage(0);
    DeInitResources();
    CDialog::OnClose();
}

void CBasicDemoDlg::InitResources()
{
    GetVersionInfo();
    InitHwndHandle();
    InitCtrlStat();
}

void CBasicDemoDlg::DeInitResources()
{
    for (int i = 0; i < MAX_DEVICE_NUM; i++)
    {
        if (m_pcMyCamera[i])
        {
            delete m_pcMyCamera[i];
            m_pcMyCamera[i] = NULL;
        }
    }
}

void CBasicDemoDlg::InitCtrlStat()
{
    GetDlgItem(IDC_CAM_COMBOX_1)->EnableWindow(FALSE);
    GetDlgItem(IDC_CAM_COMBOX_2)->EnableWindow(FALSE);
    GetDlgItem(IDC_CAM_COMBOX_3)->EnableWindow(FALSE);
    GetDlgItem(IDC_CAM_COMBOX_4)->EnableWindow(FALSE);

    GetDlgItem(IDC_OPEN_DEV_BTN)->EnableWindow(FALSE);
    GetDlgItem(IDC_CLOSE_DEV_BTN)->EnableWindow(FALSE);
    GetDlgItem(IDC_OPEN_MEASURE_BTN)->EnableWindow(FALSE);
    GetDlgItem(IDC_CLOSE_MEASURE_BTN)->EnableWindow(FALSE);
    GetDlgItem(IDC_SOFT_TRIGGER_BTN)->EnableWindow(FALSE);
    GetDlgItem(IDC_SAVE_RAW_BUTTON)->EnableWindow(FALSE);

}

int CBasicDemoDlg::InitHwndHandle()  
{
    m_cwmdDisplay[0] = GetDlgItem(IDC_IMAGE_1);
    m_cwmdDisplay[1] = GetDlgItem(IDC_IMAGE_2);
    m_cwmdDisplay[2] = GetDlgItem(IDC_IMAGE_3);
    m_cwmdDisplay[3] = GetDlgItem(IDC_IMAGE_4);
    if (NULL == m_cwmdDisplay[0] || NULL == m_cwmdDisplay[1] || 
        NULL == m_cwmdDisplay[2] ||    NULL == m_cwmdDisplay[3])
    {
        return MV3D_RGBD_E_RESOURCE;
    }

    for (int i = 0; i < MAX_DEVICE_NUM; i++)
    {
        m_hwndDisplay[i] = m_cwmdDisplay[i]->GetSafeHwnd();
        if (NULL == m_hwndDisplay[i])
        {
            return MV3D_RGBD_E_RESOURCE;
        }
    }

    return MV3D_RGBD_OK;
}

void CBasicDemoDlg::GetVersionInfo()
{
    MV3D_RGBD_VERSION_INFO stVersion;
    CString cstrInfo;

    int nRet = MV3D_RGBD_GetSDKVersion(&stVersion) ;
    if (MV3D_RGBD_OK != nRet)
    {
       cstrInfo.Format(_T("MV3D_RGBD_GetSDKVersion failed! error code:%#x"),nRet);
       MessageBox(cstrInfo);
       return ;
    }

   cstrInfo.Format(_T("%d.%d.%d"),stVersion.nMajor, stVersion.nMinor, stVersion.nRevision);
   m_ctrlVersionEdit.SetWindowText(cstrInfo);

}

void CBasicDemoDlg::EnableCtrlBySwitch(BOOL bOpenDev)
{
    GetDlgItem(IDC_DEVICE_COMBOX)->EnableWindow(!bOpenDev);
    GetDlgItem(IDC_ENUM_BUTTON)->EnableWindow(!bOpenDev);
    GetDlgItem(IDC_CAM_COMBOX_1)->EnableWindow(!bOpenDev);
    GetDlgItem(IDC_CAM_COMBOX_2)->EnableWindow(!bOpenDev);
    GetDlgItem(IDC_CAM_COMBOX_3)->EnableWindow(!bOpenDev);
    GetDlgItem(IDC_CAM_COMBOX_4)->EnableWindow(!bOpenDev);

    GetDlgItem(IDC_OPEN_DEV_BTN)->EnableWindow(!bOpenDev);
    GetDlgItem(IDC_CLOSE_DEV_BTN)->EnableWindow(bOpenDev);
    GetDlgItem(IDC_OPEN_MEASURE_BTN)->EnableWindow(bOpenDev);
    GetDlgItem(IDC_SOFT_TRIGGER_BTN)->EnableWindow(bOpenDev);

    GetDlgItem(IDC_CLOSE_MEASURE_BTN)->EnableWindow(FALSE);
    GetDlgItem(IDC_SAVE_RAW_BUTTON)->EnableWindow(FALSE);
}

void CBasicDemoDlg::EnableCtrlByGrabStat(BOOL bStartGrab)
{
    GetDlgItem(IDC_OPEN_MEASURE_BTN)->EnableWindow(!bStartGrab);
    GetDlgItem(IDC_CLOSE_MEASURE_BTN)->EnableWindow(bStartGrab);
    GetDlgItem(IDC_SAVE_RAW_BUTTON)->EnableWindow(bStartGrab);
}

bool CBasicDemoDlg::IsRepeatIndex()
{
    // 获取当前选择的设备信息
    m_nCurDevIndex[0] = m_ctrlCamCb1.GetCurSel();
    m_nCurDevIndex[1] = m_ctrlCamCb2.GetCurSel();
    m_nCurDevIndex[2] = m_ctrlCamCb3.GetCurSel();
    m_nCurDevIndex[3] = m_ctrlCamCb4.GetCurSel();

    for (int i = 0; i < MAX_DEVICE_NUM - 1 ; i++)
    {
        if (0 == m_nCurDevIndex[i])
        {
             continue;
        }

        for (int j = i + 1; j < MAX_DEVICE_NUM; j++)
        {
            if ((m_nCurDevIndex[i] == m_nCurDevIndex[j]) )
            {
                return true;
            }
        }
    }

    return false;
}

// ch:判断字符类型 | en:str type
bool CBasicDemoDlg::IsStrUTF8(const char* pBuffer, int size)
{
    if (size < 0)
    {
        return false;
    }

    bool IsUTF8 = true;
    unsigned char* start = (unsigned char*)pBuffer;
    unsigned char* end = (unsigned char*)pBuffer + size;
    if (NULL == start ||
        NULL == end)
    {
        return false;
    }
    while (start < end)
    {
        if (*start < 0x80) // ch:(10000000): 值小于0x80的为ASCII字符 | en:(10000000): if the value is smaller than 0x80, it is the ASCII character
        {
            start++;
        }
        else if (*start < (0xC0)) // ch:(11000000): 值介于0x80与0xC0之间的为无效UTF-8字符 | en:(11000000): if the value is between 0x80 and 0xC0, it is the invalid UTF-8 character
        {
            IsUTF8 = false;
            break;
        }
        else if (*start < (0xE0)) // ch:(11100000): 此范围内为2字节UTF-8字符  | en: (11100000): if the value is between 0xc0 and 0xE0, it is the 2-byte UTF-8 character
        {
            if (start >= end - 1)
            {
                break;
            }

            if ((start[1] & (0xC0)) != 0x80)
            {
                IsUTF8 = false;
                break;
            }

            start += 2;
        }
        else if (*start < (0xF0)) // ch:(11110000): 此范围内为3字节UTF-8字符 | en: (11110000): if the value is between 0xE0 and 0xF0, it is the 3-byte UTF-8 character 
        {
            if (start >= end - 2)
            {
                break;
            }

            if ((start[1] & (0xC0)) != 0x80 || (start[2] & (0xC0)) != 0x80)
            {
                IsUTF8 = false;
                break;
            }

            start += 3;
        }
        else
        {
            IsUTF8 = false;
            break;
        }
    }

    return IsUTF8;
}

// ch: 单字节转宽字节 | en: char convert to Wchar
bool CBasicDemoDlg::Char2Wchar(const char *pStr, wchar_t *pOutWStr, int nOutStrSize)
{
    if (!pStr || !pOutWStr)
    {
        return false;
    }

    bool bIsUTF = IsStrUTF8(pStr, strlen(pStr));
    UINT nChgType = bIsUTF ? CP_UTF8 : CP_ACP;

    int iLen = MultiByteToWideChar(nChgType, 0, (LPCSTR)pStr, -1, NULL, 0);

    memset(pOutWStr, 0, sizeof(wchar_t) * nOutStrSize);

    if (iLen >= nOutStrSize)
    {
        iLen = nOutStrSize - 1;
    }

    MultiByteToWideChar(nChgType, 0, (LPCSTR)pStr, -1, pOutWStr, iLen);

    pOutWStr[iLen] = 0;

    return true;
}

// ch: 宽字节转单字节 | en: Wchar convert to char
bool CBasicDemoDlg::Wchar2char(wchar_t *pOutWStr, char *pStr)
{
    if (!pStr || !pOutWStr)
    {
        return false;
    }

    int nLen =  WideCharToMultiByte(CP_ACP, 0, pOutWStr, wcslen(pOutWStr), NULL, 0, NULL, NULL);

    WideCharToMultiByte(CP_ACP, 0 , pOutWStr, wcslen(pOutWStr), pStr, nLen, NULL, NULL);

    pStr[nLen] = '\0';

    return true;
}


BOOL CBasicDemoDlg::PreTranslateMessage(MSG* pMsg)
{
    // 屏蔽ESC和ENTER按键
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

