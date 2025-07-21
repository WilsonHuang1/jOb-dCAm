#pragma once
#define GLFW_EXPOSE_NATIVE_WIN32
#include "afxwin.h"
#include "CEditEx.h"
#include <map>
#include <vector>
#include "Mv3dRgbdApi.h"
#include "Mv3dRgbdDefine.h"
#include "Mv3dRgbdImgProc.h"

#define EXPOSURE_TIME                   "ExposureTime"
#define GAIN                            "Gain"

#define Camera_Width                    "Width"
#define Camera_Height                   "Height"

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


// CBasicDemo_PointCloudDlg 对话框
class CBasicDemo_PointCloudDlg : public CDialog
{
// 构造
public:
    CBasicDemo_PointCloudDlg(CWnd* pParent = NULL);    // 标准构造函数

// 对话框数据
    enum { IDD = IDD_BasicDemo_PointCloud_DIALOG };

protected:
    virtual void DoDataExchange(CDataExchange* pDX);

    int        InitResources();
    void    DeInitResources();

// 实现
protected:
    HICON m_hIcon;

    // 生成的消息映射函数
    virtual BOOL OnInitDialog();
    afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
    afx_msg void OnPaint();
    afx_msg HCURSOR OnQueryDragIcon();
    DECLARE_MESSAGE_MAP()
public:
    afx_msg void OnBnClickedEnumButton();
    afx_msg void OnBnClickedOpenButton();
    afx_msg void OnBnClickedCloseButton();
    afx_msg void OnBnClickedStopGrabbingButton();
    afx_msg void OnBnClickedStartGrabbingButton();
    afx_msg void OnBnClickedGetParameterButton();
    afx_msg void OnBnClickedSetParameterButton();
    afx_msg void OnClose();

    void    SetCtrlWhenInit();
    void    SetCtrlWhenOpen();
    void    SetCtrlWhenClose();
    void    SetCtrlWhenStart();
    void    SetCtrlWhenStop();

private:
    BOOL    PreTranslateMessage(MSG* pMsg);
                                                                                     
    static void*  __stdcall WINAPI ProcessThread(void* pUser);  // 图像显示线程
    int        CloseDevice(void);
    int        SaveImage(Mv3dRgbdPointCloudFileType enFileType);

    int        Draw(MV3D_RGBD_DRAW_PARAM* pstParam);

private:
    unsigned int            m_nDevNum;
    MV3D_RGBD_DEVICE_INFO   m_stDeviceInfoList[20];             // 设备信息列表

    MV3D_RGBD_IMAGE_DATA    m_stImageInfo;                        // 图片参数信息
    bool                    m_bStartJob;                        // 是否工作线程已开启

    void*                   m_handle;                           // 设备句柄
    bool                    m_bConnect;                         // 是否设备已连接

    HANDLE hProcessThread;  //渲染线程

    int                     m_MaxImageSize;                     // 图像最大尺寸
    unsigned char*          m_pcDataBuf;                        // 存储图像数据

	int                     m_MaxRgbTransImageSize;             // RGB转换图像大小
	unsigned char*          m_pcRgbTransDataBuf;                // RGB转换图像数据

    CCriticalSection        m_criSection;                       // 临界区
    BITMAPINFO*             m_bBitmapInfo;

    bool                    m_bCalibInfo;                       // 是否获取标定信息

    unsigned int            m_nDisplayMapDataLen;
    unsigned char*          m_pDisplayMapData;
	int                     m_nPointCloudType;

private:
    CButton     m_ctrlOpenButton;
    CButton     m_ctrlCloseButton;
    CButton     m_ctrlContinusModeRadio;
    CButton     m_ctrlTriggerModeRadio;
    CButton     m_ctrlSoftwareTriggerCheck;
    CButton     m_ctrlStartGrabbingButton;
    CButton     m_ctrlStopGrabbingButton;
    CButton     m_ctrlSoftwareOnceButton;
    CButton     m_ctrlGetParameterButton;
    CButton     m_ctrlSetParameterButton;

private:
    CEditEx     m_ctrlExposureEdit;
    CEditEx     m_ctrlGainEdit;
    CComboBox   m_ctrlDeviceCombo;
    CComboBox    m_ctrlImageAlignCombo;
    CComboBox    m_ctrlPointCloudType;
public:
    afx_msg void OnBnClickedSavePlyBtn();
    afx_msg void OnBnClickedSavePlyBinaryBtn();
    afx_msg void OnBnClickedSavePcdBtn();
    afx_msg void OnBnClickedSavePcdBinaryBtn();
	afx_msg void OnCbnSelchangeDeviceCombo3();
};
