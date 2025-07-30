// BasicDemoDlg.h : 头文件
//
#pragma once
#include "afxwin.h"
#include "MyCamera.h"
#include "BasicDemo.h"
#include "CEditEx.h"
#include <map>

#define MAX_DEVICE_NUM          4

typedef struct _MV3D_RGBD_DEVICE_SERIAL_
{
    unsigned int            nIndex;                         // ch:设备索引
    char*                   pSerial;                        // ch:设备序列号

}MV3D_RGBD_DEVICE_SERIAL;

// CBasicDemoDlg 对话框
class CBasicDemoDlg : public CDialog
{
// 构造
public:
    CBasicDemoDlg(CWnd* pParent = NULL);    // 标准构造函数

// 对话框数据
    enum { IDD = IDD_MultipleCamera_DIALOG };

    protected:
    virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

    void InitResources();
    void DeInitResources();
    void InitCtrlStat();
    int InitHwndHandle();             
    int InitDisplayWindow();          
    void GetVersionInfo();
    void EnableCtrlBySwitch(BOOL bOpenDev);
    void EnableCtrlByGrabStat(BOOL bStartGrab);

    bool IsStrUTF8(const char* pBuffer, int size);// ch:判断字符类型 | en:str type
    bool Char2Wchar(const char *pStr, wchar_t *pOutWStr, int nOutStrSize);    // ch: 单字节转宽字节 | en: char convert to Wchar
    bool Wchar2char(wchar_t *pOutWStr, char *pStr);
    bool IsRepeatIndex();
// 实现
protected:
    HICON m_hIcon;

    // 生成的消息映射函数
    virtual BOOL OnInitDialog();
    afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
    afx_msg void OnPaint();
    afx_msg HCURSOR OnQueryDragIcon();
    DECLARE_MESSAGE_MAP()

private:
    static void*  __stdcall WINAPI ProcessThread(void* pUser);            // 图像显示线程
    BOOL    PreTranslateMessage(MSG* pMsg);

public:
    afx_msg void OnBnClickedEnumButton();
    afx_msg void OnBnClickedOpenDevBtn();
    afx_msg void OnBnClickedCloseDevBtn();
    afx_msg void OnBnClickedOpenGrabBtn();
    afx_msg void OnBnClickedCloseGrabBtn();
    afx_msg void OnBnClickedSoftTriggerBtn();
    afx_msg void OnBnClickedSaveRawButton();
    afx_msg void OnClose();

private:
    CMyCamera*   m_pcMyCamera[MAX_DEVICE_NUM];      // ch:CMyCamera封装了常用接口
    uint32_t      m_nDeviceNumber;            //设备数
    uint32_t      m_nUseCbxNum;
    
    //相关控件
    CEditEx       m_ctrlVersionEdit;
    CEditEx       m_ctrlDevCountEdit;
    CComboBox     m_ctrlDeviceCombo;
    CComboBox     m_ctrlCamCb1;
    CComboBox     m_ctrlCamCb2;
    CComboBox     m_ctrlCamCb3;
    CComboBox     m_ctrlCamCb4;

    HWND  m_hwndDisplay[MAX_DEVICE_NUM];            // ch:显示句柄 | en:Display window
    CWnd* m_cwmdDisplay[MAX_DEVICE_NUM];
    int   m_nCurDevIndex[MAX_DEVICE_NUM];
    MV3D_RGBD_DEVICE_INFO        m_stDeviceInfoList[20];             // 设备信息列表

};
