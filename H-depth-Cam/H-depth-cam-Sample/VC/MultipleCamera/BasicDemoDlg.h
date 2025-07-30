// BasicDemoDlg.h : ͷ�ļ�
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
    unsigned int            nIndex;                         // ch:�豸����
    char*                   pSerial;                        // ch:�豸���к�

}MV3D_RGBD_DEVICE_SERIAL;

// CBasicDemoDlg �Ի���
class CBasicDemoDlg : public CDialog
{
// ����
public:
    CBasicDemoDlg(CWnd* pParent = NULL);    // ��׼���캯��

// �Ի�������
    enum { IDD = IDD_MultipleCamera_DIALOG };

    protected:
    virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV ֧��

    void InitResources();
    void DeInitResources();
    void InitCtrlStat();
    int InitHwndHandle();             
    int InitDisplayWindow();          
    void GetVersionInfo();
    void EnableCtrlBySwitch(BOOL bOpenDev);
    void EnableCtrlByGrabStat(BOOL bStartGrab);

    bool IsStrUTF8(const char* pBuffer, int size);// ch:�ж��ַ����� | en:str type
    bool Char2Wchar(const char *pStr, wchar_t *pOutWStr, int nOutStrSize);    // ch: ���ֽ�ת���ֽ� | en: char convert to Wchar
    bool Wchar2char(wchar_t *pOutWStr, char *pStr);
    bool IsRepeatIndex();
// ʵ��
protected:
    HICON m_hIcon;

    // ���ɵ���Ϣӳ�亯��
    virtual BOOL OnInitDialog();
    afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
    afx_msg void OnPaint();
    afx_msg HCURSOR OnQueryDragIcon();
    DECLARE_MESSAGE_MAP()

private:
    static void*  __stdcall WINAPI ProcessThread(void* pUser);            // ͼ����ʾ�߳�
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
    CMyCamera*   m_pcMyCamera[MAX_DEVICE_NUM];      // ch:CMyCamera��װ�˳��ýӿ�
    uint32_t      m_nDeviceNumber;            //�豸��
    uint32_t      m_nUseCbxNum;
    
    //��ؿؼ�
    CEditEx       m_ctrlVersionEdit;
    CEditEx       m_ctrlDevCountEdit;
    CComboBox     m_ctrlDeviceCombo;
    CComboBox     m_ctrlCamCb1;
    CComboBox     m_ctrlCamCb2;
    CComboBox     m_ctrlCamCb3;
    CComboBox     m_ctrlCamCb4;

    HWND  m_hwndDisplay[MAX_DEVICE_NUM];            // ch:��ʾ��� | en:Display window
    CWnd* m_cwmdDisplay[MAX_DEVICE_NUM];
    int   m_nCurDevIndex[MAX_DEVICE_NUM];
    MV3D_RGBD_DEVICE_INFO        m_stDeviceInfoList[20];             // �豸��Ϣ�б�

};
