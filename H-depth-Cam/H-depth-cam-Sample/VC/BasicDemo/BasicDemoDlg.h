#pragma once
#include "afxwin.h"
#include "CEditEx.h"
#include "Mv3dRgbdApi.h"
#include "Mv3dRgbdDefine.h"
#include "Mv3dRgbdImgProc.h"
#include <map>
#include <vector>

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


// CBasicDemoDlg �Ի���
class CBasicDemoDlg : public CDialog
{
// ����
public:
    CBasicDemoDlg(CWnd* pParent = NULL);    // ��׼���캯��

// �Ի�������
    enum { IDD = IDD_BasicDemo_DIALOG };

protected:
    virtual void DoDataExchange(CDataExchange* pDX);

    int        InitResources();
    void    DeInitResources();

// ʵ��
protected:
    HICON m_hIcon;

    // ���ɵ���Ϣӳ�亯��
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
                                                                                     
    static void*  __stdcall WINAPI ProcessThread(void* pUser);  // ͼ����ʾ�߳�
    int        CloseDevice(void);
    int        SaveImage(Mv3dRgbdFileType enFileType);

    int        Draw(MV3D_RGBD_DRAW_PARAM* pstParam);
    int        Display(void* handle, void* hWnd, MV3D_RGBD_IMAGE_DATA* pstDisplayImage);

private:
    unsigned int            m_nDevNum;
    MV3D_RGBD_DEVICE_INFO   m_stDeviceInfoList[50];             // �豸��Ϣ�б�

    MV3D_RGBD_IMAGE_DATA    m_stImageInfo;                        // ͼƬ������Ϣ
    bool                    m_bStartJob;                        // �Ƿ����߳��ѿ���

    void*                   m_handle;                           // �豸���
    void*                   m_hWndDisplay;                      // ��ʾ���ھ��
    void*                   m_hWndDisplay2;                     // ��ʾ���ھ��2
    bool                    m_bConnect;                         // �Ƿ��豸������

    HANDLE                  m_hProcessThread;                   //��Ⱦ�߳�

    int                     m_MaxImageSize;                     // ͼ�����ߴ�
    unsigned char*          m_pcDataBuf;                        // �洢ͼ������

    CCriticalSection        m_criSection;                       // �ٽ���
    BITMAPINFO*             m_bBitmapInfo;

    unsigned int            m_nDisplayMapDataLen;
    unsigned char*          m_pDisplayMapData; 
	int                     m_nConvertColorMapMode;
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
    CEditEx     m_ctrlWidthEdit;
    CEditEx     m_ctrlHeightEdit;
    CEditEx     m_ctrlDepthEdit;
    CComboBox   m_ctrlDeviceCombo;
    CComboBox    m_ctrlImageAlignCombo;
    CComboBox    m_ctrlConvertColorMapCombo;
public:

    afx_msg void OnBnClickedSaveTiffBtn();
    afx_msg void OnBnClickedSaveBmpBtn();
    afx_msg void OnBnClickedSaveJpgBtn();
    afx_msg void OnBnClickedSaveRawBtn();
    afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnCbnSelchangeDeviceCombo3();
};
