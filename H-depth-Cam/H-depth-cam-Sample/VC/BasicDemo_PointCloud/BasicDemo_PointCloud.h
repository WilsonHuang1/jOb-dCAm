
// BasicDemo_PointCloud.h : PROJECT_NAME Ӧ�ó������ͷ�ļ�
//

#pragma once

#ifndef __AFXWIN_H__
    #error "�ڰ������ļ�֮ǰ������stdafx.h�������� PCH �ļ�"
#endif

#include "resource.h"        // ������


// CBasicDemo_PointCloudApp:
// �йش����ʵ�֣������ BasicDemo_PointCloud.cpp
//

class CBasicDemo_PointCloudApp : public CWinAppEx
{
public:
    CBasicDemo_PointCloudApp();

// ��д
    public:
    virtual BOOL InitInstance();

// ʵ��

    DECLARE_MESSAGE_MAP()
};

extern CBasicDemo_PointCloudApp theApp;