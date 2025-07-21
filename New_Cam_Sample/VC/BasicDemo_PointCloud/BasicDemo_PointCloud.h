
// BasicDemo_PointCloud.h : PROJECT_NAME 应用程序的主头文件
//

#pragma once

#ifndef __AFXWIN_H__
    #error "在包含此文件之前包含“stdafx.h”以生成 PCH 文件"
#endif

#include "resource.h"        // 主符号


// CBasicDemo_PointCloudApp:
// 有关此类的实现，请参阅 BasicDemo_PointCloud.cpp
//

class CBasicDemo_PointCloudApp : public CWinAppEx
{
public:
    CBasicDemo_PointCloudApp();

// 重写
    public:
    virtual BOOL InitInstance();

// 实现

    DECLARE_MESSAGE_MAP()
};

extern CBasicDemo_PointCloudApp theApp;