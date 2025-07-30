#pragma once
#include "afxwin.h"
class CEditEx :
    public CEdit
{
public:
    CEditEx();
    ~CEditEx();
    DECLARE_MESSAGE_MAP()
    afx_msg void OnChar(UINT nChar, UINT nRepCnt, UINT nFlags);
};

