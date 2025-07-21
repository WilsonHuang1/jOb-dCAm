#include "stdafx.h"
#include "CEditEX.h"


CEditEx::CEditEx()
{
}


CEditEx::~CEditEx()
{
}
BEGIN_MESSAGE_MAP(CEditEx, CEdit)
    ON_WM_CHAR()
END_MESSAGE_MAP()


void CEditEx::OnChar(UINT nChar, UINT nRepCnt, UINT nFlags)
{
    //处理小数点
    if (nChar == '.')
    {
        CString str;
        GetWindowText(str);

        // 限制第一位为小数
        if (str.GetLength() == 0)
        {
            // 第一位输入小数点
            MessageBox(_T("The first cannot be the decimal point"));
            return;
        }
        // 限制只运行有一个小数点
        if (str.Find('.') == -1)
        {
            CEdit::OnChar(nChar, nRepCnt, nFlags);
        }
        else
        {
            // 小数点出现第二次
            MessageBox(_T("The decimal point can only be entered once"));
        }
    }
    // 数字和退格键
    else if ((nChar >= '0' && nChar <= '9') || nChar == 0x08)
    {
        CEdit::OnChar(nChar, nRepCnt, nFlags);
    }else
    {
        // 出现非数字，退格键
        MessageBox(_T("Please enter correct type!"));
    }
}

