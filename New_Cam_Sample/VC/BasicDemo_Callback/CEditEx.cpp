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
    //����С����
    if (nChar == '.')
    {
        CString str;
        GetWindowText(str);

        // ���Ƶ�һλΪС��
        if (str.GetLength() == 0)
        {
            // ��һλ����С����
            MessageBox(_T("The first cannot be the decimal point"));
            return;
        }
        // ����ֻ������һ��С����
        if (str.Find('.') == -1)
        {
            CEdit::OnChar(nChar, nRepCnt, nFlags);
        }
        else
        {
            // С������ֵڶ���
            MessageBox(_T("The decimal point can only be entered once"));
        }
    }
    // ���ֺ��˸��
    else if ((nChar >= '0' && nChar <= '9') || nChar == 0x08)
    {
        CEdit::OnChar(nChar, nRepCnt, nFlags);
    }else
    {
        // ���ַ����֣��˸��
        MessageBox(_T("Please enter correct type!"));
    }
}

