
#include "../common/common.hpp"

void DumpParam(void* pHandle, char *pParamName)
{
    MV3D_RGBD_PARAM pstValue;
    int nRet = MV3D_RGBD_OK;
    memset(&pstValue, 0, sizeof(MV3D_RGBD_PARAM));
    nRet = MV3D_RGBD_GetParam(pHandle, pParamName, &pstValue);
    if (MV3D_RGBD_OK != nRet)
    {
        return;
    }

    if (ParamType_Int == pstValue.enParamType)
    {
        LOG("ParamName : %s, Current Value: %I64d ,Max Value: %I64d ,Min Value: %I64d \r\n",
            pParamName, pstValue.ParamInfo.stIntParam.nCurValue, pstValue.ParamInfo.stIntParam.nMax,
            pstValue.ParamInfo.stIntParam.nMin);
    }
    else if (ParamType_Float == pstValue.enParamType)
    {
        LOG("ParamName : %s, Current Value: %f ,Max Value: %f ,Min Value: %f \r\n",
        pParamName, pstValue.ParamInfo.stFloatParam.fCurValue, pstValue.ParamInfo.stFloatParam.fMax,
            pstValue.ParamInfo.stFloatParam.fMin);
    } 
    else if (ParamType_Enum == pstValue.enParamType)
    {
        LOG("ParamName : %s, Current Value: %d ,Supported Number: %d \r\n",
            pParamName, pstValue.ParamInfo.stEnumParam.nCurValue, pstValue.ParamInfo.stEnumParam.nSupportedNum);

        LOG("            %s Enum options :\r\n", pParamName);
        for (int i = 0;i < pstValue.ParamInfo.stEnumParam.nSupportedNum ;i++)
        {
            LOG("            Support Value is [%d] \r\n",pstValue.ParamInfo.stEnumParam.nSupportValue[i]);
        }
    } 
    else if (ParamType_Bool == pstValue.enParamType)
    {
        LOG("ParamName : %s, Current BoolValue: %d \r\n", pParamName, pstValue.ParamInfo.bBoolParam);
    } 
    else if (ParamType_String == pstValue.enParamType)
    {
        LOG("ParamName : %s, Current String MaxLength: %d,Current String Value: %s\r\n", pParamName, pstValue.ParamInfo.stStringParam.nMaxLength,
            pstValue.ParamInfo.stStringParam.chCurValue);
    }
}

int DumpAllParams(void* pHandle)
{
    DumpParam(pHandle, MV3D_RGBD_INT_WIDTH);
    DumpParam(pHandle, MV3D_RGBD_INT_HEIGHT);
    DumpParam(pHandle, MV3D_RGBD_ENUM_WORKINGMODE);
    DumpParam(pHandle, MV3D_RGBD_ENUM_PIXELFORMAT);
    DumpParam(pHandle, MV3D_RGBD_ENUM_IMAGEMODE);
    DumpParam(pHandle, MV3D_RGBD_FLOAT_GAIN);
    DumpParam(pHandle, MV3D_RGBD_FLOAT_EXPOSURETIME);
    DumpParam(pHandle, MV3D_RGBD_FLOAT_FRAMERATE);
    DumpParam(pHandle, MV3D_RGBD_ENUM_TRIGGERSELECTOR);
    DumpParam(pHandle, MV3D_RGBD_ENUM_TRIGGERMODE);
    DumpParam(pHandle, MV3D_RGBD_ENUM_TRIGGERSOURCE);
    DumpParam(pHandle, MV3D_RGBD_FLOAT_TRIGGERDELAY);
    DumpParam(pHandle, MV3D_RGBD_INT_IMAGEALIGN);
    return MV3D_RGBD_OK;
}

int main(int argc,char** argv)
{
    MV3D_RGBD_VERSION_INFO stVersion;
    ASSERT_OK( MV3D_RGBD_GetSDKVersion(&stVersion) );
    LOGD("dll version: %d.%d.%d", stVersion.nMajor, stVersion.nMinor, stVersion.nRevision);
    
    ASSERT_OK( MV3D_RGBD_Initialize() );

    unsigned int nDevNum = 0;
    ASSERT_OK(MV3D_RGBD_GetDeviceNumber(DeviceType_Ethernet | DeviceType_USB | DeviceType_Ethernet_Vir | DeviceType_USB_Vir, &nDevNum));
    LOGD("MV3D_RGBD_GetDeviceNumber success! nDevNum:%d.", nDevNum);
    ASSERT(nDevNum);

    // ch:枚举设备 | en:Enumerate device 
    std::vector<MV3D_RGBD_DEVICE_INFO> devs(nDevNum);
    ASSERT_OK(MV3D_RGBD_GetDeviceList(DeviceType_Ethernet | DeviceType_USB | DeviceType_Ethernet_Vir | DeviceType_USB_Vir, &devs[0], nDevNum, &nDevNum));
    for (unsigned int i = 0; i < nDevNum; i++)
    {  
        if (DeviceType_Ethernet == devs[i].enDeviceType || DeviceType_Ethernet_Vir == devs[i].enDeviceType)
        {
            LOG("Index[%d]. SerialNum[%s] IP[%s] name[%s] Type[%d].\r\n", i, devs[i].chSerialNumber, devs[i].SpecialInfo.stNetInfo.chCurrentIp, devs[i].chModelName, (devs[i].nDevTypeInfo & 0xff000000) >> 24);
        }
        else if (DeviceType_USB == devs[i].enDeviceType || DeviceType_USB_Vir == devs[i].enDeviceType)
        {
            LOG("Index[%d]. SerialNum[%s] UsbProtocol[%d] name[%s] Type[%d].\r\n", i, devs[i].chSerialNumber, devs[i].SpecialInfo.stUsbInfo.enUsbProtocol, devs[i].chModelName, (devs[i].nDevTypeInfo & 0xff000000) >> 24);
        }
    }

    // ch:打开设备 | en:Open device
    void* handle = NULL;
    unsigned int nIndex = 0;
    ASSERT_OK(MV3D_RGBD_OpenDevice(&handle, &devs[nIndex]));
    LOGD("OpenDevice success.");

    ASSERT_OK(DumpAllParams(handle));

    ASSERT_OK(MV3D_RGBD_CloseDevice(&handle));
    ASSERT_OK(MV3D_RGBD_Release());
    
    LOGD("Main done!");
    return  0;
}

