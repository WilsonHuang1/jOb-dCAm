
#include "../common/common.hpp"

void WaitForKeyPress(void)
{
    while(!_kbhit())
    {
        Sleep(10);
    }
    _getch();
}

void __stdcall  CallBackFunc(MV3D_RGBD_FRAME_DATA* pstFrameData, void* pUser)
{
    if (NULL != pstFrameData)
    {
        for(int i = 0; i < pstFrameData->nImageCount; i++)
        {
            if (ImageType_Depth == pstFrameData->stImageData[i].enImageType)
            {
                LOGD("MV3D_RGBD_FetchFrame ImageType_Depth success: FrameNum[%d] Width[%d] Height[%d] Len[%d] Coordinate[%d]!\r\n", pstFrameData->stImageData[i].nFrameNum,
                    pstFrameData->stImageData[i].nWidth, pstFrameData->stImageData[i].nHeight, pstFrameData->stImageData[i].nDataLen, pstFrameData->stImageData[i].enCoordinateType);
            }
            else if (ImageType_RGB8_Planar == pstFrameData->stImageData[i].enImageType)
            {
                LOGD("MV3D_RGBD_FetchFrame ImageType_RGB8_Planar success: FrameNum[%d] Width[%d] Height[%d] Len[%d] Coordinate[%d]!\r\n", pstFrameData->stImageData[i].nFrameNum,
                    pstFrameData->stImageData[i].nWidth, pstFrameData->stImageData[i].nHeight, pstFrameData->stImageData[i].nDataLen, pstFrameData->stImageData[i].enCoordinateType);
            }
            else
            {
                LOGD("MV3D_RGBD_FetchFrame success: FrameNum[%d] Width[%d] Height[%d] Len[%d] Coordinate[%d] ImageType[%ld] StreamType[%d]!\r\n", pstFrameData->stImageData[i].nFrameNum,
                    pstFrameData->stImageData[i].nWidth, pstFrameData->stImageData[i].nHeight, pstFrameData->stImageData[i].nDataLen, pstFrameData->stImageData[i].enCoordinateType, pstFrameData->stImageData[i].enImageType, pstFrameData->stImageData[i].enStreamType);
            }
        }
    }
    else
    {
        LOGD("pstFrameData is null!\r\n");
    }
}

int main(int argc,char** argv)
{
    MV3D_RGBD_VERSION_INFO stVersion;
    ASSERT_OK( MV3D_RGBD_GetSDKVersion(&stVersion) );
    LOGD("dll version: %d.%d.%d", stVersion.nMajor, stVersion.nMinor, stVersion.nRevision);

    ASSERT_OK(MV3D_RGBD_Initialize());

    unsigned int nDevNum = 0;
    ASSERT_OK(MV3D_RGBD_GetDeviceNumber(DeviceType_Ethernet | DeviceType_USB | DeviceType_Ethernet_Vir | DeviceType_USB_Vir, &nDevNum));
    LOGD("MV3D_RGBD_GetDeviceNumber success! nDevNum:%d.", nDevNum);
    ASSERT(nDevNum);

    // ch:枚举设备 | en:Enumerate devices
    std::vector<MV3D_RGBD_DEVICE_INFO> devs(nDevNum);
    ASSERT_OK(MV3D_RGBD_GetDeviceList(DeviceType_Ethernet | DeviceType_USB | DeviceType_Ethernet_Vir | DeviceType_USB_Vir, &devs[0], nDevNum, &nDevNum));
    for (unsigned int i = 0; i < nDevNum; i++)
    {  
        if (DeviceType_Ethernet == devs[i].enDeviceType || DeviceType_Ethernet_Vir == devs[i].enDeviceType)
        {
            LOG("Index[%d]. SerialNum[%s] IP[%s] name[%s] Type[%d].\r\n",
                i, devs[i].chSerialNumber, devs[i].SpecialInfo.stNetInfo.chCurrentIp, devs[i].chModelName,
                (devs[i].nDevTypeInfo & 0xff000000) >> 24);
        }
        else if (DeviceType_USB == devs[i].enDeviceType || DeviceType_USB_Vir == devs[i].enDeviceType)
        {
            LOG("Index[%d]. SerialNum[%s] UsbProtocol[%d] name[%s] Type[%d].\r\n",
                i, devs[i].chSerialNumber, devs[i].SpecialInfo.stUsbInfo.enUsbProtocol, devs[i].chModelName,
                (devs[i].nDevTypeInfo & 0xff000000) >> 24);
        }
    }

    // ch:打开设备 | en:Open device
    void* handle = NULL; 
    unsigned int nIndex = 0;
    ASSERT_OK(MV3D_RGBD_OpenDevice(&handle, &devs[nIndex]));
    LOGD("OpenDevice success.");

    // ch:注册图像回调 | en:Register image callback
    ASSERT_OK(MV3D_RGBD_RegisterFrameCallBack(handle, CallBackFunc, handle));

    // ch:开始取流 | en:Start work
    ASSERT_OK(MV3D_RGBD_Start(handle));
    LOGD("Start work success.");

    LOGD("Press a key to stop grabbing.\n");
    WaitForKeyPress();

    ASSERT_OK(MV3D_RGBD_Stop(handle));
    ASSERT_OK(MV3D_RGBD_CloseDevice(&handle));
    ASSERT_OK(MV3D_RGBD_Release());
    
    LOGD("Main done!");
    return  0;
}

