
#include "../common/common.hpp"

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

    LOG("Please enter the number of cameras to be connected:\n");
    int nLinkCount = 0;
    scanf("%d",&nLinkCount);

    while(nLinkCount > nDevNum || 0 >= nLinkCount)
    {
        LOG("The input is greater than the total number of devices.Please input again:\n");
        scanf("%d",&nLinkCount);
    }

    std::vector<unsigned int> nIndexArray(nLinkCount);
    for (int i = 0;i < nLinkCount;i++)
    {
        LOG("Please enter the Index number of No.%d camera to be connected:\n",i+1);        
        scanf("%d",&nIndexArray[i]);
        LOG("Connected camera:%d \r\n", nIndexArray[i]);

        if ((nDevNum  <= nIndexArray[i]) ||
            (0 > nIndexArray[i]))
        {
            LOG("enter error!\r\n");
        }
        else
        {
            continue;
        }
    }
 
    std::vector<void*> pHandle(nLinkCount);
    unsigned int nIndex = 0;
    for (int i = 0;i < nLinkCount;i++)
    {
        nIndex = nIndexArray[i];
        // ch:打开设备 | en:Open device   
        ASSERT_OK(MV3D_RGBD_OpenDevice(&pHandle[i], &devs[nIndex]));
        LOGD("OpenDevice index(%d) success.",nIndexArray[i]);

        // ch:开始取流 | en:Start work
        ASSERT_OK(MV3D_RGBD_Start(pHandle[i]));
        LOGD("Start work success.");
    }

    LOGD("While loop to fetch frame\r\n");

    int cam_index = 0;
    MV3D_RGBD_FRAME_DATA stFrameData = {0};
    BOOL bExit_Main = FALSE;
    while (!bExit_Main)
    {
        if (cam_index >= nLinkCount)
        {
            cam_index = 0;
        }

        // ch:获取图像数据 | en:Get image data
        int nRet = MV3D_RGBD_FetchFrame(pHandle[cam_index], &stFrameData, 5000);
        if (MV3D_RGBD_OK == nRet)
        {
            for(int i = 0; i < stFrameData.nImageCount; i++)
            {
               LOGD("Cam:%d MV3D_RGBD_FetchFrame success: FrameNum[%d] Width[%d] Height[%d] Len[%d] CoordinateType[%d]!", nIndexArray[cam_index], stFrameData.stImageData[i].nFrameNum,
                   stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nDataLen, stFrameData.stImageData[i].enCoordinateType);
            }
        }

        cam_index++;

        // ch:按任意键退出 | en:Press any key to exit
        if (_kbhit())
        {
            bExit_Main = TRUE;
        }
    }

    LOGD("All Cam stop work.");

    for(uint32_t i = 0; i < nLinkCount; i++)
    {
        ASSERT_OK(MV3D_RGBD_Stop(pHandle[i]));
        ASSERT_OK(MV3D_RGBD_CloseDevice(&pHandle[i]));
    }
    ASSERT_OK(MV3D_RGBD_Release());

    LOGD("Main done!");
    return  0;
}

