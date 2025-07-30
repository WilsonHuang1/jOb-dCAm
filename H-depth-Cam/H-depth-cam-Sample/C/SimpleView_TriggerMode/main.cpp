
#include "../common/common.hpp"

BOOL g_bThreadRunning = FALSE;
void*  __stdcall ProcessThread(void* pHandle)
{
    int nRet = MV3D_RGBD_OK;
    if (NULL != pHandle)
    {
        while (g_bThreadRunning)
        {
            ASSERT_OK(MV3D_RGBD_SoftTrigger(pHandle));
            LOGD("MV3D_RGBD_SoftTrigger success!");
            Sleep(3000);
        }
    }
    return NULL;
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
            LOG("Index[%d]. SerialNum[%s] IP[%s] Name[%s] Type[%d].\r\n",
                i, devs[i].chSerialNumber, devs[i].SpecialInfo.stNetInfo.chCurrentIp, devs[i].chModelName,
                (devs[i].nDevTypeInfo & 0xff000000) >> 24);
        }
        else if (DeviceType_USB == devs[i].enDeviceType || DeviceType_USB_Vir == devs[i].enDeviceType)
        {
            LOG("Index[%d]. SerialNum[%s] UsbProtocol[%d] Name[%s] Type[%d].\r\n",
                i, devs[i].chSerialNumber, devs[i].SpecialInfo.stUsbInfo.enUsbProtocol, devs[i].chModelName,
                (devs[i].nDevTypeInfo & 0xff000000) >> 24);
        }
    }

    // ch:打开设备 | en:Open device
    void* handle = NULL;
    unsigned int nIndex = 0;
    ASSERT_OK(MV3D_RGBD_OpenDevice(&handle, &devs[nIndex]));
    LOGD("OpenDevice success.");

    HANDLE hProcessThread = NULL;  
    // ch:创建软触发线程 | en: Create softTrigger thread
    hProcessThread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)ProcessThread, handle, 0, NULL);
    if (NULL == hProcessThread)
    {
        LOGD("Create proccess Thread failed ");
    }
    g_bThreadRunning = TRUE;

    // ch:开始取流 | en:Start work
    ASSERT_OK(MV3D_RGBD_Start(handle));
    LOGD("Start work success.");

    ASSERT_OK(MV3D_RGBD_SoftTrigger(handle));
    LOGD("MV3D_RGBD_SoftTrigger success!");

    MV3D_RGBD_FRAME_DATA stFrameData = {0};
    BOOL bExit_Main = FALSE;
    while (!bExit_Main)
    {
        // ch:获取图像数据 | en:Get image data
        int nRet = MV3D_RGBD_FetchFrame(handle, &stFrameData, 5000);
        if (MV3D_RGBD_OK == nRet)
        {
            if (!stFrameData.nValidInfo)
            {
                for(int i = 0; i < stFrameData.nImageCount; i++)
                {
                    LOGD("MV3D_RGBD_FetchFrame success: FrameNum[%d] Width[%d] Height[%d] Len[%d] CoordinateType[%d]!", stFrameData.stImageData[i].nFrameNum,
                        stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nDataLen, stFrameData.stImageData[i].enCoordinateType);
                }
            }
            else
            {
                LOGD("MV3D_RGBD_FetchFrame lost frame!");
            }
        }

        // ch:按任意键退出 | en:Press any key to exit
        if (_kbhit())
        {
            bExit_Main = TRUE;
        }
    }

    ASSERT_OK(MV3D_RGBD_Stop(handle));

    if (NULL != hProcessThread)
    {
        g_bThreadRunning = FALSE;
        WaitForSingleObject(hProcessThread,INFINITE);
        CloseHandle(hProcessThread);
        hProcessThread = NULL;
    }
    
    ASSERT_OK(MV3D_RGBD_CloseDevice(&handle));
    ASSERT_OK(MV3D_RGBD_Release());
    
    LOGD("Main done!");
    return  0;
}

