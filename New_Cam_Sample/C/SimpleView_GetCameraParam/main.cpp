#include "../common/common.hpp"

void PrintCameraParam(MV3D_RGBD_CAMERA_PARAM* pstCameraParam)
{
    if (NULL == pstCameraParam)
    {
        return;
    }
    LOGD("-------- Depth Calib Info --------");
    LOGD("Depth Width: %d", pstCameraParam->stDepthCalibInfo.nWidth);
    LOGD("Depth Height: %d", pstCameraParam->stDepthCalibInfo.nHeight);
    LOGD("--------");
    for (int i = 0; i < 9; i++)
    {
        LOGD("Depth Intrinsic[%d]: %.10f", i, pstCameraParam->stDepthCalibInfo.stIntrinsic.fData[i]);
    }
    LOGD("--------");
    for (int i = 0; i < 12; i++)
    {
        LOGD("Depth Distortion[%d]: %.10f", i, pstCameraParam->stDepthCalibInfo.stDistortion.fData[i]);
    }
    LOGD("-------- Rgb Calib Info --------");
    LOGD("Rgb Width: %d", pstCameraParam->stRgbCalibInfo.nWidth);
    LOGD("Rgb Height: %d", pstCameraParam->stRgbCalibInfo.nHeight);
    LOGD("--------");
    for (int i = 0; i < 9; i++)
    {
        LOGD("Rgb Intrinsic[%d]: %.10f", i, pstCameraParam->stRgbCalibInfo.stIntrinsic.fData[i]);
    }
    LOGD("--------");
    for (int i = 0; i < 12; i++)
    {
        LOGD("Rgb Distortion[%d]: %.10f", i, pstCameraParam->stRgbCalibInfo.stDistortion.fData[i]);
    }
    LOGD("-------- Depth to Rgb Extrinsic --------");
    for (int i = 0; i < 16; i++)
    {
        LOGD("Depth to Rgb Extrinsic[%d]: %.10f", i, pstCameraParam->stDepth2RgbExtrinsic.fData[i]);
    }
    LOGD("--------");
    return;
}

int main(int argc, char** argv)
{
    MV3D_RGBD_VERSION_INFO stVersion;
    ASSERT_OK(MV3D_RGBD_GetSDKVersion(&stVersion));
    LOGD("dll version: %d.%d.%d", stVersion.nMajor, stVersion.nMinor, stVersion.nRevision);

    ASSERT_OK(MV3D_RGBD_Initialize());

    unsigned int nDevNum = 0;
    ASSERT_OK(MV3D_RGBD_GetDeviceNumber((DeviceType_Ethernet | DeviceType_USB | DeviceType_Ethernet_Vir | DeviceType_USB_Vir), &nDevNum));
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

    // ch:获取标定参数 | en:Get sensor calib param
    MV3D_RGBD_CAMERA_PARAM stCameraParam;
    ASSERT_OK(MV3D_RGBD_GetCameraParam(handle, &stCameraParam));
    LOGD("Get camera param success.");
    PrintCameraParam(&stCameraParam);
    LOGD("Press any key to quit.");
    while (1)
    {
        // ch:按任意键退出 | en:Press any key to exit
        if (_kbhit())
        {
            break;
        }
        Sleep(100);
    }
    ASSERT_OK(MV3D_RGBD_CloseDevice(&handle));
    ASSERT_OK(MV3D_RGBD_Release());

    LOGD("Main done!");
    return  0;
}

