
#include "../common/common.hpp"
#include "../common/RenderImage.hpp"

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

    // ch:设置图像对齐彩色图坐标系 | en:Set image align rgb coordinate mode
    MV3D_RGBD_PARAM stParam;
    stParam.enParamType = ParamType_Int;
    stParam.ParamInfo.stIntParam.nCurValue = 1;
    ASSERT_OK(MV3D_RGBD_SetParam(handle, MV3D_RGBD_INT_IMAGEALIGN, &stParam));
    LOGD("Set image align success.");

    // ch:获取标定参数 | en:Get sensor calib param
    MV3D_RGBD_CAMERA_PARAM stCameraParam;
    ASSERT_OK(MV3D_RGBD_GetCameraParam(handle, &stCameraParam));
    LOGD("Get camera param success.");
    PrintCameraParam(&stCameraParam);

    // ch:开始取流 | en:Start work
    ASSERT_OK(MV3D_RGBD_Start(handle));
    LOGD("Start work success.");
    BOOL bExit_Main = FALSE;
    MV3D_RGBD_FRAME_DATA stFrameData = { 0 };
    RenderImgWnd depthViewer(768, 512, "depth");

    while (!bExit_Main && depthViewer)
    {
        // ch:获取图像数据 | en:Get image data
        int nRet = MV3D_RGBD_FetchFrame(handle, &stFrameData, 5000);
        if (MV3D_RGBD_OK == nRet)
        {
            if (!stFrameData.nValidInfo)
            {
                LOGD("MV3D_RGBD_FetchFrame success.");

                RIFrameInfo depth = { 0 };
                RIFrameInfo rgb = { 0 };
                RIFrameInfo rgbd = { 0 };
                parseFrame(&stFrameData, &depth, &rgb, &rgbd);
                
                depthViewer.RenderImage(depth);
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
    ASSERT_OK(MV3D_RGBD_CloseDevice(&handle));
    ASSERT_OK(MV3D_RGBD_Release());

    LOGD("Main done!");
    return  0;
}

