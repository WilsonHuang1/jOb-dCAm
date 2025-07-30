
#include "../common/common.hpp"

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

    // ch:关闭图像对齐坐标系 | en:Close image align depth coordinate mode
    MV3D_RGBD_PARAM stParam;
    stParam.enParamType = ParamType_Int;
    stParam.ParamInfo.stIntParam.nCurValue = 0;
    ASSERT_OK(MV3D_RGBD_SetParam(handle, MV3D_RGBD_INT_IMAGEALIGN, &stParam));
    LOGD("Close image align success.");

    // ch:获取深度量纲 | en:Get depth z-unit
    ASSERT_OK(MV3D_RGBD_GetParam(handle, MV3D_RGBD_FLOAT_Z_UNIT, &stParam));
    LOGD("Get z-unit success.");
    float fZunit = stParam.ParamInfo.stFloatParam.fCurValue;

    // ch:开始取流 | en:Start work
    ASSERT_OK(MV3D_RGBD_Start(handle));
    LOGD("Start work success.");
    BOOL bExit_Main = FALSE;
    MV3D_RGBD_FRAME_DATA stFrameData = { 0 };

    while (!bExit_Main)
    {
        // ch:获取图像数据 | en:Get image data
        int nRet = MV3D_RGBD_FetchFrame(handle, &stFrameData, 5000);
        if (MV3D_RGBD_OK == nRet)
        {
            if (!stFrameData.nValidInfo)
            {
                LOGD("MV3D_RGBD_FetchFrame success.");
                for (int i = 0; i < stFrameData.nImageCount; i++)
                {
                    if (ImageType_Depth == stFrameData.stImageData[i].enImageType)
                    {
                        LOGD("depth raw image: FrameNum[%d] Width[%d] Height[%d] Len[%d] CoordinateType[%d]!", stFrameData.stImageData[i].nFrameNum,
                            stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nDataLen, stFrameData.stImageData[i].enCoordinateType);
                        // ch:获取标定信息 | en:Get sensor calib info
                        MV3D_RGBD_CAMERA_PARAM stCameraParam;
                        nRet = MV3D_RGBD_GetCameraParam(handle, &stCameraParam);
                        if (MV3D_RGBD_OK != nRet)
                        {
                            LOGD("Get camera param failed...sts[%#x]", nRet);
                            break;
                        }
                        // ch:深度图对齐彩色图坐标系 | en:Convert depth map to rgb coordinate
                        MV3D_RGBD_IMAGE_DATA stDepthConvImg;
                        nRet = MV3D_RGBD_ImageCoordinateTrans(&stFrameData.stImageData[i], fZunit, &stDepthConvImg, &stCameraParam);
                        if (MV3D_RGBD_OK == nRet)
                        {
                            LOGD("depth convert image: FrameNum[%d] Width[%d] Height[%d] Len[%d] CoordinateType[%d]!", stDepthConvImg.nFrameNum,
                                stDepthConvImg.nWidth, stDepthConvImg.nHeight, stDepthConvImg.nDataLen, stFrameData.stImageData[i].enCoordinateType);
                        }
                        else
                        {
                            LOGD("MV3D_RGBD_ImageCoordinateTrans failed...sts[%#x]", nRet);
                            break;
                        }
                    }
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
    ASSERT_OK(MV3D_RGBD_CloseDevice(&handle));
    ASSERT_OK(MV3D_RGBD_Release());

    LOGD("Main done!");
    return  0;
}

