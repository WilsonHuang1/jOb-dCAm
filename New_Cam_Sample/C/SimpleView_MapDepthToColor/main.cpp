
#include "../common/common.hpp"
#include "../common/RenderImage.hpp"

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

    // ch:配置深度图转伪彩参数 | en:Config parameters of depth convert to color 
    MV3D_RGBD_CONVERT_COLOR_PAPRAM stConvertParam;
    memset(&stConvertParam, 0, sizeof(MV3D_RGBD_CONVERT_COLOR_PAPRAM));
    stConvertParam.enConvertColorMapMode = ConvertColorMapMode_Rainbow;

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
                for (int i = 0; i < stFrameData.nImageCount; i++)
                {
                    if (ImageType_Depth == stFrameData.stImageData[i].enImageType)
                    {
                        MV3D_RGBD_IMAGE_DATA stConvertColorImage;
                        // ch:深度图转伪彩 | en:Convert depth to color image
                        nRet = MV3D_RGBD_MapDepthToColor(&stFrameData.stImageData[i], &stConvertParam, &stConvertColorImage);
                        if (MV3D_RGBD_OK == nRet)
                        {
                            LOGD("Map depth image %d to color image success! Image Width[%d] Height[%d]", stConvertColorImage.nFrameNum, stConvertColorImage.nWidth, stConvertColorImage.nHeight);
                            RIFrameInfo rgb = { 0 };
                            rgb.enPixelType = RIPixelType_RGB8_Planar;
                            rgb.nFrameLength = stConvertColorImage.nDataLen;
                            rgb.nFrameNum = stConvertColorImage.nFrameNum;
                            rgb.nHeight = stConvertColorImage.nHeight;
                            rgb.nWidth = stConvertColorImage.nWidth;
                            rgb.pData = stConvertColorImage.pData;
                            depthViewer.RenderImage(rgb);
                        }
                        else
                        {
                            LOGD("MV3D_RGBD_MapDepthToColor failed...sts[%#x]", nRet);
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
