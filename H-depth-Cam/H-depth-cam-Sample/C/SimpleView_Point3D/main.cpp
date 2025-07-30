
#include "../common/common.hpp"
#include "../common/RenderImage.hpp"

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
    unsigned int nIndex  = 0;
    ASSERT_OK(MV3D_RGBD_OpenDevice(&handle, &devs[nIndex]));
    LOGD("OpenDevice success.");

    // ch:开始取流 | en:Start work
    ASSERT_OK(MV3D_RGBD_Start(handle));
    LOGD("Start work success.");

    BOOL bExit_Main = FALSE;
    RenderImgWnd pointCloudViewer(768, 512, "pointcloud");
    pointCloudViewer.Init3DRender();
    MV3D_RGBD_FRAME_DATA stFrameData = {0};
   
    while (!bExit_Main && pointCloudViewer)
    {
        // ch:获取图像数据 | en:Get image data
        int nRet = MV3D_RGBD_FetchFrame(handle, &stFrameData, 5000);
        if (MV3D_RGBD_OK == nRet)
        {
            for(int i = 0; i < stFrameData.nImageCount; i++)
            {
                if(ImageType_Depth == stFrameData.stImageData[i].enImageType)
                {
                    MV3D_RGBD_IMAGE_DATA stPointCloudImage = {};
                    // ch:深度图转点云 | en:Depth map to point cloud
                    nRet = MV3D_RGBD_MapDepthToPointCloud(handle, &stFrameData.stImageData[i], &stPointCloudImage);
                    if (MV3D_RGBD_OK != nRet)
                    {
                        break;
                    }
                    LOGD("MapDepthToPointCloud Run Succeed: FrameNum[%d] Width[%d] Height[%d] Len[%d] CoordinateType[%d]!", stPointCloudImage.nFrameNum,
                        stPointCloudImage.nWidth, stPointCloudImage.nHeight, stPointCloudImage.nDataLen, stPointCloudImage.enCoordinateType);
                    RIFrameInfo pointCloud    = { 0 };
                    pointCloud.enPixelType    = (RIPixelType)stPointCloudImage.enImageType;
                    printf("pixel type: %d\r\n", pointCloud.enPixelType);
                    pointCloud.nFrameNum      = stPointCloudImage.nFrameNum;
                    pointCloud.nHeight        = stPointCloudImage.nHeight;
                    pointCloud.nWidth         = stPointCloudImage.nWidth;
                    pointCloud.nFrameLength   = stPointCloudImage.nDataLen;
                    pointCloud.pData          = stPointCloudImage.pData;

                    pointCloudViewer.RenderPointCloud(pointCloud);
                }
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

