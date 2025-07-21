
#include "../common/common.hpp"
#include <Open3D/Open3D.h>

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

    // ch:查找设备 | en:Enumerate devices
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

    // ch:开始工作流程 | en:Start work
    ASSERT_OK(MV3D_RGBD_Start(handle));
    LOGD("Start work success.");
    BOOL bExit_Main = FALSE;
    MV3D_RGBD_FRAME_DATA stFrameData = { 0 };

    open3d::visualization::Visualizer vis;
    vis.CreateVisualizerWindow("RGB", 600, 400);
    open3d::visualization::Visualizer vis1;
    vis1.CreateVisualizerWindow("Depth", 600, 400);
    auto depthImageShow = std::shared_ptr<open3d::geometry::Image>(new open3d::geometry::Image());
    auto rgbImageShow = std::shared_ptr<open3d::geometry::Image>(new open3d::geometry::Image());
    while (!bExit_Main)
    {
        // ch:获取图像数据 | en:Get image data
        int nRet = MV3D_RGBD_FetchFrame(handle, &stFrameData, 5000);
        if (MV3D_RGBD_OK == nRet)
        {
            if (!stFrameData.nValidInfo)
            {
                vis.ClearGeometries();
                vis1.ClearGeometries();
                LOGD("MV3D_RGBD_FetchFrame success.");
                for (int i = 0; i < stFrameData.nImageCount; i++)
                {
                    if (ImageType_Depth == stFrameData.stImageData[i].enImageType)
                    {
                        LOGD("Get DepthImage succeed: FrameNum[%d] Width[%d] Height[%d] Len[%d] CoordinateType[%d]!", stFrameData.stImageData[i].nFrameNum,
                            stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nDataLen, stFrameData.stImageData[i].enCoordinateType);

                        depthImageShow->Prepare(stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nHeight, 1, 2);
                        uint8_t* pData = (uint8_t*)stFrameData.stImageData[i].pData;
                        for (int j = 0; j < stFrameData.stImageData[i].nDataLen; j++)
                        {
                            depthImageShow->data_[j] = (*(pData + j));
                        }
                        vis1.AddGeometry(depthImageShow);
                        vis1.PollEvents();
                        vis1.UpdateRender();

                    }

                    if (ImageType_RGB8_Planar == stFrameData.stImageData[i].enImageType)
                    {
                        LOGD("Get rgb succeed: FrameNum[%d] Width[%d] Height[%d] Len[%d] CoordinateType[%d]!", stFrameData.stImageData[i].nFrameNum,
                            stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nDataLen, stFrameData.stImageData[i].enCoordinateType);
                        rgbImageShow->Prepare(stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nHeight,3,1);
                        uint8_t* pData = (uint8_t*)stFrameData.stImageData[i].pData;
                        for (int j = 0; j < stFrameData.stImageData[i].nWidth * stFrameData.stImageData[i].nHeight; j++)
                        {
                            rgbImageShow->data_[j * 3] = (*(pData + j)); //R
                            rgbImageShow->data_[j * 3 + 1] = (*(stFrameData.stImageData[i].nHeight * stFrameData.stImageData[i].nWidth + pData + j)); //G
                            rgbImageShow->data_[j * 3 + 2] = (*(stFrameData.stImageData[i].nHeight * stFrameData.stImageData[i].nWidth * 2 + pData + j)); //B
                        }
                        vis.AddGeometry(rgbImageShow);
                        vis.PollEvents();
                        vis.UpdateRender();
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

    vis.ClearGeometries();
    vis1.ClearGeometries();
    vis.Close();
    vis1.Close();
   
    ASSERT_OK(MV3D_RGBD_Stop(handle));
    ASSERT_OK(MV3D_RGBD_CloseDevice(&handle));
    ASSERT_OK(MV3D_RGBD_Release());

    LOGD("Main done!");
    return  0;
}

