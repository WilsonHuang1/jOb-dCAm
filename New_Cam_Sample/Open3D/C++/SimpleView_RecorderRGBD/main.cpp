
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

    // ch:设置RGBD输出节点 | en : Set RGBD image output node
    MV3D_RGBD_PARAM stParam;
    stParam.enParamType = ParamType_Int;
    stParam.ParamInfo.stIntParam.nCurValue = 1;
    ASSERT_OK(MV3D_RGBD_SetParam(handle, MV3D_RGBD_INT_OUTPUT_RGBD, &stParam));
    LOGD("Set RGBDImage output success.");

    MV3D_RGBD_CAMERA_PARAM  stCameraParam;
    ASSERT_OK(MV3D_RGBD_GetCameraParam(handle, &stCameraParam));

    // ch:开始工作流程 | en:Start work
    ASSERT_OK(MV3D_RGBD_Start(handle));
    LOGD("Start work success.");
    BOOL bExit_Main = FALSE;
    MV3D_RGBD_FRAME_DATA stFrameData = { 0 };

    open3d::visualization::Visualizer vis;
    vis.CreateVisualizerWindow("RGBD", 600, 400);
    open3d::visualization::Visualizer vis1;
    vis1.CreateVisualizerWindow("RGBD-To-PointCloud", 600, 400);

    open3d::geometry::Image depthImage, rgbImage;
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
                    if (ImageType_Rgbd == stFrameData.stImageData[i].enImageType)
                    {
                        LOGD("Get RGBDImage succeed: FrameNum[%d] Width[%d] Height[%d] Len[%d] CoordinateType[%d]!", stFrameData.stImageData[i].nFrameNum,
                            stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nDataLen, stFrameData.stImageData[i].enCoordinateType);

                        depthImage.Prepare(stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nHeight, 1, 2);
                        uint8_t* pData = (uint8_t*)stFrameData.stImageData[i].pData;
                        for (int j = 0; j < stFrameData.stImageData[i].nWidth * stFrameData.stImageData[i].nHeight; j++)
                        {
                            depthImage.data_[2 * j] = (*(pData + 5 * j + 3));
                            depthImage.data_[2 * j + 1] = (*(pData + 5 * j + 4));
                        }

                        rgbImage.Prepare(stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nHeight,3,1);
                        for (int j = 0; j < stFrameData.stImageData[i].nWidth * stFrameData.stImageData[i].nHeight; j++)
                        {
                            rgbImage.data_[j * 3] = (*(pData + 5 * j)); //R
                            rgbImage.data_[j * 3 + 1] = (*(pData + 5 * j + 1)); //G
                            rgbImage.data_[j * 3 + 2] = (*(pData + 5 * j + 2)); //B
                        }
                        
                        auto RGBDImageShow = open3d::geometry::RGBDImage::CreateFromColorAndDepth(rgbImage, depthImage, 1000, 3, false);

                        open3d::camera::PinholeCameraIntrinsic inter;
                        inter.SetIntrinsics(stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nHeight, stCameraParam.stRgbCalibInfo.stIntrinsic.fData[0], stCameraParam.stRgbCalibInfo.stIntrinsic.fData[4], stCameraParam.stRgbCalibInfo.stIntrinsic.fData[2], stCameraParam.stRgbCalibInfo.stIntrinsic.fData[5]);
                        auto PointCloudShow = open3d::geometry::PointCloud::CreateFromRGBDImage(*RGBDImageShow.get(), inter);

                        vis.AddGeometry(RGBDImageShow);
                        vis.PollEvents();
                        vis.UpdateRender();

                        vis1.AddGeometry(PointCloudShow);
                        vis1.PollEvents();
                        vis1.UpdateRender();
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

