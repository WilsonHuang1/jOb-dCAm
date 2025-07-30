
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

    // ch:设置点云输出节点 | en : Set point cloud output node
    MV3D_RGBD_PARAM stParam;
    stParam.enParamType = ParamType_Enum;
    stParam.ParamInfo.stEnumParam.nCurValue = PointCloudType_Common;
    ASSERT_OK(MV3D_RGBD_SetParam(handle, MV3D_RGBD_ENUM_POINT_CLOUD_OUTPUT, &stParam));
    LOGD("Set point cloud output success.");

    // ch:开始工作流程 | en:Start work
    ASSERT_OK(MV3D_RGBD_Start(handle));
    LOGD("Start work success.");
    BOOL bExit_Main = FALSE;
    MV3D_RGBD_FRAME_DATA stFrameData = { 0 };

    open3d::visualization::Visualizer vis;
    vis.CreateVisualizerWindow("PointCloud", 900, 600);
    auto PointCloudShow = std::shared_ptr<open3d::geometry::PointCloud>(new open3d::geometry::PointCloud());
    
    while (!bExit_Main)
    {
        // ch:获取图像数据 | en:Get image data
        int nRet = MV3D_RGBD_FetchFrame(handle, &stFrameData, 5000);
        if (MV3D_RGBD_OK == nRet)
        {
            if (!stFrameData.nValidInfo)
            {
                vis.ClearGeometries();
                LOGD("MV3D_RGBD_FetchFrame success.");
                for (int i = 0; i < stFrameData.nImageCount; i++)
                {
                    if (ImageType_PointCloud == stFrameData.stImageData[i].enImageType)
                    {
                        LOGD("Get PointCloud succeed: FrameNum[%d] Width[%d] Height[%d] Len[%d] CoordinateType[%d]!", stFrameData.stImageData[i].nFrameNum,
                            stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nDataLen, stFrameData.stImageData[i].enCoordinateType);
                        float* pData = (float*)stFrameData.stImageData[i].pData;
                        
                        if(PointCloudShow->points_.size() != stFrameData.stImageData[i].nWidth * stFrameData.stImageData[i].nHeight)
                        {
							PointCloudShow->points_.resize(stFrameData.stImageData[i].nWidth * stFrameData.stImageData[i].nHeight);
                        }
                        
                        for (int j = 0; j < stFrameData.stImageData[i].nWidth * stFrameData.stImageData[i].nHeight; j++)
                        {
                            Eigen::Vector3d point;
                            point(0) = *(pData + j * 3);
                            point(1) = *(pData + j * 3 + 1);
                            point(2) = *(pData + j * 3 + 2);
                            PointCloudShow->points_[j] = point;
                        }
                    }

                    if (ImageType_RGB8_Planar == stFrameData.stImageData[i].enImageType)
                    {
                        LOGD("Get rgb succeed: FrameNum[%d] Width[%d] Height[%d] Len[%d] CoordinateType[%d]!", stFrameData.stImageData[i].nFrameNum,
                            stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nDataLen, stFrameData.stImageData[i].enCoordinateType);
                        uint8_t* pData = (uint8_t*)stFrameData.stImageData[i].pData;
                        
						if(PointCloudShow->colors_.size() != stFrameData.stImageData[i].nWidth * stFrameData.stImageData[i].nHeight)
                        {
							PointCloudShow->colors_.resize(stFrameData.stImageData[i].nWidth * stFrameData.stImageData[i].nHeight);
                        }

                        for (int j = 0; j < stFrameData.stImageData[i].nWidth * stFrameData.stImageData[i].nHeight; j++)
                        {
                            Eigen::Vector3d color;
                            color(0) = double(*(pData + j))/ 255;
                            color(1) = double(*(pData + j + stFrameData.stImageData[i].nWidth * stFrameData.stImageData[i].nHeight))/255;
                            color(2) = double(*(pData + j + stFrameData.stImageData[i].nWidth * stFrameData.stImageData[i].nHeight * 2))/255;
                            PointCloudShow->colors_[j] = color;
                        }
                    }
                }
                vis.AddGeometry(PointCloudShow);
                vis.PollEvents();
                vis.UpdateRender();
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
    open3d::io::WritePointCloud("PointCloud.ply", *PointCloudShow);
    vis.ClearGeometries();
    vis.Close();
   
    ASSERT_OK(MV3D_RGBD_Stop(handle));
    ASSERT_OK(MV3D_RGBD_CloseDevice(&handle));
    ASSERT_OK(MV3D_RGBD_Release());

    LOGD("Main done!");
    return  0;
}

