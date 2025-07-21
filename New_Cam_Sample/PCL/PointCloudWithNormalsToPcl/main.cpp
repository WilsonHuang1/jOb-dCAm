
#include "../common/common.hpp"

void convertToPCL(const MV3D_RGBD_IMAGE_DATA& stPointCloud, pcl::PointCloud<pcl::PointNormal>& pclPointCloud)
{
    pcl::PCLPointCloud2 pclCloud2;
    pclCloud2.height = stPointCloud.nHeight;
    pclCloud2.width = stPointCloud.nWidth;
    pclCloud2.point_step = sizeof(POINT_XYZNORMALS);
    pclCloud2.row_step = sizeof(POINT_XYZNORMALS) * stPointCloud.nWidth;

    pclCloud2.fields.reserve(7);
    pclCloud2.fields.push_back(createPointField("x", offsetof(POINT_XYZNORMALS, pPoint.fX), pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));
    pclCloud2.fields.push_back(createPointField("y", offsetof(POINT_XYZNORMALS, pPoint.fY), pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));
	pclCloud2.fields.push_back(createPointField("z", offsetof(POINT_XYZNORMALS, pPoint.fZ), pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));
	pclCloud2.fields.push_back(createPointField("normal_x", offsetof(POINT_XYZNORMALS, pNormals.fNormalX), pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));
	pclCloud2.fields.push_back(createPointField("normal_y", offsetof(POINT_XYZNORMALS, pNormals.fNormalY), pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));
	pclCloud2.fields.push_back(createPointField("normal_z", offsetof(POINT_XYZNORMALS, pNormals.fNormalZ), pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));
	pclCloud2.fields.push_back(createPointField("curvature", offsetof(POINT_XYZNORMALS, pNormals.fCurvature), pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));

    pclCloud2.data.resize(pclCloud2.row_step * stPointCloud.nHeight);
	uint8_t* pData = pclCloud2.data.data();
	uint8_t* pPointCloudData = stPointCloud.pData;
	for (int i = 0; i < stPointCloud.nHeight * stPointCloud.nWidth; i++)
	{
		memcpy(pData, pPointCloudData, sizeof(MV3D_RGBD_POINT_XYZ_NORMALS));
		pData += pclCloud2.point_step;
		pPointCloudData += sizeof(MV3D_RGBD_POINT_XYZ_NORMALS);
	}
	pData = NULL;
	pPointCloudData = NULL;
    pcl::fromPCLPointCloud2(pclCloud2, pclPointCloud);
    return;
}

void showPointCloud(const pcl::PointCloud<pcl::PointNormal>& pclPointCloud)
{
    vtkOutputWindow::SetGlobalWarningDisplay(0);
    if (pclPointCloud.empty())
    {
        return;
    }

    pcl::visualization::PCLVisualizer pclCloudViewer("Point Cloud Viewer");
	pclCloudViewer.setShowFPS(false);
	pclCloudViewer.setBackgroundColor(0, 0, 0);
	pclCloudViewer.addPointCloudNormals<pcl::PointNormal>(pclPointCloud.makeShared(), 10, 10);
	pclCloudViewer.addCoordinateSystem(0.01);
	pclCloudViewer.addText("Point cloud size: " + std::to_string(pclPointCloud.size()), 0, 25, 20, 1, 1, 1, "cloudSize");
	pclCloudViewer.addText("Press r/R to reset camera view point to center. Press q to quit.", 0, 0, 16, 1, 1, 1, "help");
	pclCloudViewer.initCameraParameters();
    while (!pclCloudViewer.wasStopped())
	{
		pclCloudViewer.spinOnce(20);
        Sleep(100);
        if (_kbhit() == 'q')
        {
            break;
        }
    }
}

int main(int argc,char** argv)
{
    LOGD("Initialize");
    ASSERT_OK( MV3D_RGBD_Initialize() );

    MV3D_RGBD_VERSION_INFO stVersion;
    ASSERT_OK( MV3D_RGBD_GetSDKVersion(&stVersion) );
    LOGD("dll version: %d.%d.%d", stVersion.nMajor, stVersion.nMinor, stVersion.nRevision);

    unsigned int nDevNum = 0;
    ASSERT_OK(MV3D_RGBD_GetDeviceNumber(DeviceType_Ethernet | DeviceType_USB | DeviceType_Ethernet_Vir | DeviceType_USB_Vir, &nDevNum));
    LOGD("MV3D_RGBD_GetDeviceNumber success! nDevNum:%d.", nDevNum);
    ASSERT(nDevNum);

    // ch:查找设备 | en:Enumerate devices
    std::vector<MV3D_RGBD_DEVICE_INFO> devs(nDevNum);
    ASSERT_OK(MV3D_RGBD_GetDeviceList(DeviceType_Ethernet | DeviceType_USB | DeviceType_Ethernet_Vir | DeviceType_USB_Vir, &devs[0], nDevNum, &nDevNum));

    for (unsigned int i = 0; i < nDevNum; i++)
    {  
        if (DeviceType_Ethernet == devs[i].enDeviceType || DeviceType_Ethernet_Vir == devs[i].enDeviceType)
        {
            LOG("Index[%d]. SerialNum[%s] IP[%s] Name[%s].\r\n", i, devs[i].chSerialNumber, devs[i].SpecialInfo.stNetInfo.chCurrentIp, devs[i].chModelName);
        }
        else if (DeviceType_USB == devs[i].enDeviceType || DeviceType_USB_Vir == devs[i].enDeviceType)
        {
            LOG("Index[%d]. SerialNum[%s] UsbProtocol[%d] Name[%s].\r\n", i, devs[i].chSerialNumber, devs[i].SpecialInfo.stUsbInfo.enUsbProtocol, devs[i].chModelName);
        }
    }

    // ch:打开设备 | en:Open device
    void* handle = NULL;
    unsigned int nIndex = 0;
    ASSERT_OK(MV3D_RGBD_OpenDevice(&handle, &devs[nIndex]));
    LOGD("OpenDevice success.");

    // ch:设置点云输出节点 | en:Set point cloud output node
    MV3D_RGBD_PARAM stParam;
    stParam.enParamType = ParamType_Enum;
    stParam.ParamInfo.stEnumParam.nCurValue = PointCloudType_Normals;
    ASSERT_OK(MV3D_RGBD_SetParam(handle, MV3D_RGBD_ENUM_POINT_CLOUD_OUTPUT, &stParam));
    LOGD("Set point cloud output success.");

    // ch:开始工作流程 | en:Start work
    ASSERT_OK(MV3D_RGBD_Start(handle));
    LOGD("Start work success.");

    // ch:获取图像数据 | en:Get image data
    MV3D_RGBD_FRAME_DATA stFrameData = { 0 };
    int nRet = MV3D_RGBD_FetchFrame(handle, &stFrameData, 5000);
    if (MV3D_RGBD_OK == nRet)
    {
        if (0 == stFrameData.nValidInfo)
        {
            MV3D_RGBD_IMAGE_DATA* pstPointCloudImage = NULL;
            for (int i = 0; i < stFrameData.nImageCount; i++)
            {
                LOGD("MV3D_RGBD_FetchFrame Image %d Success: FrameNum[%d] ImageType[%ld] Width[%d] Hidth[%d] Len[%d] CoordinataType[%d]!", i, stFrameData.stImageData[i].nFrameNum,
                    stFrameData.stImageData[i].enImageType, stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nDataLen, stFrameData.stImageData[i].enCoordinateType);

                if (ImageType_PointCloudWithNormals == stFrameData.stImageData[i].enImageType)
                {
                    pstPointCloudImage = &stFrameData.stImageData[i];
                }
            }
            if (NULL != pstPointCloudImage)
            {
                // ch:转换PCL格式法向量点云数据 | en:Convert PCL format point cloud with normal vectors data
                pcl::PointCloud<pcl::PointNormal> pclPointCloud(pstPointCloudImage->nWidth, pstPointCloudImage->nHeight);
                convertToPCL(*pstPointCloudImage, pclPointCloud);
                // ch:保存PCL法向量点云文件 | en:Save PCL format point cloud with normal vectors file
                char chFileName[256] = { 0 };
                sprintf(chFileName, "[%d]_PointCloudWithNormals.ply", pstPointCloudImage->nFrameNum);
                pcl::PLYWriter pclWriter;
                pclWriter.write(chFileName, pclPointCloud, false);
                LOGD("Save PCL format point cloud success!");
                // ch:渲染法向量点云 | en:Display point cloud with normal vectors
                showPointCloud(pclPointCloud);
            }
			else
			{
				LOGD("Get point cloud image failed...");
			}
        }
        else
        {
            LOGD("MV3D_RGBD_FetchFrame lost frame...");
        }
    }
    else
    {
        LOGD("MV3D_RGBD_FetchFrame failed...");
    }

    ASSERT_OK(MV3D_RGBD_Stop(handle));
    ASSERT_OK(MV3D_RGBD_CloseDevice(&handle));
    ASSERT_OK(MV3D_RGBD_Release());

    LOGD("Main done!");

    return  0;
}

