
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

    // ch:设置点云输出节点 | en:Set point cloud output node
    MV3D_RGBD_PARAM stParam;
    stParam.enParamType = ParamType_Enum;
    stParam.ParamInfo.stEnumParam.nCurValue = PointCloudType_Common;
    ASSERT_OK(MV3D_RGBD_SetParam(handle, MV3D_RGBD_ENUM_POINT_CLOUD_OUTPUT, &stParam));
    LOGD("Set point cloud output success.");

    // ch:获取标定参数 | en:Get calib info param
    MV3D_RGBD_CALIB_INFO stCalibInfo;
    ASSERT_OK(MV3D_RGBD_GetCalibInfo(handle, CoordinateType_RGB, &stCalibInfo));
    LOGD("Get calib info success.");

    // ch:开始取流 | en:Start work
    ASSERT_OK(MV3D_RGBD_Start(handle));
    LOGD("Start work success.");

    BOOL bExit_Main = FALSE;
    RenderImgWnd pointCloudViewer(768, 512, "pointcloud");
    pointCloudViewer.Init3DRender();
    MV3D_RGBD_FRAME_DATA stFrameData = { 0 };
    glfw_state stAppState;

    while (!bExit_Main && pointCloudViewer)
    {
        // ch:获取图像数据 | en:Get image data
        int nRet = MV3D_RGBD_FetchFrame(handle, &stFrameData, 5000);
        if (MV3D_RGBD_OK == nRet)
        {
            BOOL bUvMap = FALSE;
            BOOL bTexture = FALSE;
            RIFrameInfo FrameInfo = { 0 };
            RIFrameTexInfo TexInfo = { 0 };
            for (int i = 0; i < stFrameData.nImageCount; i++)
            {
                // ch:解析点云数据 | en: Parse point cloud data
                if (ImageType_PointCloud == stFrameData.stImageData[i].enImageType)
                {
                    LOGD("Get point cloud succeed: FrameNum[%d] Width[%d] Height[%d] Len[%d] CoordinateType[%d]!", stFrameData.stImageData[i].nFrameNum,
                        stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nDataLen, stFrameData.stImageData[i].enCoordinateType);
                    FrameInfo.enPixelType = (RIPixelType)stFrameData.stImageData[i].enImageType;
                    FrameInfo.nFrameNum = stFrameData.stImageData[i].nFrameNum;
                    FrameInfo.nHeight = stFrameData.stImageData[i].nHeight;
                    FrameInfo.nWidth = stFrameData.stImageData[i].nWidth;
                    FrameInfo.nFrameLength = stFrameData.stImageData[i].nDataLen;
                    FrameInfo.pData = stFrameData.stImageData[i].pData;
                    // ch:获取uv坐标 | en: Get uv coordinate
                    MV3D_RGBD_UV_DATA stUvMap;
                    nRet = MV3D_RGBD_MapPointCloudToUV(&stFrameData.stImageData[i], &stCalibInfo, &stUvMap);
                    if (MV3D_RGBD_OK == nRet)
                    {
                        TexInfo.pTexData = (float*)stUvMap.pData;
                        bUvMap = TRUE;
                    }
                    else
                    {
                        LOGD("Get uv map failed...");
                        break;
                    }
                }
                // ch:解析彩色纹理数据 | en: Parse color texture data
                if (ImageType_RGB8_Planar == stFrameData.stImageData[i].enImageType)
                {
                    LOGD("Get texture color image succeed: FrameNum[%d] Width[%d] Height[%d] Len[%d] CoordinateType[%d]!", stFrameData.stImageData[i].nFrameNum,
                        stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nDataLen, stFrameData.stImageData[i].enCoordinateType);
                    TexInfo.pRgbData = stFrameData.stImageData[i].pData;
                    TexInfo.nWidth = stFrameData.stImageData[i].nWidth;
                    TexInfo.nHeight = stFrameData.stImageData[i].nHeight;
                    TexInfo.enPixelType = (RIPixelType)ImageType_RGB8_Planar;
                    TexInfo.nFrameNum = stFrameData.stImageData[i].nFrameNum;
                    bTexture = TRUE;
                }
            }
            // ch:渲染纹理点云 | en: Display textured point cloud
            if (bUvMap && bTexture)
            {
                stAppState.tex.upload(&TexInfo);
                pointCloudViewer.RenderUVPointCloud(FrameInfo, TexInfo, stAppState);
            }
            else
            {
                LOGD("Lost uv map or textured image...");
            }
        }
        // ch:按任意键退出 | en:Press any key to exit
        if (_kbhit())
        {
            bExit_Main = TRUE;
        }
    };

    ASSERT_OK(MV3D_RGBD_Stop(handle));
    ASSERT_OK(MV3D_RGBD_CloseDevice(&handle));
    ASSERT_OK(MV3D_RGBD_Release());

    LOGD("Main done!");
    return  0;
}

