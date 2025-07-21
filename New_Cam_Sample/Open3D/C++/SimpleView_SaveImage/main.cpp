
#include "../common/common.hpp"
#include <Open3D/Open3D.h>

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

    // ch:查找设备 | en:Enumerate devices
    std::vector<MV3D_RGBD_DEVICE_INFO> devs(nDevNum);
    ASSERT_OK(MV3D_RGBD_GetDeviceList(DeviceType_Ethernet | DeviceType_USB | DeviceType_Ethernet_Vir | DeviceType_USB_Vir, &devs[0], nDevNum, &nDevNum));
    for (unsigned int i = 0; i < nDevNum; i++)
    {  
        if (DeviceType_Ethernet == devs[i].enDeviceType || DeviceType_Ethernet_Vir == devs[i].enDeviceType)
        {
            LOG("Index[%d]. SerialNum[%s] IP[%s] name[%s] 产品线[%d].\r\n",
                i, devs[i].chSerialNumber, devs[i].SpecialInfo.stNetInfo.chCurrentIp, devs[i].chModelName,
                (devs[i].nDevTypeInfo & 0xff000000) >> 24);
        }
        else if (DeviceType_USB == devs[i].enDeviceType || DeviceType_USB_Vir == devs[i].enDeviceType)
        {
            LOG("Index[%d]. SerialNum[%s] UsbProtocol[%d] name[%s] 产品线[%d].\r\n",
                i, devs[i].chSerialNumber, devs[i].SpecialInfo.stUsbInfo.enUsbProtocol, devs[i].chModelName,
                (devs[i].nDevTypeInfo & 0xff000000) >> 24);
        }
    }

    // ch:打开设备 | en:Open device
    void* handle = NULL;
    unsigned int nIndex  = 0;
    ASSERT_OK(MV3D_RGBD_OpenDevice(&handle, &devs[nIndex]));
    LOGD("OpenDevice success.");

    // ch:开始工作流程 | en:Start work
    ASSERT_OK(MV3D_RGBD_Start(handle));
    LOGD("Start work success.");

    BOOL bExit_Main = FALSE;
    MV3D_RGBD_FRAME_DATA stFrameData = { 0 };

   
    // ch:获取图像数据 | en:Get image data
    int nRet = MV3D_RGBD_FetchFrame(handle, &stFrameData, 5000);
    if (MV3D_RGBD_OK == nRet)
    {
        if (!stFrameData.nValidInfo)
        {
            LOGD("MV3D_RGBD_FetchFrame success.");
            for (int i = 0; i < stFrameData.nImageCount; i++)
            {
                // ch:解析深度图像并保存 | en:Parse and save depth image
                if (ImageType_Depth == stFrameData.stImageData[i].enImageType)
                {
                    LOGD("Get depth image succeed: FrameNum[%d] Width[%d] Height[%d] Len[%d] CoordinateType[%d]!", stFrameData.stImageData[i].nFrameNum,
                        stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nDataLen, stFrameData.stImageData[i].enCoordinateType);
                    char chDepthFileName[256] = { 0 };
                    sprintf(chDepthFileName, "Depth.png");
                    open3d::geometry::Image depthImage;
                    depthImage.Prepare(stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nHeight, 1, 2);
                    uint8_t* pData = (uint8_t*)stFrameData.stImageData[i].pData;
                    for (int j = 0; j < stFrameData.stImageData[i].nDataLen; j++)
                    {
                        depthImage.data_[j] = (*(pData + j));
                    }
                    open3d::io::WriteImage(chDepthFileName, depthImage);
                }
                // ch:解析彩色图像并保存 | en:Parse and save color image
                else if (ImageType_RGB8_Planar == stFrameData.stImageData[i].enImageType || ImageType_YUV420SP_NV12 == stFrameData.stImageData[i].enImageType
                    || ImageType_YUV420SP_NV21 == stFrameData.stImageData[i].enImageType || ImageType_YUV422 == stFrameData.stImageData[i].enImageType)
                {
                    LOGD("Get color image succeed: FrameNum[%d] Width[%d] Height[%d] Len[%d] CoordinateType[%d]!", stFrameData.stImageData[i].nFrameNum,
                        stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nDataLen, stFrameData.stImageData[i].enCoordinateType);
                    char chColorFileName[256] = { 0 };
                    sprintf(chColorFileName, "Color.png");
                    open3d::geometry::Image rgbImage;
                    rgbImage.Prepare(stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nHeight, 3, 1);
                    uint8_t* pData = (uint8_t*)stFrameData.stImageData[i].pData;
                    for (int j = 0; j < stFrameData.stImageData[i].nWidth * stFrameData.stImageData[i].nHeight; j++)
                    {
                        rgbImage.data_[j * 3] = (*(pData + j)); //R
                        rgbImage.data_[j * 3 + 1] = (*(stFrameData.stImageData[i].nHeight * stFrameData.stImageData[i].nWidth + pData + j)); //G
                        rgbImage.data_[j * 3 + 2] = (*(stFrameData.stImageData[i].nHeight * stFrameData.stImageData[i].nWidth * 2 + pData + j)); //B
                    }
                    open3d::io::WriteImage(chColorFileName, rgbImage);
                }
                // ch:解析原始图像并保存 | en:Parse and save mono image
                else if (ImageType_Mono8 == stFrameData.stImageData[i].enImageType)
                {
                    LOGD("Get mono image succeed: FrameNum[%d] Width[%d] Height[%d] Len[%d] CoordinateType[%d]!", stFrameData.stImageData[i].nFrameNum,
                        stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nDataLen, stFrameData.stImageData[i].enCoordinateType);
                    char chMonoFileName[256] = { 0 };
                    sprintf(chMonoFileName, "Mono.png");
                    open3d::geometry::Image monoImage;
                    monoImage.Prepare(stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nHeight, 1, 1);
                    uint8_t* pData = (uint8_t*)stFrameData.stImageData[i].pData;
                    for (int j = 0; j < stFrameData.stImageData[i].nDataLen; j++)
                    {
                        monoImage.data_[j] = (*(pData + j));
                    }
                    open3d::io::WriteImage(chMonoFileName, monoImage);
                }
                // ch:解析其他类型图像 | en:Parse other types of images
                else
                {
                    LOGD("Get image succeed: FrameNum[%d] Width[%d] Height[%d] Len[%d] CoordinateType[%d] ImageType[%ld] StreamType[%d]!", stFrameData.stImageData[i].nFrameNum,
                        stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nDataLen, stFrameData.stImageData[i].enCoordinateType, stFrameData.stImageData[i].enImageType, stFrameData.stImageData[i].enStreamType);
                }
            }
        }
        else
        {
            LOGD("MV3D_RGBD_FetchFrame lost frame!");
        }
    }

    ASSERT_OK(MV3D_RGBD_Stop(handle));
    ASSERT_OK(MV3D_RGBD_CloseDevice(&handle));
    ASSERT_OK(MV3D_RGBD_Release());

    LOGD("Main done!");
    return  0;
}

