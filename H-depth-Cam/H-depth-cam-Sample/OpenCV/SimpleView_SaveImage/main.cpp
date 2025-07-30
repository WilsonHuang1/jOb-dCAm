
#include "../common/common.hpp"
#include "opencv2/opencv.hpp"

unsigned char* g_pRgbData = NULL;

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

	// ch:开始工作流程 | en:Start work
	ASSERT_OK(MV3D_RGBD_Start(handle));
	LOGD("Start work success.");

    BOOL bExit_Main = FALSE;
    MV3D_RGBD_FRAME_DATA stFrameData = {0};
    int nDepthImgSaveCount = 0;
    int nRGBDImgSaveCount = 0;
    while (!bExit_Main )
    {
        // ch:获取图像数据 | en:Get image data
        int nRet = MV3D_RGBD_FetchFrame(handle, &stFrameData, 5000);
        if (MV3D_RGBD_OK == nRet)
        {
			if (0 == stFrameData.nValidInfo)
			{
				for (int i = 0; i < stFrameData.nImageCount; i++)
				{
					LOGD("MV3D_RGBD_FetchFrame Image %d Success: FrameNum[%d] Width[%d] Hidth[%d] Len[%d] CoordinataType[%d]!", i, stFrameData.stImageData[i].nFrameNum,
						stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nDataLen, stFrameData.stImageData[i].enCoordinateType);

					if (ImageType_Depth == stFrameData.stImageData[i].enImageType)
					{
						cv::Mat  mCvmat = cv::Mat(stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nWidth, CV_16UC1, stFrameData.stImageData[i].pData);
						char chFileName[256] = { 0 };
						sprintf(chFileName, "Depth_nFrameNum[%d].png", stFrameData.stImageData[i].nFrameNum);

						if (MAX_IMAGE_COUNT > nDepthImgSaveCount)
						{
							cv::imwrite(chFileName, mCvmat);
							nDepthImgSaveCount++;
						}
					}

					if (ImageType_RGB8_Planar == stFrameData.stImageData[i].enImageType)
					{
						if (NULL == g_pRgbData)
						{
							g_pRgbData = (unsigned char*)malloc(stFrameData.stImageData[i].nDataLen);
							if (NULL == g_pRgbData)
							{
								LOGD("MV3D_RGBD_FetchFrame: g_pRgbData malloc failed!");
								bExit_Main = TRUE;
								continue;
							}
							memset(g_pRgbData, 0, stFrameData.stImageData[i].nDataLen);
						}
						ConvertRGB8Planner2BGR8Packed(stFrameData.stImageData[i].pData, stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nHeight, g_pRgbData);
						cv::Mat  mCvmat = cv::Mat(stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nWidth, CV_8UC3, g_pRgbData);
						char chFileName[256] = { 0 };
						sprintf(chFileName, "RGB_nFrameNum[%d].png", stFrameData.stImageData[i].nFrameNum);

						if (MAX_IMAGE_COUNT > nRGBDImgSaveCount)
						{
							cv::imwrite(chFileName, mCvmat);
							nRGBDImgSaveCount++;
						}
					}

					if (ImageType_YUV422 == stFrameData.stImageData[i].enImageType)
					{
						cv::Mat  mYUVCvmat = cv::Mat(stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nWidth, CV_8UC2, stFrameData.stImageData[i].pData);
						cv::Mat  mCvmat(stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nWidth, CV_8UC3);
						cv::cvtColor(mYUVCvmat, mCvmat, cv::COLOR_YUV2BGR_YUYV);
						char chFileName[256] = { 0 };
						sprintf(chFileName, "RGB_nFrameNum[%d].png", stFrameData.stImageData[i].nFrameNum);
						if (MAX_IMAGE_COUNT > nRGBDImgSaveCount)
						{
							cv::imwrite(chFileName, mCvmat);
							nRGBDImgSaveCount++;
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

