
#include "../common/common.hpp"
// #include "../common/RenderImage.hpp"
#include "Mv3dRgbdAdvancedApi.h"
#include "Mv3dRgbdAdvancedDefine.h"

void __stdcall IMUCallBackFunc(MV3D_RGBD_IMU_DATA* pstIMUData, void* pUser)
{
    if (NULL != pstIMUData)
    {
        LOGD("IMU Data: X_Acc(%.1f) Y_Acc(%.1f) Z_Acc(%.1f) X_Gyro(%.1f) Y_Gyro(%.1f) Z_Gyro(%.1f)\r\n", 
             pstIMUData->fXAccSpeed, pstIMUData->fYAccSpeed, pstIMUData->fZAccSpeed, 
             pstIMUData->fXAngSpeed, pstIMUData->fYAngSpeed, pstIMUData->fZAngSpeed);
    }
}
int main(int argc, char** argv)
{
	MV3D_RGBD_VERSION_INFO stVersion;
	ASSERT_OK(MV3D_RGBD_GetSDKVersion(&stVersion));
	LOGD("dll version: %d.%d.%d", stVersion.nMajor, stVersion.nMinor, stVersion.nRevision);

	ASSERT_OK(MV3D_RGBD_Initialize());

	unsigned int nDevNum = 0;
	ASSERT_OK(MV3D_RGBD_GetDeviceNumber(DeviceType_Ethernet | DeviceType_USB, &nDevNum));
	LOGD("MV3D_RGBD_GetDeviceNumber success! nDevNum:%d.", nDevNum);
	ASSERT(nDevNum);

	// �����豸
	std::vector<MV3D_RGBD_DEVICE_INFO> devs(nDevNum);
	ASSERT_OK(MV3D_RGBD_GetDeviceList(DeviceType_Ethernet | DeviceType_USB, &devs[0], nDevNum, &nDevNum));
	for (unsigned int i = 0; i < nDevNum; i++)
	{
		if (DeviceType_Ethernet == devs[i].enDeviceType)
		{
			LOG("Index[%d]. SerialNum[%s] IP[%s] name[%s].\r\n", i, devs[i].chSerialNumber, devs[i].SpecialInfo.stNetInfo.chCurrentIp, devs[i].chModelName);
		}
		else if (DeviceType_USB == devs[i].enDeviceType)
		{
			LOG("Index[%d]. SerialNum[%s] UsbProtocol[%d] name[%s].\r\n", i, devs[i].chSerialNumber, devs[i].SpecialInfo.stUsbInfo.enUsbProtocol, devs[i].chModelName);
		}
	}
	LOG("---------------------------------------------------------------");

	unsigned int nIndex = 0;
	while (true)
	{
		LOG("Please enter the index of the camera to be connected��\n");
		scanf("%d", &nIndex);
		LOG("Connected camera index:%d \r\n", nIndex);

		if ((nDevNum <= nIndex) || (0 > nIndex))
		{
			LOG("enter error!\r\n");
		}
		else
		{
			break;
		}
	}
	LOG("---------------------------------------------------------------\r\n");

	//���豸
	void* handle = NULL;
	ASSERT_OK(MV3D_RGBD_OpenDevice(&handle, &devs[nIndex]));
	LOGD("Open device success");

	MV3D_RGBD_PARAM stParam;
	stParam.enParamType = ParamType_Enum;
	stParam.ParamInfo.stEnumParam.nCurValue = 1;
	ASSERT_OK(MV3D_RGBD_SetParam(handle, "EventNotification", &stParam));
	LOGD("Set EventNotification success.");
	ASSERT_OK(MV3D_RGBD_RegisterIMUDataCallBack(handle, IMUCallBackFunc, handle));
	LOGD("Register IMU data callback success.");

	// ��ʼ��������
	ASSERT_OK(MV3D_RGBD_Start(handle));
	LOGD("Start work success.And press q to exit!");

	BOOL bExit_Main = FALSE;
	MV3D_RGBD_FRAME_DATA stFrameData = { 0 };
	while (!bExit_Main)
	{
		// ��ȡͼ������
		int nRet = MV3D_RGBD_FetchFrame(handle, &stFrameData, 5000);
		if (MV3D_RGBD_OK == nRet)
		{
			for (int i = 0; i < stFrameData.nImageCount; i++)
			{
				/*LOGD("MV3D_RGBD_FetchFrame success: framenum (%d) height(%d) width(%d) len(%d) timestamp(%lld)!\r\n", stFrameData.stImageData[i].nFrameNum,
					stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nDataLen, stFrameData.stImageData[i].nTimeStamp);*/
			}
		}

		//��q���˳�
		if (_kbhit())
		{
			if ('q' == _getch())
			{
				LOGD("recieve exit cmd!");
				bExit_Main = TRUE;
			}
		}
	}

	ASSERT_OK(MV3D_RGBD_Stop(handle));
	ASSERT_OK(MV3D_RGBD_CloseDevice(&handle));
	ASSERT_OK(MV3D_RGBD_Release());

	LOGD("Main done!");
	return  0;
}


//int main(int argc, char** argv)
//{
//    MV3D_RGBD_VERSION_INFO stVersion;
//    ASSERT_OK(MV3D_RGBD_GetSDKVersion(&stVersion));
//    LOGD("dll version: %d.%d.%d", stVersion.nMajor, stVersion.nMinor, stVersion.nRevision);
//
//    ASSERT_OK(MV3D_RGBD_Initialize());
//
//    unsigned int nDevNum = 0;
//    ASSERT_OK(MV3D_RGBD_GetDeviceNumber((DeviceType_Ethernet | DeviceType_USB | DeviceType_Ethernet_Vir | DeviceType_USB_Vir), &nDevNum));
//    LOGD("MV3D_RGBD_GetDeviceNumber success! nDevNum:%d.", nDevNum);
//    ASSERT(nDevNum);
//
//    // �����豸
//    std::vector<MV3D_RGBD_DEVICE_INFO> devs(nDevNum);
//    ASSERT_OK(MV3D_RGBD_GetDeviceList(DeviceType_Ethernet | DeviceType_USB | DeviceType_Ethernet_Vir | DeviceType_USB_Vir, &devs[0], nDevNum, &nDevNum));
//    for (unsigned int i = 0; i < nDevNum; i++)
//    {
//        if (DeviceType_Ethernet == devs[i].enDeviceType || DeviceType_Ethernet_Vir == devs[i].enDeviceType)
//        {
//            LOG("Index[%d]. SerialNum[%s] IP[%s] name[%s] Type[%d].\r\n",
//                i, devs[i].chSerialNumber, devs[i].SpecialInfo.stNetInfo.chCurrentIp, devs[i].chModelName,
//                (devs[i].nDevTypeInfo & 0xff000000) >> 24);
//        }
//        else if (DeviceType_USB == devs[i].enDeviceType || DeviceType_USB_Vir == devs[i].enDeviceType)
//        {
//            LOG("Index[%d]. SerialNum[%s] UsbProtocol[%d] name[%s] Type[%d].\r\n",
//                i, devs[i].chSerialNumber, devs[i].SpecialInfo.stUsbInfo.enUsbProtocol, devs[i].chModelName,
//                (devs[i].nDevTypeInfo & 0xff000000) >> 24);
//        }
//    }
//
//    //���豸
//    void* handle = NULL;
//    unsigned int nIndex = 0;
//    ASSERT_OK(MV3D_RGBD_OpenDevice(&handle, &devs[nIndex]));
//    LOGD("OpenDevice success.");
//
//    // ��ʼ��������
//    ASSERT_OK(MV3D_RGBD_Start(handle));
//    LOGD("Start work success.");
//    BOOL bExit_Main = FALSE;
//    MV3D_RGBD_FRAME_DATA stFrameData = { 0 };
//    RenderImgWnd depthViewer(768, 512, "depth");
//
//    while (!bExit_Main && depthViewer)
//    {
//        // ��ȡͼ������
//        int nRet = MV3D_RGBD_FetchFrame(handle, &stFrameData, 5000);
//        if (MV3D_RGBD_OK == nRet)
//        {
//            if (!stFrameData.nValidInfo)
//            {
//                LOGD("MV3D_RGBD_FetchFrame success.");
//
//                RIFrameInfo depth = { 0 };
//                RIFrameInfo rgb = { 0 };
//                RIFrameInfo rgbd = { 0 };
//                parseFrame(&stFrameData, &depth, &rgb, &rgbd);
//                
//                depthViewer.RenderImage(depth);
//            }
//            else
//            {
//                LOGD("MV3D_RGBD_FetchFrame lost frame!");
//            }
//        }
//
//        //��������˳�
//        if (_kbhit())
//        {
//            bExit_Main = TRUE;
//        }
//    }
//    ASSERT_OK(MV3D_RGBD_Stop(handle));
//    ASSERT_OK(MV3D_RGBD_CloseDevice(&handle));
//    ASSERT_OK(MV3D_RGBD_Release());
//
//    LOGD("Main done!");
//    system("pause");
//    return  0;
//}

