
#include "../common/common.hpp"

void __stdcall  CallBackFunc(MV3D_RGBD_EXCEPTION_INFO* pstExceptionData, void* userdata)
{
    if (NULL != pstExceptionData)
    {
        if (DevException_Disconnect == pstExceptionData->enExceptionId)
        {
            *(BOOL*)userdata = TRUE;
            LOGD("=== Event Callback: Device Offline!");
        }
    }
}

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
    int loop_index = 1;
    BOOL bLoop_exit = FALSE;

    while(!bLoop_exit)
    {
        LOGD("========== loop %d", loop_index++);

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
        LOGD("OpenDevice success.\r\n");

        BOOL device_offline = FALSE;
        ASSERT_OK(MV3D_RGBD_RegisterExceptionCallBack(handle, CallBackFunc, &device_offline));
        // ch:开始取流 | en:Start work 
        ASSERT_OK(MV3D_RGBD_Start(handle));
        LOGD("Start work success.\r\n");

        int ch;
        BOOL bExit_main = FALSE;
        BOOL bSaveFrame = FALSE;
        MV3D_RGBD_FRAME_DATA stFrameData = {0};
        while (!bExit_main)
        {
            // ch:获取图象数据 | en:Get image data
            int nRet = MV3D_RGBD_FetchFrame(handle, &stFrameData, 5000);
            if (MV3D_RGBD_OK == nRet)
            {
                for(int i = 0; i < stFrameData.nImageCount; i++)
                {
                   LOGD("MV3D_RGBD_FetchFrame success: FrameNum[%d] Width[%d] Height[%d] Len[%d] CoordinateType[%d]!\r\n", stFrameData.stImageData[i].nFrameNum,
                       stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nDataLen, stFrameData.stImageData[i].enCoordinateType);

                    if ((TRUE == bSaveFrame) && (NULL != stFrameData.stImageData[i].pData))
                    {
                        // ch:保存裸数据 | en:Save raw image
                        FILE* pfile;
                        char filename[256] = {0};

                        if (ImageType_Depth == stFrameData.stImageData[i].enImageType)
                        {
                            sprintf(filename, ("Depth_Frame[%d]_Width[%d]_Height[%d].raw"), stFrameData.stImageData[i].nFrameNum, stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nHeight);
                        } 
                        else if (ImageType_RGB8_Planar == stFrameData.stImageData[i].enImageType)
                        {
                            sprintf(filename, ("RGB_Frame[%d]_Width[%d]_Height[%d].raw"), stFrameData.stImageData[i].nFrameNum, stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nHeight);
                        }

                        pfile = fopen(filename,"wb");
                        if(pfile != NULL)
                        {
                            fwrite(stFrameData.stImageData[i].pData, 1, stFrameData.stImageData[i].nDataLen, pfile);
                            LOGD("Save Raw Success!\r\n");
                            fclose(pfile);
                        }
                        else
                        {
                            LOGD("Save Raw Failed \r\n");
                        }
                    }
                } 
            }

            if(TRUE == device_offline)
            {
                LOGD("Found device offline");
                break;
            }

            LOG("Please enter the action to be performed:\n");
            LOG("[q]Exit Current Frame [s]Save Photo [x]Exit Dev \n");
            if (_kbhit())
            {
                ch = _getch();
                switch(ch)
                {
                case 'q':
                case 'Q':
                    //81
                    bExit_main = TRUE;
                    break;
                case 's':
                case 'S':
                    //83
                    bSaveFrame = TRUE;
                    break;
                case 'x':
                case 'X':
                    //88
                    bExit_main = TRUE;
                    bLoop_exit = TRUE;
                    break;
                default:
                    LOGD("Unmapped key %d", ch);
                    break;
                }
            }
        }

        if(device_offline) 
        {
            LOGD("device offline release resource");
        } 
        else
        {
            LOGD("normal exit");
        }

        ASSERT_OK(MV3D_RGBD_Stop(handle));
        ASSERT_OK(MV3D_RGBD_CloseDevice(&handle));
        ASSERT_OK(MV3D_RGBD_Release());
    }
    
    LOGD("Main done!");
    return  0;
}

