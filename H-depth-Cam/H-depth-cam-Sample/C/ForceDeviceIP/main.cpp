
#include "../common/common.hpp"

int GetDestIP(MV3D_RGBD_IP_CONFIG& stIpCfgInfo)
{
    LOG("Please enter the network mode of the camera to be configure:\n");
    LOG("[1] STATIC [2] DHCP [3] LLA \n");

    unsigned int nIPCfgMode  = 0;
    scanf("%d",&nIPCfgMode);

    if (1 == nIPCfgMode)
    {
        char  pNewIP[16] = "";
        char  pNewNetmask[16] = "";
        char  pNewGateway[16] = "";

        LOG("Please enter the camera IP to be set:\n");
        scanf("%s",pNewIP);
        LOG("Please enter the camera Netmask to be set:\n");
        scanf("%s",pNewNetmask);
        LOG("Please enter the camera Gateway to be set:\n");
        scanf("%s",pNewGateway);
        
        memcpy(stIpCfgInfo.chDestIp,     pNewIP,    strlen(pNewIP));
        memcpy(stIpCfgInfo.chDestNetMask,pNewNetmask,strlen(pNewNetmask));
        memcpy(stIpCfgInfo.chDestGateWay,pNewGateway,strlen(pNewGateway));
    }
    else if (3 == nIPCfgMode)
    {
        // ch:备注  | en:Remark
        /* 
        typedef enum Mv3dRgbdIpCfgMode
        {
            IpCfgMode_Static     = 1,           // \~chinese 静态IP                   \~english Static IP mode
            IpCfgMode_DHCP       = 2,           // \~chinese 自动分配IP(DHCP)         \~english Automatically assigned IP address (DHCP)
            IpCfgMode_LLA        = 4            // \~chinese 自动分配IP(LLA)          \~english Automatically assigned IP address (LLA) 
        } Mv3dRgbdIpCfgMode;
        */
        nIPCfgMode = IpCfgMode_LLA;
    }
    else if ((0 >=  nIPCfgMode)  || (3 < nIPCfgMode))
    {
        LOG("Enter error ! Get it error!");
        return 0;
    }

    stIpCfgInfo.enIPCfgMode = (Mv3dRgbdIpCfgMode)nIPCfgMode;
    return 1;
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

    MV3D_RGBD_IP_CONFIG    stIpCfgInfo;
    memset(&stIpCfgInfo, 0, sizeof(stIpCfgInfo));

    int nRet = GetDestIP(stIpCfgInfo);
    if (0 == nRet)
    {
        LOGD("GetDestIP fail!");
    }
    else
    {
        ASSERT_OK(MV3D_RGBD_SetIpConfig((char*)devs[0].chSerialNumber,&stIpCfgInfo));
        LOGD("MV3D_RGBD_SetIpConfig success!");
    }

    ASSERT_OK(MV3D_RGBD_Release());

    LOGD("Main done!");
    return  0;
}

