
#include "../common/common.hpp"

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
    for (unsigned int nIndex = 0; nIndex < nDevNum; nIndex++)
    {  
        LOG("Index[%d].", nIndex);
        LOG("******************Detail Info******************");
        if (DeviceType_Ethernet == devs[nIndex].enDeviceType || DeviceType_Ethernet_Vir == devs[nIndex].enDeviceType)
        {
            LOG("SerialNum[%s] IP[%s] name[%s] DeviceVersion[%s].\r\nManufacturerName[%s] UserDefinedName[%s] MacAddress[%x:%x:%x:%x:%x:%x] \
CurrentSubNetMask[%s] DefultGateWay[%s] NetExport[%s] Type[%d].\r\n",
                devs[nIndex].chSerialNumber, devs[nIndex].SpecialInfo.stNetInfo.chCurrentIp, devs[nIndex].chModelName, devs[nIndex].chDeviceVersion,
                devs[nIndex].chManufacturerName, devs[nIndex].chUserDefinedName,
                devs[nIndex].SpecialInfo.stNetInfo.chMacAddress[0], devs[nIndex].SpecialInfo.stNetInfo.chMacAddress[1], devs[nIndex].SpecialInfo.stNetInfo.chMacAddress[2],
                devs[nIndex].SpecialInfo.stNetInfo.chMacAddress[3], devs[nIndex].SpecialInfo.stNetInfo.chMacAddress[4], devs[nIndex].SpecialInfo.stNetInfo.chMacAddress[5],
                devs[nIndex].SpecialInfo.stNetInfo.chCurrentSubNetMask, devs[nIndex].SpecialInfo.stNetInfo.chDefultGateWay, devs[nIndex].SpecialInfo.stNetInfo.chNetExport,
                (devs[nIndex].nDevTypeInfo & 0xff000000) >> 24);

            if (IpCfgMode_Static & devs[nIndex].SpecialInfo.stNetInfo.enIPCfgMode)
            {
                LOG("IPCfgMode[STATIC]");
            }
            if (IpCfgMode_DHCP & devs[nIndex].SpecialInfo.stNetInfo.enIPCfgMode)
            {
                LOG("IPCfgMode[DHCP]");
            }
            if (IpCfgMode_LLA & devs[nIndex].SpecialInfo.stNetInfo.enIPCfgMode)
            {
                LOG("IPCfgMode[LLA]");
            }
        }
        else if (DeviceType_USB == devs[nIndex].enDeviceType || DeviceType_USB_Vir == devs[nIndex].enDeviceType)
        {
            LOG("SerialNum[%s] name[%s] DeviceVersion[%s].\r\nManufacturerName[%s] UserDefinedName[%s] \
VendorId[%d] ProductId[%d] enUsbProtocol[%d] DeviceGUID[%s] Type[%d].\r\n",
                devs[nIndex].chSerialNumber, devs[nIndex].chModelName, devs[nIndex].chDeviceVersion,
                devs[nIndex].chManufacturerName, devs[nIndex].chUserDefinedName,
                devs[nIndex].SpecialInfo.stUsbInfo.nVendorId, devs[nIndex].SpecialInfo.stUsbInfo.nProductId,
                devs[nIndex].SpecialInfo.stUsbInfo.enUsbProtocol, devs[nIndex].SpecialInfo.stUsbInfo.chDeviceGUID,
                (devs[nIndex].nDevTypeInfo & 0xff000000) >> 24);
        }

        LOG("***********************************************\r\n",);
    }

    unsigned int nIndex = 0;
    FILE* pfile;
    char filename[256] = "CurrentDeviceInfo.txt";
    pfile = fopen(filename, "wb");
    if (pfile != NULL)
    {
        char chDevInfo[526] = "";
        if (DeviceType_Ethernet == devs[nIndex].enDeviceType || DeviceType_Ethernet_Vir == devs[nIndex].enDeviceType)
        {
            sprintf(chDevInfo, "SerialNum[%s] IP[%s] name[%s] DeviceVersion[%s].\r\nManufacturerName[%s] UserDefinedName[%s] MacAddress[%x:%x:%x:%x:%x:%x] CurrentSubNetMask[%s] DefultGateWay[%s] NetExport[%s] Type[%d].\r\n",
               devs[nIndex].chSerialNumber, devs[nIndex].SpecialInfo.stNetInfo.chCurrentIp, devs[nIndex].chModelName, devs[nIndex].chDeviceVersion,
               devs[nIndex].chManufacturerName, devs[nIndex].chUserDefinedName, 
               devs[nIndex].SpecialInfo.stNetInfo.chMacAddress[0], devs[nIndex].SpecialInfo.stNetInfo.chMacAddress[1],devs[nIndex].SpecialInfo.stNetInfo.chMacAddress[2], 
               devs[nIndex].SpecialInfo.stNetInfo.chMacAddress[3], devs[nIndex].SpecialInfo.stNetInfo.chMacAddress[4], devs[nIndex].SpecialInfo.stNetInfo.chMacAddress[5],
               devs[nIndex].SpecialInfo.stNetInfo.chCurrentSubNetMask, devs[nIndex].SpecialInfo.stNetInfo.chDefultGateWay, devs[nIndex].SpecialInfo.stNetInfo.chNetExport,
               (devs[nIndex].nDevTypeInfo & 0xff000000) >> 24);
        }
        else if (DeviceType_USB == devs[nIndex].enDeviceType || DeviceType_Ethernet_Vir == devs[nIndex].enDeviceType)
        {  
            sprintf(chDevInfo, "SerialNum[%s] name[%s] DeviceVersion[%s].\r\nManufacturerName[%s] UserDefinedName[%s] VendorId[%d] ProductId[%d] enUsbProtocol[%d] DeviceGUID[%s] Type[%d].\r\n",
                devs[nIndex].chSerialNumber,devs[nIndex].chModelName, devs[nIndex].chDeviceVersion,
                devs[nIndex].chManufacturerName, devs[nIndex].chUserDefinedName, 
                devs[nIndex].SpecialInfo.stUsbInfo.nVendorId, devs[nIndex].SpecialInfo.stUsbInfo.nProductId,
                devs[nIndex].SpecialInfo.stUsbInfo.enUsbProtocol, devs[nIndex].SpecialInfo.stUsbInfo.chDeviceGUID,
                (devs[nIndex].nDevTypeInfo & 0xff000000) >> 24);
        }

        int nRet = fputs(chDevInfo, pfile);
        if (0 <= nRet)
        {
            LOGD("Save Device Info Success!\r\n");
        }
        else
        {
            LOGD("Save Device Info File Failed \r\n");
        }
    }
    else
    {
        LOGD("Save Device Info Failed \r\n");
    }
    fclose(pfile);

    // ch:打开设备 | en:Open device
    void* handle = NULL;
    ASSERT_OK(MV3D_RGBD_OpenDevice(&handle, &devs[nIndex]));
    LOGD("OpenDevice success.");

    ASSERT_OK(MV3D_RGBD_CloseDevice(&handle));
    ASSERT_OK(MV3D_RGBD_Release());
    
    LOGD("Main done!");
    return  0;
}

