# -- coding: utf-8 --
import threading
import msvcrt
import ctypes
import time
import os
import struct
import open3d as o3d
import numpy as np;
from ctypes import *
from Mv3dRgbdImport.Mv3dRgbdDefine import *
from Mv3dRgbdImport.Mv3dRgbdApi import *
from Mv3dRgbdImport.Mv3dRgbdDefine import DeviceType_Ethernet, DeviceType_USB, DeviceType_Ethernet_Vir, DeviceType_USB_Vir, MV3D_RGBD_FLOAT_EXPOSURETIME, \
    ParamType_Float, ParamType_Int, ParamType_Enum, CoordinateType_Depth, MV3D_RGBD_FLOAT_Z_UNIT, MV3D_RGBD_OK, \
    FileType_BMP,ImageType_Depth, ImageType_RGB8_Planar, ImageType_YUV420SP_NV12  ,ImageType_YUV420SP_NV21 , ImageType_YUV422, ImageType_Mono8
    
if __name__ == "__main__":
    nDeviceNum=ctypes.c_uint(0)
    nDeviceNum_p=byref(nDeviceNum)
    # ch:获取设备数量 | en:Get device number
    ret=Mv3dRgbd.MV3D_RGBD_GetDeviceNumber(DeviceType_Ethernet | DeviceType_USB | DeviceType_Ethernet_Vir | DeviceType_USB_Vir, nDeviceNum_p) 
    if  ret!=0:
        print("MV3D_RGBD_GetDeviceNumber fail! ret[0x%x]" % ret)
        os.system('pause')
        sys.exit()
    if  nDeviceNum==0:
        print("find no device!")
        os.system('pause')
        sys.exit()
    print("Find devices numbers:", nDeviceNum.value)
    
    stDeviceList = MV3D_RGBD_DEVICE_INFO_LIST()
    net = Mv3dRgbd.MV3D_RGBD_GetDeviceList(DeviceType_Ethernet | DeviceType_USB | DeviceType_Ethernet_Vir | DeviceType_USB_Vir, pointer(stDeviceList.DeviceInfo[0]), 20, nDeviceNum_p)
    
    for i in range(0, nDeviceNum.value):
        print("\ndevice: [%d]" % i)
        strModeName = ""
        for per in stDeviceList.DeviceInfo[i].chModelName:
            strModeName = strModeName + chr(per)
        print("device model name: %s" % strModeName)

        strSerialNumber = ""
        for per in stDeviceList.DeviceInfo[i].chSerialNumber:
            strSerialNumber = strSerialNumber + chr(per)
        print("device SerialNumber: %s" % strSerialNumber)

    # ch:创建相机示例 | en:Create a camera instance
    camera=Mv3dRgbd()
    nConnectionNum = input("please input the number of the device to connect:")
    if int(nConnectionNum) >= nDeviceNum.value:
        print("intput error!")
        os.system('pause')
        sys.exit()

    # ch:打开设备 | en:Open device  
    ret = camera.MV3D_RGBD_OpenDevice(pointer(stDeviceList.DeviceInfo[int(nConnectionNum)]))
    if ret != 0:
        print ("MV3D_RGBD_OpenDevice fail! ret[0x%x]" % ret)
        os.system('pause')
        sys.exit()

    # ch:开始取流 | en:Start grabbing
    ret=camera.MV3D_RGBD_Start()
    if ret != 0:
        print ("start fail! ret[0x%x]" % ret)
        camera.MV3D_RGBD_CloseDevice()
        os.system('pause')
        sys.exit()

    stFrameData = MV3D_RGBD_FRAME_DATA()
    # ch:获取图像数据 | en:Get image data
    ret = camera.MV3D_RGBD_FetchFrame(pointer(stFrameData), 5000)
    if MV3D_RGBD_OK == ret:
        print("MV3D_RGBD_FetchFrame success.")
        for i in range(0, stFrameData.nImageCount):
            # ch:解析深度图像并保存 | en:Parse and save depth image
            if ImageType_Depth == stFrameData.stImageData[i].enImageType:
                print("Get depth image succeed: framenum (%d) height(%d) width(%d) len(%d)!" %( stFrameData.stImageData[i].nFrameNum,
                    stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nDataLen))
                chDepthFileName ="Depth.png"
                imageDepth = o3d.geometry.Image()
                strMode = string_at(stFrameData.stImageData[i].pData, stFrameData.stImageData[i].nDataLen)
                image = np.frombuffer(strMode, dtype=np.uint16)
                image = image.reshape(stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nWidth)
                imageDepth = o3d.geometry.Image(image)
                o3d.io.write_image(chDepthFileName,imageDepth)
            # ch:解析彩色图像并保存 | en:Parse and save color image
            elif ImageType_RGB8_Planar == stFrameData.stImageData[i].enImageType:
                print("Get color image succeed: framenum (%d) height(%d) width(%d) len(%d)!" % (stFrameData.stImageData[i].nFrameNum,
                    stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nDataLen))
                chColorFileName = "Color.png"
                strMode = string_at(stFrameData.stImageData[i].pData, stFrameData.stImageData[i].nDataLen)
                image = np.frombuffer(strMode, dtype=np.uint8)
                image = image.reshape(3, stFrameData.stImageData[i].nHeight * stFrameData.stImageData[i].nWidth)
                image = image.T.reshape(stFrameData.stImageData[i].nDataLen)
                image = image.reshape(stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nWidth, 3)
                imageRgb = o3d.geometry.Image(image)
                o3d.io.write_image(chColorFileName,imageRgb)
            
            # ch:解析原始图像并保存 | en:Parse and save mono image
            elif ImageType_Mono8 == stFrameData.stImageData[i].enImageType:
                print("Get mono image succeed: framenum (%d) height(%d) width(%d) len(%d)!" % (stFrameData.stImageData[i].nFrameNum,
                    stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nDataLen))
                chMonoFileName = "Mono.png"
                imageMono = o3d.geometry.Image()
                strMode = string_at(stFrameData.stImageData[i].pData, stFrameData.stImageData[i].nDataLen)
                image = np.frombuffer(strMode, dtype=np.uint8)
                image = image.reshape(stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nWidth)
                imageMono = o3d.geometry.Image(image)
                o3d.io.write_image(chMonoFileName,imageMono)
                
            # ch:解析其他类型图像 | en:Parse other types of images
            else:
                print("Get image succeed: framenum (%d) image type(%ld) height(%d) width(%d) len(%d)!" %(stFrameData.stImageData[i].nFrameNum, stFrameData.stImageData[i].enImageType, 
                    stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nDataLen))          
    else:
        print("MV3D_RGBD_FetchFrame lost frame!")
            
    # ch:停止取流 | en:Stop grabbing
    ret=camera.MV3D_RGBD_Stop()
    if ret != 0:
        print ("stop fail! ret[0x%x]" % ret)
        os.system('pause')
        sys.exit()

    # ch:销毁句柄 | en:Destroy the device handle 
    ret=camera.MV3D_RGBD_CloseDevice()
    if ret != 0:
        print ("CloseDevice fail! ret[0x%x]" % ret)
        os.system('pause')
        sys.exit()
    
    print ("Main done")
    sys.exit()
