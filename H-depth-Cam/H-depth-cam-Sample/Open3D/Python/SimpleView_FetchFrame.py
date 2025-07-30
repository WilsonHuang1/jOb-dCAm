# -- coding: utf-8 --
import threading
import ctypes
import time
import os
import open3d as o3d
import numpy as np;
from ctypes import *
from Mv3dRgbdImport.Mv3dRgbdDefine import *
from Mv3dRgbdImport.Mv3dRgbdApi import *
from Mv3dRgbdImport.Mv3dRgbdDefine import DeviceType_Ethernet, DeviceType_USB,  MV3D_RGBD_FLOAT_EXPOSURETIME, \
    ParamType_Float, ParamType_Int, ParamType_Enum, CoordinateType_Depth, MV3D_RGBD_FLOAT_Z_UNIT, ImageType_Depth, ImageType_RGB8_Planar,ImageType_YUV422,ImageType_YUV420SP_NV12,ImageType_YUV420SP_NV21

g_bExit = False 
def work_thread(camera=0,nDataSize=0):
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name='RGB', width=600, height=400)
    vis1 = o3d.visualization.Visualizer()
    vis1.create_window(window_name='DEPTH', width=600, height=400)
    imageDepth = o3d.geometry.Image()
    imageRgb = o3d.geometry.Image()
    while True:
        stFrameData=MV3D_RGBD_FRAME_DATA()
        ret=camera.MV3D_RGBD_FetchFrame(pointer(stFrameData), 5000)
        if ret==0:
            vis.clear_geometries()
            vis1.clear_geometries()
            for i in range(0, stFrameData.nImageCount):
                print("MV3D_RGBD_FetchFrame[%d]:nFrameNum[%d],nDataLen[%d],nWidth[%d],nHeight[%d]" % (
                i, stFrameData.stImageData[i].nFrameNum, stFrameData.stImageData[i].nDataLen, stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nHeight))
                if ImageType_Depth == stFrameData.stImageData[i].enImageType:
                    strMode = string_at(stFrameData.stImageData[i].pData, stFrameData.stImageData[i].nDataLen)
                    image = np.frombuffer(strMode, dtype=np.uint16)
                    image = image.reshape(stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nWidth)
                    imageDepth = o3d.geometry.Image(image)

                if ImageType_RGB8_Planar == stFrameData.stImageData[i].enImageType:
                    strMode = string_at(stFrameData.stImageData[i].pData, stFrameData.stImageData[i].nDataLen)
                    image = np.frombuffer(strMode, dtype=np.uint8)
                    image = image.reshape(3, stFrameData.stImageData[i].nHeight * stFrameData.stImageData[i].nWidth)
                    image = image.T.reshape(stFrameData.stImageData[i].nDataLen)
                    image = image.reshape(stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nWidth, 3)
                    imageRgb = o3d.geometry.Image(image)

                if ImageType_YUV422 == stFrameData.stImageData[i].enImageType:
                    YUV = string_at(stFrameData.stImageData[i].pData, stFrameData.stImageData[i].nDataLen)
                    YUV_np = np.frombuffer(YUV, dtype=np.uint8)
                    YUV_np = YUV_np.reshape((int)(stFrameData.stImageData[i].nHeight * stFrameData.stImageData[i].nWidth / 2), 4)
                    YUV_np = YUV_np.T.reshape(stFrameData.stImageData[i].nDataLen)
                    Y1 = YUV_np[0:(int)(stFrameData.stImageData[i].nHeight * stFrameData.stImageData[i].nWidth /2)]
                    U = YUV_np[(int)(stFrameData.stImageData[i].nHeight * stFrameData.stImageData[i].nWidth/2) : stFrameData.stImageData[i].nHeight * stFrameData.stImageData[i].nWidth]
                    U = np.repeat(U,2)
                    Y2 = YUV_np[stFrameData.stImageData[i].nHeight * stFrameData.stImageData[i].nWidth : (int)(3 * stFrameData.stImageData[i].nHeight * stFrameData.stImageData[i].nWidth/2)]
                    V = YUV_np[(int)(3 * stFrameData.stImageData[i].nHeight * stFrameData.stImageData[i].nWidth / 2) : 2 * stFrameData.stImageData[i].nHeight * stFrameData.stImageData[i].nWidth]
                    V = np.repeat(V,2)

                    Y = np.append(Y1, Y2)
                    Y = Y.reshape(2, (int)(stFrameData.stImageData[i].nHeight * stFrameData.stImageData[i].nWidth/2)).T
                    Y = Y.reshape(stFrameData.stImageData[i].nHeight * stFrameData.stImageData[i].nWidth)

                    R = Y + 1.370705 * ( V - np.array([128]))
                    G = Y - 0.698001 * ( U - np.array([128])) - (0.703125 * (V - np.array([128])))
                    B = Y + 1.732446 * ( U - np.array([128]))
                
                    R = np.where(R < 0, 0, R)
                    R = np.where(R > 255,255,R)
                    G = np.where(G < 0, 0, G)
                    G = np.where(G > 255,255,G)
                    B = np.where(B < 0, 0, B)
                    B = np.where(B > 255,255,B)

                    RGB = np.zeros((stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nWidth, 3), dtype=np.uint8)
                    RGB[:, :, 2] = B.reshape(stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nWidth)
                    RGB[:, :, 1] = G.reshape(stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nWidth)
                    RGB[:, :, 0] = R.reshape(stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nWidth)
                    imageRgb = o3d.geometry.Image(RGB)

                if ImageType_YUV420SP_NV12 == stFrameData.stImageData[i].enImageType:
                    YUV = string_at(stFrameData.stImageData[i].pData, stFrameData.stImageData[i].nDataLen)
                    YUV_np = np.frombuffer(YUV, dtype=np.uint8)
                    Y = YUV_np[0:stFrameData.stImageData[i].nHeight * stFrameData.stImageData[i].nWidth]
                    UV = YUV_np[stFrameData.stImageData[i].nHeight * stFrameData.stImageData[i].nWidth:(int)(3 * stFrameData.stImageData[i].nHeight * stFrameData.stImageData[i].nWidth / 2)]
                    UV = UV.reshape((int)(stFrameData.stImageData[i].nHeight * stFrameData.stImageData[i].nWidth / 4), 2)
                    UV = UV.T.reshape((int)(stFrameData.stImageData[i].nHeight * stFrameData.stImageData[i].nWidth / 2))
                    U = UV[0 : (int)(stFrameData.stImageData[i].nHeight * stFrameData.stImageData[i].nWidth / 4)]
                    U = np.repeat(U,2).reshape((int)(stFrameData.stImageData[i].nHeight/2),stFrameData.stImageData[i].nWidth)
                    U = np.repeat(U,2, axis = 0).reshape(stFrameData.stImageData[i].nHeight * stFrameData.stImageData[i].nWidth)

                    V = UV[(int)(stFrameData.stImageData[i].nHeight * stFrameData.stImageData[i].nWidth / 4) : (int)(stFrameData.stImageData[i].nHeight * stFrameData.stImageData[i].nWidth / 2)]
                    V = np.repeat(V,2).reshape((int)(stFrameData.stImageData[i].nHeight/2),stFrameData.stImageData[i].nWidth)
                    V = np.repeat(V, 2, axis = 0).reshape(stFrameData.stImageData[i].nHeight * stFrameData.stImageData[i].nWidth)

                    R = Y + (140 * ( V- np.array([128])) ) / 100
                    G = Y - (34 * (U - np.array([128]))) / 100 - (71 * (V - np.array([128]))) / 100
                    B = Y + (177 * (U - np.array([128]))) / 100

                    R = np.where(R < 0, 0, R)
                    R = np.where(R > 255,255,R)
                    G = np.where(G < 0, 0, G)
                    G = np.where(G > 255,255,G)
                    B = np.where(B < 0, 0, B)
                    B = np.where(B > 255,255,B)

                    RGB = np.zeros((stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nWidth, 3), dtype=np.uint8)
                    RGB[:, :, 2] = B.reshape(stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nWidth)
                    RGB[:, :, 1] = G.reshape(stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nWidth)
                    RGB[:, :, 0] = R.reshape(stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nWidth)
                    imageRgb = o3d.geometry.Image(RGB)

                if ImageType_YUV420SP_NV21 == stFrameData.stImageData[i].enImageType:
                    YUV = string_at(stFrameData.stImageData[i].pData, stFrameData.stImageData[i].nDataLen)
                    YUV_np = np.frombuffer(YUV, dtype=np.uint8)
                    Y = YUV_np[0:stFrameData.stImageData[i].nHeight * stFrameData.stImageData[i].nWidth]
                    UV = YUV_np[stFrameData.stImageData[i].nHeight * stFrameData.stImageData[i].nWidth:(int)(3 * stFrameData.stImageData[i].nHeight * stFrameData.stImageData[i].nWidth / 2)]
                    UV = UV.reshape((int)(stFrameData.stImageData[i].nHeight * stFrameData.stImageData[i].nWidth / 4), 2)
                    UV = UV.T.reshape((int)(stFrameData.stImageData[i].nHeight * stFrameData.stImageData[i].nWidth / 2))
                    V = UV[0 : (int)(stFrameData.stImageData[i].nHeight * stFrameData.stImageData[i].nWidth / 4)]
                    V = np.repeat(V,2).reshape((int)(stFrameData.stImageData[i].nHeight/2),stFrameData.stImageData[i].nWidth)
                    V = np.repeat(V,2, axis = 0).reshape(stFrameData.stImageData[i].nHeight * stFrameData.stImageData[i].nWidth)

                    U = UV[(int)(stFrameData.stImageData[i].nHeight * stFrameData.stImageData[i].nWidth / 4) : (int)(stFrameData.stImageData[i].nHeight * stFrameData.stImageData[i].nWidth / 2)]
                    U = np.repeat(U,2).reshape((int)(stFrameData.stImageData[i].nHeight/2),stFrameData.stImageData[i].nWidth)
                    U = np.repeat(U, 2, axis = 0).reshape(stFrameData.stImageData[i].nHeight * stFrameData.stImageData[i].nWidth)

                    R = Y + (140 * ( V- np.array([128])) ) / 100
                    G = Y - (34 * (U - np.array([128]))) / 100 - (71 * (V - np.array([128]))) / 100
                    B = Y + (177 * (U - np.array([128]))) / 100

                    R = np.where(R < 0, 0, R)
                    R = np.where(R > 255,255,R)
                    G = np.where(G < 0, 0, G)
                    G = np.where(G > 255,255,G)
                    B = np.where(B < 0, 0, B)
                    B = np.where(B > 255,255,B)

                    RGB = np.zeros((stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nWidth, 3), dtype=np.uint8)
                    RGB[:, :, 2] = B.reshape(stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nWidth)
                    RGB[:, :, 1] = G.reshape(stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nWidth)
                    RGB[:, :, 0] = R.reshape(stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nWidth)
                    imageRgb = o3d.geometry.Image(RGB)
            
            vis.add_geometry(imageRgb)
            vis.poll_events()
            vis.update_renderer()

            vis1.add_geometry(imageDepth)
            vis1.poll_events()
            vis1.update_renderer()
        else:
            print("no data[0x%x]" % ret)
        if g_bExit == True:
            break
    vis.destroy_window()
    vis1.destroy_window()

if __name__ == "__main__":
    nDeviceNum=ctypes.c_uint(0)
    nDeviceNum_p=byref(nDeviceNum)
    # ch:获取设备数量 | en:Get device number
    ret=Mv3dRgbd.MV3D_RGBD_GetDeviceNumber(DeviceType_Ethernet | DeviceType_USB , nDeviceNum_p) 
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
    net = Mv3dRgbd.MV3D_RGBD_GetDeviceList(DeviceType_Ethernet | DeviceType_USB , pointer(stDeviceList.DeviceInfo[0]), 20, nDeviceNum_p)
    
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

    # ch:获取图像线程 | en:Get image thread
    try:
        hthreadhandle=threading.Thread(target=work_thread,args=(camera,None))
        hthreadhandle.start()
    except:
        print("error: unable to start thread")
    
    time.sleep(20)
    g_bExit = True
    hthreadhandle.join()

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
    
    sys.exit()
