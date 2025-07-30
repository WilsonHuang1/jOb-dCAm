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
    ParamType_Float, ParamType_Int, ParamType_Enum, CoordinateType_Depth, MV3D_RGBD_FLOAT_Z_UNIT, MV3D_RGBD_INT_OUTPUT_RGBD,ImageType_Rgbd,MV3D_RGBD_CAMERA_PARAM,MV3D_RGBD_FLOAT_Z_UNIT

g_bExit = False 
def work_thread(camera=0, rgbIntrinsic=0 , ZUnit= 0, nDataSize=0):
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name='RGBD', width=600, height=400)

    vis1 = o3d.visualization.Visualizer()
    vis1.create_window(window_name='RGBD-To-PointCloud', width=600, height=400)
    while True:
        stFrameData=MV3D_RGBD_FRAME_DATA()
        ret=camera.MV3D_RGBD_FetchFrame(pointer(stFrameData), 5000)
        if ret==0:
            vis.clear_geometries()
            vis1.clear_geometries()
            for i in range(0, stFrameData.nImageCount):
                print("MV3D_RGBD_FetchFrame[%d]:nFrameNum[%d],nDataLen[%d],nWidth[%d],nHeight[%d]" % (
                i, stFrameData.stImageData[i].nFrameNum, stFrameData.stImageData[i].nDataLen, stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nHeight))
                if ImageType_Rgbd == stFrameData.stImageData[i].enImageType:
                    strMode = string_at(stFrameData.stImageData[i].pData, stFrameData.stImageData[i].nDataLen)
                    imageRgbd = np.frombuffer(strMode, dtype=np.int8)

                    imageRgbd = imageRgbd.reshape(stFrameData.stImageData[i].nHeight , stFrameData.stImageData[i].nWidth , 5)
                    RGB = np.zeros((stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nWidth, 3), dtype=np.uint8)
                    RGB[:, :, 2] = imageRgbd[:,:,2]
                    RGB[:, :, 1] = imageRgbd[:,:,1]
                    RGB[:, :, 0] = imageRgbd[:,:,0]
                    Open3dRgb = o3d.geometry.Image(RGB)

                    D1 = imageRgbd[:,:,3].reshape(stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nWidth)
                    D2 = imageRgbd[:,:,4].reshape(stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nWidth)
                    imageDepthUin8 = np.append(D1,D2)
                    imageDepthUin8 = imageDepthUin8.reshape(2,stFrameData.stImageData[i].nHeight * stFrameData.stImageData[i].nWidth).T.reshape(stFrameData.stImageData[i].nHeight * stFrameData.stImageData[i].nWidth * 2)
                    depthBytesArr = imageDepthUin8.tobytes()
                    Depth = np.frombuffer(depthBytesArr, dtype=np.uint16)
                    Depth = Depth.reshape(stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nWidth)
                   
                    Open3dDepth = o3d.geometry.Image(Depth)
                    
                    Open3dRgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(Open3dRgb, Open3dDepth, depth_scale = (1000 / ZUnit) , depth_trunc = 3, convert_rgb_to_intensity = False)
                    vis.add_geometry(Open3dRgbd)
                    vis.poll_events()
                    vis.update_renderer()

                    inter = o3d.camera.PinholeCameraIntrinsic()
                    # W H fx fy cx cy
                    inter.set_intrinsics(stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nHeight, rgbIntrinsic[0], rgbIntrinsic[4], rgbIntrinsic[2], rgbIntrinsic[5])
                    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(Open3dRgbd, inter)

                    vis1.add_geometry(pcd)
                    vis1.poll_events()
                    vis1.update_renderer()
        else:
            print("no data[0x%x]" % ret)
        if g_bExit == True:
            break
    vis.destroy_window()

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

    # ch:设置RGBD输出节点 | en:Set RGBD image output node
    stParam = MV3D_RGBD_PARAM()
    stParam.enParamType = ParamType_Int
    stParam.ParamInfo.stIntParam.nCurValue = 1
    ret = camera.MV3D_RGBD_SetParam(MV3D_RGBD_INT_OUTPUT_RGBD, pointer(stParam))
    if ret != 0:
        print ("SetParam fail! ret[0x%x]" % ret)
        camera.MV3D_RGBD_CloseDevice()
        os.system('pause')
        sys.exit()

    stCameraParam = MV3D_RGBD_CAMERA_PARAM()
    ret = camera.MV3D_RGBD_GetCameraParam(pointer(stCameraParam))
    if ret != 0:
        print ("GetCameraParam fail! ret[0x%x]" % ret)
        camera.MV3D_RGBD_CloseDevice()
        os.system('pause')
        sys.exit()
    
    intrinsic = np.array(stCameraParam.stRgbCalibInfo.stIntrinsic.fData).reshape(9)

    ret = camera.MV3D_RGBD_GetParam(MV3D_RGBD_FLOAT_Z_UNIT, pointer(stParam))
    if ret != 0:
        print ("GetParam fail! ret[0x%x]" % ret)
        camera.MV3D_RGBD_CloseDevice()
        os.system('pause')
        sys.exit()
    ZUnit = stParam.ParamInfo.stFloatParam.fCurValue


    # ch:开始取流 | en:Start grabbing
    ret=camera.MV3D_RGBD_Start()
    if ret != 0:
        print ("start fail! ret[0x%x]" % ret)
        camera.MV3D_RGBD_CloseDevice()
        os.system('pause')
        sys.exit()

    # ch:获取图像线程 | en:Get image thread
    try:
        hthreadhandle=threading.Thread(target=work_thread, args=(camera, intrinsic, ZUnit, None))
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
