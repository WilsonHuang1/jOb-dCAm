# -- coding: utf-8 --
import threading
import msvcrt
import ctypes
import time
import os
import struct
import open3d as o3d;
import numpy as np;

if __name__ == "__main__":
    
    rgbimage = o3d.io.read_image("Color.png")       
    depthImage = o3d.io.read_image("Depth.png") 
    Open3dRgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(rgbimage, depthImage, depth_scale = 1000, convert_rgb_to_intensity = False)
    o3d.visualization.draw_geometries([Open3dRgbd],window_name ="Open3d RGBD图像渲染",width=600,height=450)
    
    pointCloud = o3d.io.read_point_cloud("PointCloud.ply")
    o3d.visualization.draw_geometries([pointCloud],window_name ="Open3d点云渲染",width=600,height=450)

 
