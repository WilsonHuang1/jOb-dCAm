
#include "../common/common.hpp"
#include <Open3D/Open3D.h>

int main(int argc,char** argv)
{
    open3d::geometry::Image rgbImage;
    open3d::geometry::Image depthImage;
    if (!open3d::io::ReadImage("Color.png", rgbImage))
    {
        LOGD("read color image failed!");
        return  0;
    }

    if (!open3d::io::ReadImage("Depth.png", depthImage))
    {
        LOGD("read depth image failed!");
        return  0;
    }

    auto PointCloud = std::make_shared<open3d::geometry::PointCloud>();
    open3d::io::ReadPointCloudOption param;
    if (!open3d::io::ReadPointCloud("PointCloud.ply", *PointCloud, param))
    {
        LOGD("read PointCloud image failed!");
        return  0;
    }

    auto RGBDImage = open3d::geometry::RGBDImage::CreateFromColorAndDepth(rgbImage, depthImage,1000, 3, false);

    open3d::visualization::DrawGeometries({ RGBDImage }, "Open3d-RGBD", 600, 450);
    open3d::visualization::DrawGeometries({ PointCloud }, "Open3d-PointCloud", 600, 450);

    LOGD("Main done!");
    return  0;
}

