#include <Open3D/Open3D.h>
#include <iostream>

int main() {
    std::cout << "Open3D version: " << open3d::utility::GetVersionString() << std::endl;
    
    // Create a simple point cloud
    auto pcd = std::make_shared<open3d::geometry::PointCloud>();
    pcd->points_.push_back(Eigen::Vector3d(0, 0, 0));
    pcd->points_.push_back(Eigen::Vector3d(1, 0, 0));
    pcd->points_.push_back(Eigen::Vector3d(0, 1, 0));
    
    std::cout << "Created point cloud with " << pcd->points_.size() << " points" << std::endl;
    return 0;
}