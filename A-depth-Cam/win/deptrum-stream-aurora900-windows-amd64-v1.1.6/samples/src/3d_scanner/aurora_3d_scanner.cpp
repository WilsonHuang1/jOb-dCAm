void PrintUsage() {
    std::cout << "\n=== Aurora 3D Scanner Controls ===" << std::endl;
    std::cout << "Key Controls:" << std::endl;
    std::cout << "  ESC/q     - Quit application" << std::endl;
    std::cout << "  s#include <iostream>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cfloat>
#include <cmath>

// Deptrum SDK headers
#include "deptrum/device.h"
#include "deptrum/stream.h"
#include "functional/base.h"
#include "functional/frame_rate_helper.h"
#include "sample_helper.h"

#ifdef DEVICE_TYPE_STELLAR200
#include "deptrum/stellar200_series.h"
#endif
#ifdef DEVICE_TYPE_AURORA300
#include "deptrum/aurora300_series.h"
#endif
#ifdef DEVICE_TYPE_STELLAR400
#include "deptrum/stellar400_series.h"
#endif
#ifdef DEVICE_TYPE_AURORA500
#include "deptrum/aurora500.h"
#endif
#ifdef DEVICE_TYPE_AURORA700
#include "deptrum/aurora700.h"
#endif
#ifdef DEVICE_TYPE_AURORA900
#include "deptrum/aurora900_series.h"
#endif
#ifdef DEVICE_TYPE_NEBULA100
#include "deptrum/nebula100.h"
#endif
#ifdef DEVICE_TYPE_NEBULA200
#include "deptrum/nebula200_series.h"
#endif

// OpenCV for visualization
#include "opencv2/opencv.hpp"
#ifdef HAVE_OPENCV_VIZ
#include "opencv2/viz.hpp"  // For 3D visualization (optional)
#endif

// Forward declarations for SavePoint functions (from sample_helper.cc)
void SavePointXyz(const std::string& file_name, char* pBuffer, int frame_length);
void SavePointXyzRgb(const std::string& file_name, char* pBuffer, int frame_length);
void SavePointXyzRgbIr(const std::string& file_name, char* pBuffer, int frame_length);

// If sample_helper functions are not available, include them here
#ifndef SAMPLE_HELPER_INCLUDED
void SavePointXyz(const std::string& file_name, char* pBuffer, int frame_length) {
    deptrum::PointXyz<float>* current_point = (deptrum::PointXyz<float>*) pBuffer;
    auto point_size = frame_length / sizeof(deptrum::PointXyz<float>);

    std::ofstream out(file_name, std::ios::out);
    out << "ply\n";
    out << "format ascii 1.0\n";
    out << "comment created by Aurora 3D Scanner\n";
    out << "element vertex " << point_size << "\n";
    out << "property float x\n";
    out << "property float y\n";
    out << "property float z\n";
    out << "end_header\n";

    for (size_t i = 0; i < point_size; i++) {
        out << (float) current_point[i].x << "\t" << (float) current_point[i].y << "\t"
            << (float) current_point[i].z << "\n";
    }
    out.flush();
    out.close();
}

void SavePointXyzRgb(const std::string& file_name, char* pBuffer, int frame_length) {
    deptrum::PointXyzRgb<float>* current_point = (deptrum::PointXyzRgb<float>*) pBuffer;
    auto point_size = frame_length / sizeof(deptrum::PointXyzRgb<float>);

    std::ofstream out(file_name, std::ios::out);
    out << "ply\n";
    out << "format ascii 1.0\n";
    out << "comment created by Aurora 3D Scanner\n";
    out << "element vertex " << point_size << "\n";
    out << "property float x\n";
    out << "property float y\n";
    out << "property float z\n";
    out << "property uchar red\n";
    out << "property uchar green\n";
    out << "property uchar blue\n";
    out << "end_header\n";

    for (size_t i = 0; i < point_size; i++) {
        out << (float) current_point[i].x << "\t" << (float) current_point[i].y << "\t"
            << (float) current_point[i].z << "\t " << (float) current_point[i].r << "\t"
            << (float) current_point[i].g << "\t" << (float) current_point[i].b << "\n";
    }
    out.flush();
    out.close();
}

void SavePointXyzRgbIr(const std::string& file_name, char* pBuffer, int frame_length) {
    deptrum::PointXyzRgbIr<float>* current_point = (deptrum::PointXyzRgbIr<float>*) pBuffer;
    auto point_size = frame_length / sizeof(deptrum::PointXyzRgbIr<float>);

    std::ofstream out(file_name, std::ios::out);
    out << "ply\n";
    out << "format ascii 1.0\n";
    out << "comment created by Aurora 3D Scanner\n";
    out << "element vertex " << point_size << "\n";
    out << "property float x\n";
    out << "property float y\n";
    out << "property float z\n";
    out << "property uchar red\n";
    out << "property uchar green\n";
    out << "property uchar blue\n";
    out << "property uchar ir\n";
    out << "end_header\n";

    for (size_t i = 0; i < point_size; i++) {
        out << (float) current_point[i].x << "\t" << (float) current_point[i].y << "\t"
            << (float) current_point[i].z << "\t " << (float) current_point[i].r << "\t"
            << (float) current_point[i].g << "\t" << (float) current_point[i].b << "\t"
            << (float) current_point[i].ir << "\n";
    }
    out.flush();
    out.close();
}
#endif // SAMPLE_HELPER_INCLUDED

using namespace std;
using namespace deptrum;
using namespace deptrum::stream;

// Global variables
CameraParam g_camera_param;
bool is_running{true};
bool is_print_fps{false};
bool save_point_clouds{false};
bool show_3d_viewer{true};
bool show_depth_projection{true};
int frame_count = 0;

#ifdef HAVE_OPENCV_VIZ
// 3D Viewer (only if viz module is available)
cv::viz::Viz3d point_cloud_window("3D Scanner - Point Cloud");
#endif

std::unordered_map<int, std::string> stream_type_map = {
    {StreamType::kRgb, "Rgb"},
    {StreamType::kIr, "Ir"},
    {StreamType::kDepth, "Depth"},
    {StreamType::kRgbd, "Rgbd"},
    {StreamType::kRgbIr, "RgbIr"},
    {StreamType::kRgbdIr, "RgbdIr"},
    {StreamType::kDepthIr, "DepthIr"},
    {StreamType::kDepthIrLaser, "DepthIrLaser"},
    {StreamType::kSpeckleCloud, "SpeckleCloud"},
    {StreamType::kPointCloud, "PointCloud"},
    {StreamType::kRgbdPointCloud, "RgbdPointCloud"},
    {StreamType::kRgbdIrPointCloud, "RgbdIrPointCloud"},
    {StreamType::kRgbdIrFlag, "RgbdIrFlag"},
    {StreamType::kDepthIrFlag, "DepthIrFlag"}
};

std::unordered_map<int, std::string> frame_type_to_string_map = {
    {FrameType::kInvalidFrameType, "Invalid"},
    {FrameType::kRgbFrame, "RGB"},
    {FrameType::kIrFrame, "IR"},
    {FrameType::kDepthFrame, "Depth"},
    {FrameType::kLaserFrame, "Laser"},
    {FrameType::kPointCloudFrame, "PointCloud"},
    {FrameType::kRgbdPointCloudFrame, "RgbdPointCloud"},
    {FrameType::kRgbdIrPointCloudFrame, "RgbdIrPointCloud"}
};

std::unordered_map<int, std::string> frame_attr_to_string_map = {
    {FrameAttr::kAttrInvalid, "Invalid"},
    {FrameAttr::kAttrLeft, "Left"},
    {FrameAttr::kAttrRight, "Right"}
};

// Enhanced ViewerHelper class for point cloud handling
class Enhanced3DViewerHelper {
private:
    std::string device_type_;
    std::unordered_set<std::string> viewer_set_;
    
public:
    Enhanced3DViewerHelper(const std::string& device_type) : device_type_(device_type) {}
    
    void ShowPointCloud(const StreamFrames& frames) {
        for (int index = 0; index < frames.count; index++) {
            auto frame = frames.frame_ptr[index];
            if (!frame || !frame->data) continue;
            
            FrameType frame_type = frame->frame_type;
            
            // Handle different types of point cloud frames
            if (frame_type == kPointCloudFrame) {
                DisplayXyzPointCloud(frame);
            }
            else if (frame_type == kRgbdPointCloudFrame) {
                DisplayRgbPointCloud(frame);
            }
            else if (frame_type == kRgbdIrPointCloudFrame) {
                DisplayRgbIrPointCloud(frame);
            }
        }
    }
    
    void ShowFrame(const StreamFrames& frames, CameraParam camera_param = {}) {
        for (int index = 0; index < frames.count; index++) {
            auto frame = frames.frame_ptr[index];
            FrameType frame_type = frame->frame_type;
            std::string title;
            
            if (frame->frame_attr != FrameAttr::kAttrInvalid) {
                title = frame_type_to_string_map[frame_type] + "_" + 
                        frame_attr_to_string_map[frame->frame_attr];
            } else {
                title = frame_type_to_string_map[frame_type];
            }

            // Handle point cloud frames instead of skipping them
            if (kPointCloudFrame == frame->frame_type || 
                kRgbdPointCloudFrame == frame->frame_type ||
                kRgbdIrPointCloudFrame == frame->frame_type) {
                
                if (show_3d_viewer) {
                    ShowPointCloud(frames);
                }
                
                if (save_point_clouds) {
                    SavePointCloudFrame(frame);
                }
                continue;
            }

            // Handle regular frames (RGB, Depth, IR)
            auto iter = viewer_set_.find(title);
            if (iter == viewer_set_.end()) {
                InitViewer(frame.get(), title);
            }

            switch (frame_type) {
                case kRgbFrame: {
                    DisplayRgbFrame(frame, title);
                    break;
                }
                case kDepthFrame: {
                    DisplayDepthFrame(frame, title);
                    break;
                }
                case kIrFrame: {
                    DisplayIrFrame(frame, title);
                    break;
                }
                default:
                    break;
            }
        }
    }
    
private:
    void InitViewer(StreamFrame* frame, const std::string& title) {
        cv::namedWindow(title, cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
        cv::resizeWindow(title, frame->cols, frame->rows);
        viewer_set_.emplace(title);
    }
    
    void DisplayXyzPointCloud(std::shared_ptr<StreamFrame> frame) {
        if (!frame->data || frame->size == 0) return;
        
        // Cast data to point cloud format
        deptrum::PointXyz<float>* points = (deptrum::PointXyz<float>*)frame->data.get();
        size_t point_count = frame->size / sizeof(deptrum::PointXyz<float>);
        
        std::cout << "Processing XYZ Point Cloud: " << point_count << " points" << std::endl;
        
#ifdef HAVE_OPENCV_VIZ
        // Use OpenCV viz if available
        std::vector<cv::Point3f> cv_points;
        cv_points.reserve(point_count);
        
        for (size_t i = 0; i < point_count; i++) {
            if (std::isfinite(points[i].x) && std::isfinite(points[i].y) && std::isfinite(points[i].z) &&
                points[i].z > 0.1f && points[i].z < 5000.0f) {
                cv_points.emplace_back(points[i].x / 1000.0f, points[i].y / 1000.0f, points[i].z / 1000.0f);
            }
        }
        
        if (!cv_points.empty()) {
            cv::viz::WCloud cloud_widget(cv_points, cv::viz::Color::white());
            point_cloud_window.showWidget("point_cloud", cloud_widget);
            cv::Affine3d cam_pose = cv::viz::makeCameraPose(cv::Vec3d(0, 0, -2), cv::Vec3d(0, 0, 0), cv::Vec3d(0, -1, 0));
            point_cloud_window.setViewerPose(cam_pose);
            point_cloud_window.spinOnce(1, true);
        }
#else
        // Alternative: Create depth projection image
        if (show_depth_projection) {
            CreateDepthProjection(points, point_count, "XYZ Point Cloud Projection");
        }
#endif
    }
    
    void DisplayRgbPointCloud(std::shared_ptr<StreamFrame> frame) {
        if (!frame->data || frame->size == 0) return;
        
        // Cast data to RGB point cloud format
        deptrum::PointXyzRgb<float>* points = (deptrum::PointXyzRgb<float>*)frame->data.get();
        size_t point_count = frame->size / sizeof(deptrum::PointXyzRgb<float>);
        
        std::cout << "Processing RGB Point Cloud: " << point_count << " points" << std::endl;
        
#ifdef HAVE_OPENCV_VIZ
        // Use OpenCV viz if available
        std::vector<cv::Point3f> cv_points;
        std::vector<cv::Vec3b> cv_colors;
        cv_points.reserve(point_count);
        cv_colors.reserve(point_count);
        
        for (size_t i = 0; i < point_count; i++) {
            if (std::isfinite(points[i].x) && std::isfinite(points[i].y) && std::isfinite(points[i].z) &&
                points[i].z > 0.1f && points[i].z < 5000.0f) {
                cv_points.emplace_back(points[i].x / 1000.0f, points[i].y / 1000.0f, points[i].z / 1000.0f);
                cv_colors.emplace_back(points[i].b, points[i].g, points[i].r);
            }
        }
        
        if (!cv_points.empty()) {
            cv::viz::WCloud cloud_widget(cv_points, cv_colors);
            point_cloud_window.showWidget("rgb_point_cloud", cloud_widget);
            cv::Affine3d cam_pose = cv::viz::makeCameraPose(cv::Vec3d(0, 0, -2), cv::Vec3d(0, 0, 0), cv::Vec3d(0, -1, 0));
            point_cloud_window.setViewerPose(cam_pose);
            point_cloud_window.spinOnce(1, true);
        }
#else
        // Alternative: Create RGB depth projection
        if (show_depth_projection) {
            CreateRgbDepthProjection(points, point_count, "RGB Point Cloud Projection");
        }
#endif
    }
    
    void DisplayRgbIrPointCloud(std::shared_ptr<StreamFrame> frame) {
        if (!frame->data || frame->size == 0) return;
        
        // Cast data to RGB+IR point cloud format
        deptrum::PointXyzRgbIr<float>* points = (deptrum::PointXyzRgbIr<float>*)frame->data.get();
        size_t point_count = frame->size / sizeof(deptrum::PointXyzRgbIr<float>);
        
        std::cout << "Processing RGB+IR Point Cloud: " << point_count << " points" << std::endl;
        
#ifdef HAVE_OPENCV_VIZ
        // Use OpenCV viz if available
        std::vector<cv::Point3f> cv_points;
        std::vector<cv::Vec3b> cv_colors;
        cv_points.reserve(point_count);
        cv_colors.reserve(point_count);
        
        for (size_t i = 0; i < point_count; i++) {
            if (std::isfinite(points[i].x) && std::isfinite(points[i].y) && std::isfinite(points[i].z) &&
                points[i].z > 0.1f && points[i].z < 5000.0f) {
                cv_points.emplace_back(points[i].x / 1000.0f, points[i].y / 1000.0f, points[i].z / 1000.0f);
                
                // Mix RGB and IR data for visualization
                uint8_t mixed_r = (points[i].r + points[i].ir) / 2;
                uint8_t mixed_g = (points[i].g + points[i].ir) / 2;
                uint8_t mixed_b = (points[i].b + points[i].ir) / 2;
                cv_colors.emplace_back(mixed_b, mixed_g, mixed_r);
            }
        }
        
        if (!cv_points.empty()) {
            cv::viz::WCloud cloud_widget(cv_points, cv_colors);
            point_cloud_window.showWidget("rgbir_point_cloud", cloud_widget);
            cv::Affine3d cam_pose = cv::viz::makeCameraPose(cv::Vec3d(0, 0, -2), cv::Vec3d(0, 0, 0), cv::Vec3d(0, -1, 0));
            point_cloud_window.setViewerPose(cam_pose);
            point_cloud_window.spinOnce(1, true);
        }
#else
        // Alternative: Create RGB+IR depth projection
        if (show_depth_projection) {
            CreateRgbIrDepthProjection(points, point_count, "RGB+IR Point Cloud Projection");
        }
#endif
    }
    
    void DisplayRgbFrame(std::shared_ptr<StreamFrame> frame, const std::string& title) {
        if (!frame->data || frame->size == 0) return;
        
        cv::Mat rgb_display;
        
        if (device_type_ == "Aurora912" || device_type_ == "Aurora930" || device_type_ == "Aurora932") {
            int rgb_height = frame->rows;
            int rgb_width = frame->cols;
            cv::Mat yuv_mat(rgb_height * 1.5f, rgb_width, CV_8UC1, frame->data.get());
            rgb_display = cv::Mat(rgb_height, rgb_width, CV_8UC3);
            cv::cvtColor(yuv_mat, rgb_display, cv::COLOR_YUV2BGR_NV12);
        } else {
            cv::Mat rgb_mat(frame->rows, frame->cols, CV_8UC3, frame->data.get());
            rgb_mat.copyTo(rgb_display);
        }
        
        if (!rgb_display.empty()) {
            cv::imshow(title, rgb_display);
        }
    }
    
    void DisplayDepthFrame(std::shared_ptr<StreamFrame> frame, const std::string& title) {
        if (!frame->data || frame->size == 0) return;
        
        cv::Mat color_image(frame->rows, frame->cols, CV_8UC3);
        uint16_t* depth_data = reinterpret_cast<uint16_t*>(frame->data.get());
        
        // Create colorized depth image
        for (int i = 0; i < frame->rows * frame->cols; i++) {
            uint16_t depth_value = depth_data[i];
            
            if (depth_value == 0) {
                color_image.at<cv::Vec3b>(i) = cv::Vec3b(0, 0, 0);  // Black for invalid depth
            } else {
                // Map depth to color (rainbow colormap)
                float normalized_depth = std::min(1.0f, depth_value / 3000.0f);
                cv::Mat depth_mat(1, 1, CV_32F, &normalized_depth);
                cv::Mat color_mat;
                cv::applyColorMap(depth_mat * 255, color_mat, cv::COLORMAP_JET);
                color_image.at<cv::Vec3b>(i) = color_mat.at<cv::Vec3b>(0, 0);
            }
        }
        
        cv::imshow(title, color_image);
    }
    
    void DisplayIrFrame(std::shared_ptr<StreamFrame> frame, const std::string& title) {
        if (!frame->data || frame->size == 0) return;
        
        cv::Mat ir_mat(frame->rows, frame->cols, CV_16UC1, frame->data.get());
        cv::Mat ir_display;
        ir_mat.convertTo(ir_display, CV_8UC1, 255.0 / 65535.0);
        cv::imshow(title, ir_display);
    }
    
    void SavePointCloudFrame(std::shared_ptr<StreamFrame> frame) {
        std::stringstream ss;
        ss << "scan_" << std::setfill('0') << std::setw(6) << frame_count << "_" << frame->timestamp;
        
        if (frame->frame_type == FrameType::kPointCloudFrame) {
            SavePointXyz(ss.str() + ".ply", (char*)frame->data.get(), frame->size);
        } else if (frame->frame_type == FrameType::kRgbdPointCloudFrame) {
            SavePointXyzRgb(ss.str() + "_rgb.ply", (char*)frame->data.get(), frame->size);
        } else if (frame->frame_type == FrameType::kRgbdIrPointCloudFrame) {
            SavePointXyzRgbIr(ss.str() + "_rgbir.ply", (char*)frame->data.get(), frame->size);
        }
        
        frame_count++;
    }
    
#ifndef HAVE_OPENCV_VIZ
    // Alternative visualization methods when viz module is not available
    void CreateDepthProjection(deptrum::PointXyz<float>* points, size_t point_count, const std::string& title) {
        // Create a top-down depth projection
        cv::Mat projection = cv::Mat::zeros(600, 800, CV_8UC3);
        
        // Find bounds
        float min_x = FLT_MAX, max_x = -FLT_MAX;
        float min_y = FLT_MAX, max_y = -FLT_MAX;
        float min_z = FLT_MAX, max_z = -FLT_MAX;
        
        for (size_t i = 0; i < point_count; i++) {
            if (std::isfinite(points[i].x) && std::isfinite(points[i].y) && std::isfinite(points[i].z) &&
                points[i].z > 0.1f && points[i].z < 5000.0f) {
                min_x = std::min(min_x, points[i].x);
                max_x = std::max(max_x, points[i].x);
                min_y = std::min(min_y, points[i].y);
                max_y = std::max(max_y, points[i].y);
                min_z = std::min(min_z, points[i].z);
                max_z = std::max(max_z, points[i].z);
            }
        }
        
        if (max_x > min_x && max_y > min_y && max_z > min_z) {
            for (size_t i = 0; i < point_count; i++) {
                if (std::isfinite(points[i].x) && std::isfinite(points[i].y) && std::isfinite(points[i].z) &&
                    points[i].z > 0.1f && points[i].z < 5000.0f) {
                    
                    int px = (int)((points[i].x - min_x) / (max_x - min_x) * (projection.cols - 1));
                    int py = (int)((points[i].y - min_y) / (max_y - min_y) * (projection.rows - 1));
                    
                    if (px >= 0 && px < projection.cols && py >= 0 && py < projection.rows) {
                        float depth_norm = (points[i].z - min_z) / (max_z - min_z);
                        cv::Vec3b color = cv::Vec3b(
                            (uint8_t)(255 * depth_norm),
                            (uint8_t)(255 * (1.0f - depth_norm)),
                            128
                        );
                        projection.at<cv::Vec3b>(py, px) = color;
                    }
                }
            }
        }
        
        cv::imshow(title, projection);
    }
    
    void CreateRgbDepthProjection(deptrum::PointXyzRgb<float>* points, size_t point_count, const std::string& title) {
        cv::Mat projection = cv::Mat::zeros(600, 800, CV_8UC3);
        
        // Find bounds
        float min_x = FLT_MAX, max_x = -FLT_MAX;
        float min_y = FLT_MAX, max_y = -FLT_MAX;
        
        for (size_t i = 0; i < point_count; i++) {
            if (std::isfinite(points[i].x) && std::isfinite(points[i].y) && std::isfinite(points[i].z) &&
                points[i].z > 0.1f && points[i].z < 5000.0f) {
                min_x = std::min(min_x, points[i].x);
                max_x = std::max(max_x, points[i].x);
                min_y = std::min(min_y, points[i].y);
                max_y = std::max(max_y, points[i].y);
            }
        }
        
        if (max_x > min_x && max_y > min_y) {
            for (size_t i = 0; i < point_count; i++) {
                if (std::isfinite(points[i].x) && std::isfinite(points[i].y) && std::isfinite(points[i].z) &&
                    points[i].z > 0.1f && points[i].z < 5000.0f) {
                    
                    int px = (int)((points[i].x - min_x) / (max_x - min_x) * (projection.cols - 1));
                    int py = (int)((points[i].y - min_y) / (max_y - min_y) * (projection.rows - 1));
                    
                    if (px >= 0 && px < projection.cols && py >= 0 && py < projection.rows) {
                        cv::Vec3b color = cv::Vec3b(points[i].b, points[i].g, points[i].r);
                        projection.at<cv::Vec3b>(py, px) = color;
                    }
                }
            }
        }
        
        cv::imshow(title, projection);
    }
    
    void CreateRgbIrDepthProjection(deptrum::PointXyzRgbIr<float>* points, size_t point_count, const std::string& title) {
        cv::Mat projection = cv::Mat::zeros(600, 800, CV_8UC3);
        
        // Find bounds
        float min_x = FLT_MAX, max_x = -FLT_MAX;
        float min_y = FLT_MAX, max_y = -FLT_MAX;
        
        for (size_t i = 0; i < point_count; i++) {
            if (std::isfinite(points[i].x) && std::isfinite(points[i].y) && std::isfinite(points[i].z) &&
                points[i].z > 0.1f && points[i].z < 5000.0f) {
                min_x = std::min(min_x, points[i].x);
                max_x = std::max(max_x, points[i].x);
                min_y = std::min(min_y, points[i].y);
                max_y = std::max(max_y, points[i].y);
            }
        }
        
        if (max_x > min_x && max_y > min_y) {
            for (size_t i = 0; i < point_count; i++) {
                if (std::isfinite(points[i].x) && std::isfinite(points[i].y) && std::isfinite(points[i].z) &&
                    points[i].z > 0.1f && points[i].z < 5000.0f) {
                    
                    int px = (int)((points[i].x - min_x) / (max_x - min_x) * (projection.cols - 1));
                    int py = (int)((points[i].y - min_y) / (max_y - min_y) * (projection.rows - 1));
                    
                    if (px >= 0 && px < projection.cols && py >= 0 && py < projection.rows) {
                        uint8_t mixed_r = (points[i].r + points[i].ir) / 2;
                        uint8_t mixed_g = (points[i].g + points[i].ir) / 2;
                        uint8_t mixed_b = (points[i].b + points[i].ir) / 2;
                        cv::Vec3b color = cv::Vec3b(mixed_b, mixed_g, mixed_r);
                        projection.at<cv::Vec3b>(py, px) = color;
                    }
                }
            }
        }
        
        cv::imshow(title, projection);
    }
#endif
};

void PrintUsage() {
    std::cout << "\n=== Aurora 3D Scanner Controls ===" << std::endl;
    std::cout << "Key Controls:" << std::endl;
    std::cout << "  ESC/q     - Quit application" << std::endl;
    std::cout << "  s         - Save current point cloud" << std::endl;
    std::cout << "  t         - Toggle auto-save point clouds" << std::endl;
    std::cout << "  f         - Toggle FPS display" << std::endl;
    std::cout << "  d         - Toggle depth projection display" << std::endl;
#ifdef HAVE_OPENCV_VIZ
    std::cout << "  v         - Toggle 3D viewer" << std::endl;
    std::cout << "  r         - Reset 3D viewer pose" << std::endl;
#endif
    std::cout << "======================================\n" << std::endl;
}

void ChooseStreamType(std::shared_ptr<Device> device, std::vector<StreamType>& stream_types_vector) {
    std::cout << std::endl;
    std::cout << "---------------------- Choose Stream Type ----------------------" << std::endl;
    
    int num = 0;
    std::vector<StreamType> device_support_streamtype_vec;
    device->GetSupportedStreamType(device_support_streamtype_vec);

    for (auto stream_type : device_support_streamtype_vec) {
        std::cout << "[" << num++ << "]: " << stream_type_map[stream_type] << std::endl;
    }

    std::cout << "\nFor 3D scanning, recommended options:" << std::endl;
    std::cout << "- RgbdPointCloud (RGB + Point Cloud)" << std::endl;
    std::cout << "- RgbdIrPointCloud (RGB + IR + Point Cloud)" << std::endl;
    std::cout << "- PointCloud (Point Cloud only)" << std::endl;

    std::string input;
    while (true) {
        std::cout << "\nEnter your choice (0-" << (num-1) << "): ";
        std::cin >> input;
        
        try {
            int index = std::stoi(input);
            if (index >= 0 && index < device_support_streamtype_vec.size()) {
                stream_types_vector.push_back(device_support_streamtype_vec[index]);
                break;
            } else {
                std::cout << "Invalid choice. Please try again." << std::endl;
            }
        } catch (...) {
            std::cout << "Invalid input. Please enter a number." << std::endl;
        }
    }
}

int StreamProcess(std::shared_ptr<Device> device,
                  const std::vector<StreamType>& stream_type,
                  long frame_loop_times,
                  std::string device_name) {
    
    Stream* stream = nullptr;
    
    int ret = device->CreateStream(stream, stream_type);
    CHECK_SDK_RETURN_VALUE(ret);

    ret = stream->Start();
    CHECK_SDK_RETURN_VALUE(ret);

    StreamFrames frames;
    FrameRateHelper frame_rate_helper;
    long cnt = 0;

    Enhanced3DViewerHelper viewer_helper(device_name);
    
    PrintUsage();

    while (is_running) {
        if (frame_loop_times != -1) {
            if (frame_loop_times == 0) break;
            frame_loop_times--;
        }
        
        ret = stream->GetFrames(frames, 2000);
        CHECK_GET_FRAMES(ret);
        
        frame_rate_helper.RecordTimestamp();
        
        if (is_print_fps && 0 == cnt++ % 10) {
            std::cout << "FPS: " << frame_rate_helper.GetFrameRate() << std::endl;
        }

        // Display frames using enhanced viewer
        viewer_helper.ShowFrame(frames, g_camera_param);
        
        // Handle keyboard input
        int key = cv::waitKey(1) & 0xFF;
        switch (key) {
            case 27:  // ESC
            case 'q':
            case 'Q':
                is_running = false;
                break;
            case 's':
            case 'S':
                std::cout << "Saving current point cloud..." << std::endl;
                save_point_clouds = true;
                break;
            case 't':
            case 'T':
                save_point_clouds = !save_point_clouds;
                std::cout << "Auto-save point clouds: " << (save_point_clouds ? "ON" : "OFF") << std::endl;
                break;
            case 'f':
            case 'F':
                is_print_fps = !is_print_fps;
                std::cout << "FPS display: " << (is_print_fps ? "ON" : "OFF") << std::endl;
                break;
            case 'd':
            case 'D':
                show_depth_projection = !show_depth_projection;
                std::cout << "Depth projection: " << (show_depth_projection ? "ON" : "OFF") << std::endl;
                break;
#ifdef HAVE_OPENCV_VIZ
            case 'v':
            case 'V':
                show_3d_viewer = !show_3d_viewer;
                std::cout << "3D viewer: " << (show_3d_viewer ? "ON" : "OFF") << std::endl;
                break;
            case 'r':
            case 'R':
                if (show_3d_viewer) {
                    cv::Affine3d cam_pose = cv::viz::makeCameraPose(cv::Vec3d(0, 0, -2), cv::Vec3d(0, 0, 0), cv::Vec3d(0, -1, 0));
                    point_cloud_window.setViewerPose(cam_pose);
                    std::cout << "3D viewer pose reset" << std::endl;
                }
                break;
#endif
        }
    }

    stream->Stop();
    device->DestroyStream(stream);
    
    return 0;
}

int main() {
    std::cout << "=== Aurora 3D Scanner Starting ===" << std::endl;
    
#ifdef HAVE_OPENCV_VIZ
    std::cout << "OpenCV viz module available - Full 3D visualization enabled" << std::endl;
#else
    std::cout << "OpenCV viz module not available - Using 2D projection mode" << std::endl;
#endif
    
    PrintUsage();
    
    // Initialize device
    std::shared_ptr<Device> device;
    
    // Try to create device based on available device types
#ifdef DEVICE_TYPE_AURORA900
    device = std::make_shared<Aurora900>();
    std::cout << "Using Aurora900 device" << std::endl;
#elif defined(DEVICE_TYPE_AURORA700)
    device = std::make_shared<Aurora700>();
    std::cout << "Using Aurora700 device" << std::endl;
#elif defined(DEVICE_TYPE_AURORA500)
    device = std::make_shared<Aurora500>();
    std::cout << "Using Aurora500 device" << std::endl;
#elif defined(DEVICE_TYPE_AURORA300)
    device = std::make_shared<Aurora300>();
    std::cout << "Using Aurora300 device" << std::endl;
#elif defined(DEVICE_TYPE_NEBULA200)
    device = std::make_shared<Nebula200>();
    std::cout << "Using Nebula200 device" << std::endl;
#elif defined(DEVICE_TYPE_NEBULA100)
    device = std::make_shared<Nebula100>();
    std::cout << "Using Nebula100 device" << std::endl;
#elif defined(DEVICE_TYPE_STELLAR400)
    device = std::make_shared<Stellar400>();
    std::cout << "Using Stellar400 device" << std::endl;
#elif defined(DEVICE_TYPE_STELLAR200)
    device = std::make_shared<Stellar200>();
    std::cout << "Using Stellar200 device" << std::endl;
#else
    std::cerr << "ERROR: No supported device type found!" << std::endl;
    std::cerr << "Please ensure you have compiled with a supported device type:" << std::endl;
    std::cerr << "  -DDEVICE_TYPE_AURORA900" << std::endl;
    std::cerr << "  -DDEVICE_TYPE_AURORA700" << std::endl;
    std::cerr << "  -DDEVICE_TYPE_AURORA500" << std::endl;
    std::cerr << "  etc." << std::endl;
    return -1;
#endif

    if (!device) {
        std::cerr << "Failed to create device!" << std::endl;
        return -1;
    }

    std::cout << "Initializing device..." << std::endl;
    int ret = device->Init();
    if (ret != 0) {
        std::cerr << "Device initialization failed with error: " << ret << std::endl;
        return ret;
    }

    std::cout << "Opening device..." << std::endl;
    ret = device->Open();
    if (ret != 0) {
        std::cerr << "Device open failed with error: " << ret << std::endl;
        return ret;
    }

    // Get camera parameters
    ret = device->GetCameraParam(g_camera_param);
    if (ret != 0) {
        std::cerr << "Failed to get camera parameters with error: " << ret << std::endl;
        device->Close();
        return ret;
    }

    std::string device_name = device->GetDeviceInfo().device_name;
    std::cout << "Connected to device: " << device_name << std::endl;

    // Choose stream type
    std::vector<StreamType> stream_types;
    ChooseStreamType(device, stream_types);

    if (stream_types.empty()) {
        std::cerr << "No stream type selected!" << std::endl;
        device->Close();
        return -1;
    }

    // Start streaming and processing
    std::cout << "Starting 3D scanner..." << std::endl;
    ret = StreamProcess(device, stream_types, -1, device_name);
    
    std::cout << "Closing device..." << std::endl;
    device->Close();
    
    std::cout << "3D Scanner terminated successfully." << std::endl;
    return ret;
}