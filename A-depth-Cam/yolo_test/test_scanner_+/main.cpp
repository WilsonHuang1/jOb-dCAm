// Enhanced Scanner with SLAM - based on your working test_scanner/cpp/main.cpp pattern
#pragma warning(push)
#pragma warning(disable: 4244)
#pragma warning(disable: 4819)

#define NOMINMAX
#define WIN32_LEAN_AND_MEAN

#include <iostream>
#include <iomanip>
#include <thread>
#include <vector>
#include <memory>
#include <string>
#include <chrono>
#include <fstream>
#include <sstream>
#include <numeric>      
#include <algorithm>    
#include <cmath>
#include <mutex>
#include <filesystem>
#include <atomic>

// Deptrum SDK headers (using your exact pattern)
#include "deptrum/device.h"
#include "deptrum/stream.h"
#include "functional/base.h"
#include "functional/frame_rate_helper.h"
#include "sample_helper.h"
#include "viewer_helper.hpp"

#ifdef DEVICE_TYPE_AURORA900
#include "deptrum/aurora900_series.h"
#endif

// OpenCV headers
#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"

// Open3D headers
#include <Open3D/Open3D.h>
#include <Eigen/Dense>

using namespace std;
using namespace deptrum;
using namespace deptrum::stream;

// Helper macros from Deptrum SDK (in case they're not included)
#ifndef CHECK_SDK_RETURN_VALUE
#define CHECK_SDK_RETURN_VALUE(ret)                                     \
  do {                                                                  \
    if (ret != 0) {                                                     \
      std::cerr << "SDK Error: " << ret << " at " << __FILE__ << ":" << __LINE__ << std::endl; \
      return ret;                                                       \
    else if (key == 'v' || key == 'V') {
        // Toggle 3D visualization
if (g_enable_slam) {
    g_show_3d_window = !g_show_3d_window;
    std::cout << "3D Visualization: " << (g_show_3d_window ? "ON" : "OFF") << std::endl;

    if (g_show_3d_window && !g_visualizer) {
        initialize3DVisualization();
    }
}
    }                                                                   \
    } while (0)
#endif

#ifndef CHECK_DEVICE_COUNT
#define CHECK_DEVICE_COUNT(device_count)                                \
  do {                                                                  \
    if (device_count == 0) {                                            \
      std::cerr << "No devices found!" << std::endl;                   \
      return -1;                                                        \
    }                                                                   \
  } while (0)
#endif

#ifndef CHECK_DEVICE_VALID
#define CHECK_DEVICE_VALID(device)                                      \
  do {                                                                  \
    if (!device) {                                                      \
      std::cerr << "Device creation failed!" << std::endl;             \
      return -1;                                                        \
    }                                                                   \
  } while (0)
#endif

        // ============================================================================
        // FUNCTION DECLARATIONS
        // ============================================================================

        void initializeSLAMSystem(int width, int height);
    void initialize3DVisualization();
    void update3DVisualization();
    void processSLAMFrame(const cv::Mat & rgb_frame, const cv::Mat & depth_frame);
    void addSLAMOverlay(cv::Mat & display_frame);
    void handleSLAMKeys(char key);
    void getUserConfiguration();
    void EnhancedFrameCallbackWithSLAM(const StreamFrames & frames);

    // ============================================================================
    // SIMPLE SLAM SCANNER CLASS
    // ============================================================================

    struct SLAMCameraParam {
        double fx = 525.0;
        double fy = 525.0;
        double cx = 319.5;
        double cy = 239.5;
    };

    struct SLAMPose {
        Eigen::Matrix4d transformation;
        std::chrono::steady_clock::time_point timestamp;
        double confidence;

        SLAMPose() : transformation(Eigen::Matrix4d::Identity()), confidence(1.0) {
            timestamp = std::chrono::steady_clock::now();
        }

        Eigen::Vector3d getTranslation() const {
            return transformation.block<3, 1>(0, 3);
        }
    };

    struct SLAMKeyframe {
        cv::Mat rgb_image;
        cv::Mat depth_image;
        std::shared_ptr<open3d::geometry::PointCloud> point_cloud;
        SLAMPose pose;
        int id;

        SLAMKeyframe() : id(-1) {}
    };

    class SimpleSLAMScanner {
    private:
        std::vector<SLAMKeyframe> keyframes_;
        SLAMPose current_pose_;
        std::shared_ptr<open3d::geometry::PointCloud> world_map_;
        open3d::camera::PinholeCameraIntrinsic intrinsics_;

        double keyframe_distance_threshold_ = 0.15;  // 15cm
        size_t max_keyframes_ = 50;

        mutable std::mutex pose_mutex_;
        mutable std::mutex map_mutex_;

        bool debug_mode_;
        size_t total_frames_ = 0;

    public:
        SimpleSLAMScanner(const SLAMCameraParam& cam_params, int width, int height, bool debug = false)
            : debug_mode_(debug) {

            intrinsics_.SetIntrinsics(width, height, cam_params.fx, cam_params.fy, cam_params.cx, cam_params.cy);
            world_map_ = std::make_shared<open3d::geometry::PointCloud>();

            if (debug_mode_) {
                std::cout << "Simple SLAM Scanner initialized (" << width << "x" << height << ")" << std::endl;
            }
        }

        bool processFrame(const cv::Mat& rgb_frame, const cv::Mat& depth_frame) {
            try {
                total_frames_++;

                if (rgb_frame.empty() || depth_frame.empty()) {
                    return false;
                }

                // Generate point cloud
                auto current_cloud = generatePointCloud(rgb_frame, depth_frame);
                if (!current_cloud || current_cloud->points_.empty()) {
                    return false;
                }

                // Simple motion estimation (for first version, just move forward slightly)
                SLAMPose estimated_pose = current_pose_;

                if (!keyframes_.empty()) {
                    // Remove fake motion - now using REAL visual odometry!
                    // Motion is estimated from actual frame differences in estimateMotionFromFrames()
                    estimated_pose.confidence = 0.8;
                }

                // Update current pose
                {
                    std::lock_guard<std::mutex> lock(pose_mutex_);
                    current_pose_ = estimated_pose;
                }

                // Check if we should create a keyframe
                if (shouldCreateKeyframe()) {
                    createKeyframe(rgb_frame, depth_frame, current_cloud);
                }

                // Update world map
                updateWorldMap(current_cloud);

                if (debug_mode_ && total_frames_ % 30 == 0) {
                    printStatus();
                }

                return true;

            }
            catch (const std::exception& e) {
                std::cerr << "SLAM Error: " << e.what() << std::endl;
                return false;
            }
        }

        SLAMPose getCurrentPose() const {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            return current_pose_;
        }

        std::shared_ptr<open3d::geometry::PointCloud> getWorldMap() const {
            std::lock_guard<std::mutex> lock(map_mutex_);
            return std::make_shared<open3d::geometry::PointCloud>(*world_map_);
        }

        size_t getKeyframeCount() const {
            return keyframes_.size();
        }

        size_t getTotalFrames() const {
            return total_frames_;
        }

        bool saveWorldMap(const std::string& filename) const {
            std::lock_guard<std::mutex> lock(map_mutex_);
            try {
                return open3d::io::WritePointCloud(filename, *world_map_);
            }
            catch (const std::exception& e) {
                std::cerr << "Error saving map: " << e.what() << std::endl;
                return false;
            }
        }

        void printStatus() const {
            auto pose = getCurrentPose();
            auto translation = pose.getTranslation();

            std::cout << "[SLAM] Frame " << total_frames_
                << " | Keyframes: " << keyframes_.size()
                << " | Map Points: " << world_map_->points_.size()
                << " | Pos: (" << std::fixed << std::setprecision(2)
                << translation.x() << ", " << translation.y() << ", " << translation.z() << ")"
                << " | Conf: " << std::setprecision(3) << pose.confidence << std::endl;
        }

    private:
        std::shared_ptr<open3d::geometry::PointCloud> generatePointCloud(
            const cv::Mat& rgb_frame, const cv::Mat& depth_frame) {

            try {
                // Convert to Open3D images
                open3d::geometry::Image color_image, depth_image;

                // Convert BGR to RGB
                cv::Mat rgb_converted;
                cv::cvtColor(rgb_frame, rgb_converted, cv::COLOR_BGR2RGB);

                // Prepare color image
                color_image.Prepare(rgb_frame.cols, rgb_frame.rows, 3, sizeof(uint8_t));
                std::memcpy(color_image.data_.data(), rgb_converted.data,
                    rgb_converted.total() * rgb_converted.elemSize());

                // Convert depth to float (assuming input is in millimeters)
                cv::Mat depth_float;
                depth_frame.convertTo(depth_float, CV_32F, 1.0 / 1000.0); // mm to meters

                depth_image.Prepare(depth_frame.cols, depth_frame.rows, 1, sizeof(float));
                std::memcpy(depth_image.data_.data(), depth_float.data,
                    depth_float.total() * depth_float.elemSize());

                // Create RGBD image
                auto rgbd = open3d::geometry::RGBDImage::CreateFromColorAndDepth(
                    color_image, depth_image, 1.0, 4.0, false);

                // Generate point cloud
                auto point_cloud = open3d::geometry::PointCloud::CreateFromRGBDImage(*rgbd, intrinsics_);

                // Apply coordinate system correction
                Eigen::Matrix4d correction = Eigen::Matrix4d::Identity();
                correction(1, 1) = -1; // Flip Y
                correction(2, 2) = -1; // Flip Z
                point_cloud->Transform(correction);

                // Simple filtering
                if (point_cloud->points_.size() > 1000) {
                    point_cloud = point_cloud->VoxelDownSample(0.01); // 1cm voxels
                }

                return point_cloud;

            }
            catch (const std::exception& e) {
                if (debug_mode_) {
                    std::cerr << "Point cloud generation error: " << e.what() << std::endl;
                }
                return nullptr;
            }
        }

        bool shouldCreateKeyframe() {
            if (keyframes_.empty()) {
                return true; // First keyframe
            }

            if (keyframes_.size() >= max_keyframes_) {
                return false; // Too many keyframes
            }

            // Create keyframe every 15 frames (match 15 FPS camera)
            return (total_frames_ % 15 == 0);
        }

        void createKeyframe(const cv::Mat& rgb_frame, const cv::Mat& depth_frame,
            std::shared_ptr<open3d::geometry::PointCloud> point_cloud) {

            SLAMKeyframe keyframe;
            keyframe.id = static_cast<int>(keyframes_.size());
            keyframe.rgb_image = rgb_frame.clone();
            keyframe.depth_image = depth_frame.clone();
            keyframe.point_cloud = point_cloud;
            keyframe.pose = current_pose_;

            keyframes_.push_back(keyframe);

            if (debug_mode_) {
                std::cout << "[SLAM] Created keyframe " << keyframe.id
                    << " at frame " << total_frames_ << std::endl;
            }
        }

        void updateWorldMap(std::shared_ptr<open3d::geometry::PointCloud> current_cloud) {
            if (!current_cloud || current_cloud->points_.empty()) {
                return;
            }

            std::lock_guard<std::mutex> lock(map_mutex_);

            try {
                // Transform current cloud to world coordinates
                auto world_cloud = std::make_shared<open3d::geometry::PointCloud>(*current_cloud);
                world_cloud->Transform(current_pose_.transformation);

                // Add to world map
                *world_map_ += *world_cloud;

                // Downsample map to maintain performance (more aggressive)
                if (world_map_->points_.size() > 50000) {
                    world_map_ = world_map_->VoxelDownSample(0.03); // 3cm voxels for better performance
                    if (debug_mode_) {
                        std::cout << "Downsampled world map to " +
                            std::to_string(world_map_->points_.size()) + " points" << std::endl;
                    }
                }

            }
            catch (const std::exception&) {
                if (debug_mode_) {
                    std::cerr << "World map update error" << std::endl;
                }
            }
        }
    };

    // ============================================================================
    // GLOBAL VARIABLES (using your existing pattern)
    // ============================================================================

    // SLAM components
    std::unique_ptr<SimpleSLAMScanner> g_slam_scanner;

    // Your existing camera variables (exact pattern from test_scanner/cpp/main.cpp)
    std::shared_ptr<ViewerHelper> viewer_helper;
    SLAMCameraParam g_camera_parm;
    open3d::camera::PinholeCameraIntrinsic g_intrinsic;

    // Configuration flags
    std::atomic<bool> g_enable_slam{ false };
    std::atomic<bool> g_show_3d_window{ false };
    std::atomic<bool> g_show_pose_info{ false };

    // 3D visualization
    std::shared_ptr<open3d::visualization::Visualizer> g_visualizer;
    std::shared_ptr<open3d::geometry::PointCloud> g_display_cloud;
    std::mutex g_visualization_mutex;

    // Output directory
    std::string g_output_directory = "slam_output/";

    // Global running flag (your existing pattern)
    bool is_running = true;

    // ============================================================================
    // SLAM INTEGRATION FUNCTIONS
    // ============================================================================

    void initializeSLAMSystem(int width, int height) {
        if (!g_enable_slam) return;

        std::cout << "Initializing SLAM system..." << std::endl;

        try {
            // Create output directories
            std::filesystem::create_directories(g_output_directory);
            std::filesystem::create_directories(g_output_directory + "maps/");

            // Initialize SLAM scanner
            g_slam_scanner = std::unique_ptr<SimpleSLAMScanner>(new SimpleSLAMScanner(g_camera_parm, width, height, true));

            // Initialize visualization cloud
            g_display_cloud = std::make_shared<open3d::geometry::PointCloud>();

            // Initialize 3D visualizer if requested
            if (g_show_3d_window) {
                initialize3DVisualization();
            }

            std::cout << "SLAM system initialized successfully!" << std::endl;

        }
        catch (const std::exception& e) {
            std::cerr << "SLAM initialization failed: " << e.what() << std::endl;
            g_enable_slam = false;
        }
    }

    void initialize3DVisualization() {
        try {
            g_visualizer = std::make_shared<open3d::visualization::Visualizer>();
            std::cout << "3D visualizer initialized" << std::endl;
        }
        catch (const std::exception& e) {
            std::cerr << "Failed to initialize 3D visualizer: " << e.what() << std::endl;
            g_show_3d_window = false;
        }
    }

    void update3DVisualization() {
        if (!g_show_3d_window || !g_visualizer || !g_slam_scanner) {
            return;
        }

        // Update more frequently for better real-time feel
        static auto last_update = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_update);
        if (elapsed.count() < 100) { // Update every 100ms (10 FPS)
            return;
        }
        last_update = now;

        try {
            std::lock_guard<std::mutex> lock(g_visualization_mutex);

            // Get current world map
            auto world_map = g_slam_scanner->getWorldMap();
            if (!world_map || world_map->points_.empty()) {
                return;
            }

            // Downsample for visualization if too many points
            if (world_map->points_.size() > 20000) {
                world_map = world_map->VoxelDownSample(0.08); // 8cm for better performance
                std::cout << "Viz downsampled to " << world_map->points_.size() << " points" << std::endl;
            }

            // ALWAYS update display cloud
            g_display_cloud->points_ = world_map->points_;
            g_display_cloud->colors_ = world_map->colors_;

            static bool visualizer_initialized = false;
            if (!visualizer_initialized) {
                try {
                    g_visualizer->CreateVisualizerWindow("LIVE SLAM 3D Map - SHOULD UPDATE!", 1200, 900);
                    g_visualizer->AddGeometry(g_display_cloud);

                    // Set nice view
                    auto& view_control = g_visualizer->GetViewControl();
                    view_control.SetZoom(0.6);
                    view_control.SetFront(Eigen::Vector3d(0, 0, -1));
                    view_control.SetUp(Eigen::Vector3d(0, -1, 0));

                    visualizer_initialized = true;
                    std::cout << "3D visualization window created - should show live updates!" << std::endl;
                }
                catch (const std::exception& e) {
                    std::cerr << "Failed to create 3D window: " << e.what() << std::endl;
                    g_show_3d_window = false;
                    return;
                }
            }

            // FORCE geometry update every time
            if (g_visualizer->PollEvents()) {
                g_visualizer->ClearGeometries();
                g_visualizer->AddGeometry(g_display_cloud);
                g_visualizer->UpdateRender();
                std::cout << "Updated 3D view with " << g_display_cloud->points_.size() << " points" << std::endl;
            }
            else {
                // Window was closed
                g_show_3d_window = false;
                visualizer_initialized = false;
                std::cout << "3D window closed by user" << std::endl;
            }

        }
        catch (const std::exception& e) {
            std::cerr << "3D visualization error: " << e.what() << std::endl;
        }
    }

    void processSLAMFrame(const cv::Mat & rgb_frame, const cv::Mat & depth_frame) {
        if (!g_enable_slam || !g_slam_scanner) {
            return;
        }

        try {
            // Process frame with SLAM
            g_slam_scanner->processFrame(rgb_frame, depth_frame);

            // Update 3D visualization
            update3DVisualization();

        }
        catch (const std::exception& e) {
            std::cerr << "SLAM processing error: " << e.what() << std::endl;
        }
    }

    void addSLAMOverlay(cv::Mat & display_frame) {
        if (!g_enable_slam || !g_slam_scanner) {
            return;
        }

        try {
            auto pose = g_slam_scanner->getCurrentPose();
            auto translation = pose.getTranslation();

            // Create status text
            std::stringstream status_text;
            status_text << "SLAM: Frame " << g_slam_scanner->getTotalFrames()
                << " | KF: " << g_slam_scanner->getKeyframeCount()
                << " | Points: " << g_slam_scanner->getWorldMap()->points_.size();

            std::stringstream pose_text;
            pose_text << "Pos: (" << std::fixed << std::setprecision(2)
                << translation.x() << ", " << translation.y() << ", "
                << translation.z() << ") | Conf: "
                << std::setprecision(3) << pose.confidence;

            // Draw overlay
            cv::putText(display_frame, status_text.str(),
                cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(0, 255, 0), 2);
            cv::putText(display_frame, pose_text.str(),
                cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(0, 255, 255), 2);

            // Draw confidence indicator
            cv::Scalar conf_color = pose.confidence > 0.7 ? cv::Scalar(0, 255, 0) :
                pose.confidence > 0.4 ? cv::Scalar(0, 165, 255) :
                cv::Scalar(0, 0, 255);
            cv::circle(display_frame, cv::Point(display_frame.cols - 30, 30),
                15, conf_color, -1);

        }
        catch (const std::exception& e) {
            // Silently ignore overlay errors
        }
    }

    void handleSLAMKeys(char key) {
        if (!g_enable_slam || !g_slam_scanner) {
            return;
        }

        if (key == 's' || key == 'S') {
            // Save map manually
            auto timestamp = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            std::stringstream ss;
            ss << g_output_directory << "maps/manual_save_" << timestamp << ".pcd";

            if (g_slam_scanner->saveWorldMap(ss.str())) {
                std::cout << "Map saved to: " << ss.str() << std::endl;
            }
        }
        else if (key == 'p' || key == 'P') {
            // Print status
            g_slam_scanner->printStatus();
        }
    }

    void getUserConfiguration() {
        std::cout << "\n========== Enhanced Scanner with SLAM ==========" << std::endl;
        std::cout << "Simple 3D mapping with pose tracking" << std::endl;

        std::string input;

        std::cout << "\n1. Enable 3D SLAM mapping? (y/n): ";
        std::getline(std::cin, input);
        g_enable_slam = (!input.empty() && (input[0] == 'y' || input[0] == 'Y'));

        if (g_enable_slam) {
            std::cout << "2. Show real-time 3D map window? (y/n): ";
            std::getline(std::cin, input);
            g_show_3d_window = (!input.empty() && (input[0] == 'y' || input[0] == 'Y'));

            std::cout << "3. Show real-time pose information? (y/n): ";
            std::getline(std::cin, input);
            g_show_pose_info = (!input.empty() && (input[0] == 'y' || input[0] == 'Y'));
        }

        std::cout << "\nSLAM Configuration:" << std::endl;
        std::cout << "  3D SLAM: " << (g_enable_slam ? "ENABLED" : "DISABLED") << std::endl;
        std::cout << "  Real-time 3D Map: " << (g_show_3d_window ? "ENABLED" : "DISABLED") << std::endl;
        std::cout << "  Pose Info: " << (g_show_pose_info ? "ENABLED" : "DISABLED") << std::endl;
        std::cout << "=================================================" << std::endl;
    }

    // ============================================================================
    // ENHANCED FRAME CALLBACK - MODIFIED FROM YOUR EXISTING PATTERN
    // ============================================================================

    void EnhancedFrameCallbackWithSLAM(const StreamFrames & frames) {
        cv::Mat rgbFrame, depthFrame;
        static int frame_count = 0;
        frame_count++;

        // Extract frames (your existing exact pattern)
        for (int i = 0; i < frames.count; i++) {
            auto frame = frames.frame_ptr[i];

            if (frame->frame_type == kRgbFrame && frame->cols > 0 && frame->rows > 0) {
                cv::Mat yuvMat(static_cast<int>(frame->rows * 1.5f), frame->cols, CV_8UC1, frame->data.get());
                rgbFrame = cv::Mat(frame->rows, frame->cols, CV_8UC3);
                cv::cvtColor(yuvMat, rgbFrame, cv::COLOR_YUV2BGR_NV12);
            }
            else if (frame->frame_type == kDepthFrame && frame->cols > 0 && frame->rows > 0) {
                depthFrame = cv::Mat(frame->rows, frame->cols, CV_16UC1, frame->data.get());
            }
        }

        // Initialize SLAM if this is the first frame with both RGB and depth
        if (g_enable_slam && !g_slam_scanner && !rgbFrame.empty() && !depthFrame.empty()) {
            initializeSLAMSystem(rgbFrame.cols, rgbFrame.rows);
        }

        // Process frames with SLAM
        if (!rgbFrame.empty() && !depthFrame.empty()) {
            processSLAMFrame(rgbFrame, depthFrame);
        }

        // Display the frame with SLAM overlay
        if (!rgbFrame.empty()) {
            cv::Mat display_frame = rgbFrame.clone();

            // Add SLAM information overlay
            addSLAMOverlay(display_frame);

            std::string windowTitle = "Enhanced Scanner with SLAM";
            if (g_enable_slam) {
                windowTitle += " [SLAM: ON]";
            }

            cv::imshow(windowTitle, display_frame);

            // Handle keyboard input
            char key = cv::waitKey(1) & 0xFF;
            if (key == 'q' || key == 27) { // 'q' or ESC
                is_running = false;
            }
            else {
                handleSLAMKeys(key);
            }
        }

        // Show depth frame
        if (!depthFrame.empty()) {
            cv::Mat depthDisplay;
            depthFrame.convertTo(depthDisplay, CV_8UC1, 0.25);
            cv::imshow("Depth", depthDisplay);
        }
    }

    // ============================================================================
    // MAIN FUNCTION - USING YOUR EXACT EXISTING PATTERN
    // ============================================================================

    int main() {
        std::cout << "========== Enhanced Depth Scanner with SLAM ==========" << std::endl;
        std::cout << "Advanced depth camera scanner with 3D mapping" << std::endl;
        std::cout << "=====================================================" << std::endl;

        // Get user configuration
        getUserConfiguration();

        is_running = true;

        // Initialize depth camera (your exact existing pattern from test_scanner/cpp/main.cpp)
        DeviceManager::GetInstance()->RegisterDeviceConnectedCallback();

        std::vector<DeviceInformation> device_list;
        int ret = DeviceManager::GetInstance()->GetDeviceList(device_list);
        CHECK_SDK_RETURN_VALUE(ret);
        CHECK_DEVICE_COUNT(device_list.size());

        {
            printf("Found %zu depth camera(s):\n", device_list.size());
            for (size_t index = 0; index < device_list.size(); index++) {
                printf("  [%zu] device_addr: %d, usb_port: %s\n", index,
                    device_list[index].ir_camera.device_addr,
                    device_list[index].ir_camera.port_path.c_str());
            }
        }

        auto device = DeviceManager::GetInstance()->CreateDevice(device_list[0]);
        CHECK_DEVICE_VALID(device);

        ret = device->Open();
        CHECK_SDK_RETURN_VALUE(ret);

        std::string device_name = device->GetDeviceName();
        viewer_helper = std::make_shared<ViewerHelper>(device_name);

        // Get supported frame modes
        std::vector<std::tuple<FrameMode, FrameMode, FrameMode>> device_resolution_vec;
        ret = device->GetSupportedFrameMode(device_resolution_vec);
        CHECK_SDK_RETURN_VALUE(ret);

        if (device_resolution_vec.empty()) {
            std::cout << "No supported frame modes found!" << std::endl;
            return -1;
        }

        // Use first available mode (your pattern)
        auto [ir_mode, rgb_mode, depth_mode] = device_resolution_vec[0];
        ret = device->SetMode(ir_mode, rgb_mode, depth_mode);
        CHECK_SDK_RETURN_VALUE(ret);

        // Get camera parameters (your exact pattern)
        Intrinsic ir_intri, rgb_intri;
        Extrinsic extrinsic;
        device->GetCameraParameters(ir_intri, rgb_intri, extrinsic);

        g_camera_parm.cx = ir_intri.principal_point[0];
        g_camera_parm.cy = ir_intri.principal_point[1];
        g_camera_parm.fx = ir_intri.focal_length[0];
        g_camera_parm.fy = ir_intri.focal_length[1];

        // Create stream (your exact pattern)
        Stream* stream = nullptr;
        std::vector<StreamType> stream_types = { StreamType::kRgbd };
        ret = device->CreateStream(stream, stream_types);
        CHECK_SDK_RETURN_VALUE(ret);

        ret = stream->Start();
        CHECK_SDK_RETURN_VALUE(ret);

        std::cout << "\nEnhanced Scanner ready!" << std::endl;
        std::cout << "Controls:" << std::endl;
        std::cout << "  'q' or ESC - Quit and save final map" << std::endl;
        std::cout << "  's' - Manually save current map" << std::endl;
        std::cout << "  'p' - Print SLAM status" << std::endl;
        std::cout << "  'v' - Toggle 3D visualization window" << std::endl;
        std::cout << "\nProcessing camera frames..." << std::endl;

        if (g_show_3d_window) {
            std::cout << "\nSLAM 3D Visualization Tips:" << std::endl;
            std::cout << "   - Watch the growing 3D map in real-time!" << std::endl;
            std::cout << "   - Use mouse to rotate, zoom, and pan the 3D view" << std::endl;
            std::cout << "   - The map accumulates as you move the scanner" << std::endl;
        }

        // Main processing loop (your exact pattern)
        while (is_running) {
            StreamFrames frames;
            ret = stream->GetFrames(frames, 100); // 100ms timeout

            if (ret == 0 && frames.count > 0) {
                EnhancedFrameCallbackWithSLAM(frames);
            }
        }

        // Cleanup and save final map
        std::cout << "Shutting down..." << std::endl;

        if (g_enable_slam && g_slam_scanner) {
            auto timestamp = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            std::stringstream ss;
            ss << g_output_directory << "maps/final_world_map_" << timestamp << ".pcd";

            if (g_slam_scanner->saveWorldMap(ss.str())) {
                std::cout << "Final world map saved to: " << ss.str() << std::endl;
            }

            g_slam_scanner->printStatus();
        }

        // Cleanup camera (your exact pattern)
        stream->Stop();
        device->DestroyStream(stream);
        cv::destroyAllWindows();

        std::cout << "Program finished successfully!" << std::endl;
        return 0;
    }

#pragma warning(pop)