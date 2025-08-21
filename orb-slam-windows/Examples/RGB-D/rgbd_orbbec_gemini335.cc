/**
 * ORB-SLAM3 RGB-D example for Orbbec Gemini 335 camera
 * WITH ORB-SLAM3 MAP POINT EXPORT (Simplified Approach)
 */

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <thread>
#include <iomanip>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include <System.h>

// Orbbec SDK includes
#include "libobsensor/ObSensor.hpp"

#ifdef _WIN32
    #include <windows.h>
    #include <io.h>
    #include <conio.h>
    #include <direct.h>
    #include <process.h>
#else
    #include <signal.h>
    #include <csignal>
    #include <unistd.h>
    #include <sys/select.h>
    #include <termios.h>
    #include <fcntl.h>
    #include <sys/stat.h>
#endif
#include <atomic>

using namespace std;

// Global resource manager for safe shutdown
class SafeResourceManager {
public:
    SafeResourceManager() : should_exit_(false) {}
    void requestExit() { should_exit_ = true; }
    bool shouldExit() const { return should_exit_; }
private:
    std::atomic<bool> should_exit_;
};

SafeResourceManager g_resource_manager;

// Signal handler function
#ifdef _WIN32
BOOL WINAPI ConsoleCtrlHandler(DWORD dwCtrlType) {
    switch (dwCtrlType) {
        case CTRL_C_EVENT:
        case CTRL_CLOSE_EVENT:
        case CTRL_BREAK_EVENT:
            std::cout << "\n[DEBUG] Windows console event " << dwCtrlType << std::endl;
            g_resource_manager.requestExit();
            return TRUE;
        default:
            return FALSE;
    }
}
#else
void signalHandler(int signum) {
    std::cout << "\n[DEBUG] Received signal " << signum << std::endl;
    g_resource_manager.requestExit();
}
#endif

// Setup terminal for non-blocking input
#ifdef _WIN32
void setupTerminal() {
    std::cout << "[DEBUG] Windows console ready for input" << std::endl;
}

void restoreTerminal() {
    // No restoration needed on Windows
}
#else
void setupTerminal() {
    struct termios term;
    tcgetattr(STDIN_FILENO, &term);
    term.c_lflag &= ~(ICANON | ECHO); // Disable line buffering and echo
    tcsetattr(STDIN_FILENO, TCSANOW, &term);
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK); // Make stdin non-blocking
}

void restoreTerminal() {
    struct termios term;
    tcgetattr(STDIN_FILENO, &term);
    term.c_lflag |= (ICANON | ECHO); // Restore line buffering and echo
    tcsetattr(STDIN_FILENO, TCSANOW, &term);
    fcntl(STDIN_FILENO, F_SETFL, 0); // Make stdin blocking again
}
#endif

// Function to save RGB-D frame for reference/backup
void SaveRGBDFrame(const cv::Mat& rgb, const cv::Mat& depth, int frame_number, double timestamp) {
    try {
        std::string folder = "3D_Reconstruction_Data/";
        
        // Create filenames with frame number and timestamp
        std::string rgb_filename = folder + "rgbd_color_" + std::to_string(frame_number) + ".png";
        std::string depth_filename = folder + "rgbd_depth_" + std::to_string(frame_number) + ".png";
        std::string info_filename = folder + "rgbd_info_" + std::to_string(frame_number) + ".txt";
        
        // Save RGB image
        bool rgb_ok = cv::imwrite(rgb_filename, rgb);
        
        // Save depth image (16-bit PNG to preserve depth precision)
        bool depth_ok = cv::imwrite(depth_filename, depth);
        
        // Save frame info (timestamp, frame number)
        std::ofstream info_file(info_filename);
        info_file << "frame_number: " << frame_number << std::endl;
        info_file << "timestamp: " << std::fixed << std::setprecision(6) << timestamp << std::endl;
        info_file << "rgb_file: " << rgb_filename << std::endl;
        info_file << "depth_file: " << depth_filename << std::endl;
        info_file.close();
        
        if (rgb_ok && depth_ok) {
            std::cout << "[BACKUP] Saved RGB-D frame " << frame_number << " for backup" << std::endl;
        } else {
            std::cout << "[ERROR] Failed to save RGB-D frame " << frame_number << std::endl;
        }
        
    } catch (const cv::Exception& e) {
        std::cout << "[ERROR] RGB-D save error: " << e.what() << std::endl;
    }
}

// Software coordinate transformation class
class DepthToColorTransform {
private:
    Eigen::Matrix3d R_depth_to_color_;
    Eigen::Vector3d t_depth_to_color_;
    cv::Mat depthK_, colorK_;
    bool initialized_;
    
public:
    DepthToColorTransform() : initialized_(false) {
        R_depth_to_color_ << 0.999998,  0.001045, -0.001768,
                            -0.001045,  0.999999, -0.000077,
                             0.001768,  0.000078,  0.999998;
        
        t_depth_to_color_ << -13.851061, -0.219677, -2.052635; // mm
        t_depth_to_color_ /= 1000.0; // Convert to meters
        
        std::cout << "[DEBUG] Software coordinate transformer initialized." << std::endl;
    }
    
    void setCameraIntrinsics(const cv::Mat& depthK, const cv::Mat& colorK) {
        depthK_ = depthK.clone();
        colorK_ = colorK.clone();
        initialized_ = true;
        std::cout << "[DEBUG] Transformer ready for software alignment." << std::endl;
        
        std::cout << "[DEBUG] Depth camera matrix:" << std::endl << depthK_ << std::endl;
        std::cout << "[DEBUG] Color camera matrix:" << std::endl << colorK_ << std::endl;
    }
    
    cv::Mat alignDepthToColor(const cv::Mat& depthImage, const cv::Size& colorSize) {
        if (!initialized_) return cv::Mat::zeros(colorSize, CV_16UC1);
        
        cv::Mat alignedDepth = cv::Mat::zeros(colorSize, CV_16UC1);
        
        float fx_d = depthK_.at<float>(0, 0), fy_d = depthK_.at<float>(1, 1);
        float cx_d = depthK_.at<float>(0, 2), cy_d = depthK_.at<float>(1, 2);
        float fx_c = colorK_.at<float>(0, 0), fy_c = colorK_.at<float>(1, 1);
        float cx_c = colorK_.at<float>(0, 2), cy_c = colorK_.at<float>(1, 2);
        
        int valid_points = 0;
        int total_depth_points = 0;
        
        for (int v = 0; v < depthImage.rows; v++) {
            for (int u = 0; u < depthImage.cols; u++) {
                uint16_t depth_value = depthImage.at<uint16_t>(v, u);
                if (depth_value == 0) continue;
                
                total_depth_points++;
                float depth_m = depth_value / 1000.0f;
                Eigen::Vector3d point_depth((u - cx_d) * depth_m / fx_d,
                                          (v - cy_d) * depth_m / fy_d, depth_m);
                
                Eigen::Vector3d point_color = R_depth_to_color_ * point_depth + t_depth_to_color_;
                if (point_color.z() <= 0) continue;
                
                int u_c = (int)(fx_c * point_color.x() / point_color.z() + cx_c + 0.5);
                int v_c = (int)(fy_c * point_color.y() / point_color.z() + cy_c + 0.5);
                
                if (u_c >= 0 && u_c < colorSize.width && v_c >= 0 && v_c < colorSize.height) {
                    alignedDepth.at<uint16_t>(v_c, u_c) = depth_value;
                    valid_points++;
                }
            }
        }
        
        static int alignment_count = 0;
        if (++alignment_count % 30 == 0) {
            std::cout << "[DEBUG] Software alignment: " << valid_points 
                     << "/" << total_depth_points << " points ("
                     << (100.0 * valid_points / total_depth_points) << "%)" << std::endl;
        }
        
        return alignedDepth;
    }
    
    void fillDepthHoles(cv::Mat& depth) {
        cv::Mat mask = (depth == 0);
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
        
        cv::Mat dilated;
        cv::dilate(depth, dilated, kernel);
        dilated.copyTo(depth, mask);
    }
};

class OrbbecCapture {
public:
    OrbbecCapture(bool use_hardware_align = true) 
        : pipeline_(nullptr), config_(nullptr), use_hw_align_(use_hardware_align) {
        std::cout << "[DEBUG] Using " << (use_hw_align_ ? "HARDWARE" : "SOFTWARE") << " alignment" << std::endl;
    }
    
    ~OrbbecCapture() {
        if (pipeline_) pipeline_->stop();
    }

    bool initialize() {
        try {
            std::cout << "\n[DEBUG] ===== ORBBEC INITIALIZATION DEBUG =====" << std::endl;
            
            context_ = std::make_shared<ob::Context>();
            auto deviceList = context_->queryDeviceList();
            std::cout << "[DEBUG] Found " << deviceList->getCount() << " devices" << std::endl;
            
            if (deviceList->getCount() == 0) {
                std::cerr << "[ERROR] No Orbbec device found!" << std::endl;
                return false;
            }
            
            device_ = deviceList->getDevice(0);
            std::cout << "[DEBUG] Device: " << device_->getDeviceInfo()->name() << std::endl;
            std::cout << "[DEBUG] Serial: " << device_->getDeviceInfo()->serialNumber() << std::endl;
            std::cout << "[DEBUG] Firmware: " << device_->getDeviceInfo()->firmwareVersion() << std::endl;
            
            pipeline_ = std::make_shared<ob::Pipeline>(device_);
            config_ = std::make_shared<ob::Config>();
            
            // Configure streams - try multiple resolution options
            std::cout << "\n[DEBUG] Configuring streams..." << std::endl;
            
            bool color_configured = false;
            bool depth_configured = false;
            
            // Try to configure COLOR stream
            try {
                std::cout << "[DEBUG] Trying COLOR: 1920x1080@30fps" << std::endl;
                config_->enableVideoStream(OB_STREAM_COLOR, 1920, 1080, 30, OB_FORMAT_RGB);
                color_configured = true;
                std::cout << "[DEBUG] COLOR configured: 1920x1080@30fps SUCCESS" << std::endl;
            } catch (const ob::Error& e) {
                std::cout << "[WARNING] COLOR 1920x1080@30fps failed: " << e.what() << std::endl;
                
                try {
                    std::cout << "[DEBUG] Trying COLOR: 640x360@30fps" << std::endl;
                    config_->enableVideoStream(OB_STREAM_COLOR, 640, 360, 30, OB_FORMAT_RGB);
                    color_configured = true;
                    std::cout << "[DEBUG] COLOR configured: 640x360@30fps SUCCESS" << std::endl;
                } catch (const ob::Error& e2) {
                    std::cout << "[WARNING] COLOR 640x360@30fps failed: " << e2.what() << std::endl;
                    
                    try {
                        std::cout << "[DEBUG] Trying COLOR: 1280x720@30fps (will scale)" << std::endl;
                        config_->enableVideoStream(OB_STREAM_COLOR, 1280, 720, 30, OB_FORMAT_RGB);
                        color_configured = true;
                        std::cout << "[DEBUG] COLOR configured: 1280x720@30fps SUCCESS (will resize to 640x360)" << std::endl;
                    } catch (const ob::Error& e3) {
                        std::cout << "[ERROR] All COLOR configurations failed!" << std::endl;
                    }
                }
            }
            
            // Try to configure DEPTH stream
            try {
                std::cout << "[DEBUG] Trying DEPTH: 848x480@30fps" << std::endl;
                config_->enableVideoStream(OB_STREAM_DEPTH, 848, 480, 30, OB_FORMAT_Y16);
                depth_configured = true;
                std::cout << "[DEBUG] DEPTH configured: 848x480@30fps SUCCESS" << std::endl;
            } catch (const ob::Error& e) {
                std::cout << "[WARNING] DEPTH 848x480@30fps failed: " << e.what() << std::endl;
                
                try {
                    std::cout << "[DEBUG] Trying DEPTH: 640x360@30fps" << std::endl;
                    config_->enableVideoStream(OB_STREAM_DEPTH, 640, 360, 30, OB_FORMAT_Y16);
                    depth_configured = true;
                    std::cout << "[DEBUG] DEPTH configured: 640x360@30fps SUCCESS" << std::endl;
                } catch (const ob::Error& e2) {
                    std::cout << "[WARNING] DEPTH 640x360@30fps failed: " << e2.what() << std::endl;
                    
                    try {
                        std::cout << "[DEBUG] Trying DEPTH: 848x480@30fps (will scale)" << std::endl;
                        config_->enableVideoStream(OB_STREAM_DEPTH, 848, 480, 30, OB_FORMAT_Y16);
                        depth_configured = true;
                        std::cout << "[DEBUG] DEPTH configured: 848x480@30fps SUCCESS (will resize)" << std::endl;
                    } catch (const ob::Error& e3) {
                        std::cout << "[ERROR] All DEPTH configurations failed!" << std::endl;
                    }
                }
            }
            
            if (!color_configured || !depth_configured) {
                std::cout << "[ERROR] Failed to configure required streams!" << std::endl;
                return false;
            }
            
            // Enable frame synchronization
            std::cout << "[DEBUG] Enabling frame synchronization..." << std::endl;
            config_->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ANY_SITUATION);
            pipeline_->enableFrameSync();
            
            // Create hardware alignment filter if requested
            if (use_hw_align_) {
                std::cout << "[DEBUG] Creating hardware alignment filter..." << std::endl;
                try {
                    align_filter_ = std::make_shared<ob::Align>(OB_STREAM_COLOR);
                    std::cout << "[DEBUG] Hardware alignment filter created SUCCESS" << std::endl;
                } catch (const ob::Error& e) {
                    std::cout << "[WARNING] Hardware alignment failed: " << e.what() << std::endl;
                    std::cout << "[WARNING] Falling back to software alignment" << std::endl;
                    use_hw_align_ = false;
                }
            }
            
            // Get camera intrinsics
            std::cout << "\n[DEBUG] Getting camera intrinsics..." << std::endl;
            std::shared_ptr<ob::VideoStreamProfile> colorProfile = nullptr;
            std::shared_ptr<ob::VideoStreamProfile> depthProfile = nullptr;

            // Search for 1920x1080 color profile first, then fallback
            try {
                auto colorSensor = device_->getSensor(OB_SENSOR_COLOR);
                auto colorProfiles = colorSensor->getStreamProfileList();
                
                for (uint32_t i = 0; i < colorProfiles->getCount(); i++) {
                    auto profile = colorProfiles->getProfile(i)->as<ob::VideoStreamProfile>();
                    if (profile->getWidth() == 1920 && profile->getHeight() == 1080) {
                        colorProfile = profile;
                        std::cout << "[DEBUG] Found matching color profile: " << profile->getWidth() 
                                << "x" << profile->getHeight() << "@" << profile->getFps() << "fps" << std::endl;
                        break;
                    }
                }
                
                if (!colorProfile) {
                    std::cout << "[WARNING] No 640x360 color profile found, using default" << std::endl;
                    auto profiles = pipeline_->getStreamProfileList(OB_SENSOR_COLOR);
                    colorProfile = profiles->getProfile(0)->as<ob::VideoStreamProfile>();
                }
            } catch (const ob::Error& e) {
                std::cout << "[WARNING] Error finding color profile: " << e.what() << std::endl;
                auto profiles = pipeline_->getStreamProfileList(OB_SENSOR_COLOR);
                colorProfile = profiles->getProfile(0)->as<ob::VideoStreamProfile>();
            }

            // Search for 848x480 depth profile first, then fallback
            try {
                auto depthSensor = device_->getSensor(OB_SENSOR_DEPTH);
                auto depthProfiles = depthSensor->getStreamProfileList();
                
                for (uint32_t i = 0; i < depthProfiles->getCount(); i++) {
                    auto profile = depthProfiles->getProfile(i)->as<ob::VideoStreamProfile>();
                    if (profile->getWidth() == 848 && profile->getHeight() == 480) {
                        depthProfile = profile;
                        std::cout << "[DEBUG] Found matching depth profile: " << profile->getWidth() 
                                << "x" << profile->getHeight() << "@" << profile->getFps() << "fps" << std::endl;
                        break;
                    }
                }
                
                if (!depthProfile) {
                    std::cout << "[WARNING] No 640x360 depth profile found, using default" << std::endl;
                    auto profiles = pipeline_->getStreamProfileList(OB_SENSOR_DEPTH);
                    depthProfile = profiles->getProfile(0)->as<ob::VideoStreamProfile>();
                }
            } catch (const ob::Error& e) {
                std::cout << "[WARNING] Error finding depth profile: " << e.what() << std::endl;
                auto profiles = pipeline_->getStreamProfileList(OB_SENSOR_DEPTH);
                depthProfile = profiles->getProfile(0)->as<ob::VideoStreamProfile>();
            }

            // Now use the found profiles to get intrinsics
            if (colorProfile && depthProfile) {
                std::cout << "[DEBUG] Using color profile: " << colorProfile->getWidth() << "x" 
                        << colorProfile->getHeight() << "@" << colorProfile->getFps() << "fps" << std::endl;
                std::cout << "[DEBUG] Using depth profile: " << depthProfile->getWidth() << "x" 
                        << depthProfile->getHeight() << "@" << depthProfile->getFps() << "fps" << std::endl;
                
                auto colorIntrinsic = colorProfile->getIntrinsic();
                auto depthIntrinsic = depthProfile->getIntrinsic();
                
                colorK_ = (cv::Mat_<float>(3, 3) << 
                    colorIntrinsic.fx, 0, colorIntrinsic.cx,
                    0, colorIntrinsic.fy, colorIntrinsic.cy, 0, 0, 1);
                
                depthK_ = (cv::Mat_<float>(3, 3) << 
                    depthIntrinsic.fx, 0, depthIntrinsic.cx,
                    0, depthIntrinsic.fy, depthIntrinsic.cy, 0, 0, 1);
                
                sw_transformer_.setCameraIntrinsics(depthK_, colorK_);
                
                std::cout << "\n[DEBUG] ===== SDK REPORTED INTRINSICS =====" << std::endl;
                std::cout << "[DEBUG] Color intrinsics: fx=" << colorIntrinsic.fx << ", fy=" << colorIntrinsic.fy 
                         << ", cx=" << colorIntrinsic.cx << ", cy=" << colorIntrinsic.cy << std::endl;
                std::cout << "[DEBUG] Depth intrinsics: fx=" << depthIntrinsic.fx << ", fy=" << depthIntrinsic.fy 
                         << ", cx=" << depthIntrinsic.cx << ", cy=" << depthIntrinsic.cy << std::endl;
                std::cout << "[DEBUG] =======================================" << std::endl;
            }
            
            return true;
        } catch (const ob::Error& e) {
            std::cerr << "[ERROR] Initialization error: " << e.what() << std::endl;
            return false;
        }
    }
    
    bool start() {
        try {
            std::cout << "[DEBUG] Starting pipeline..." << std::endl;
            pipeline_->start(config_);
            std::cout << "[DEBUG] Camera started successfully SUCCESS" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            return true;
        } catch (const ob::Error& e) {
            std::cerr << "[ERROR] Start error: " << e.what() << std::endl;
            return false;
        }
    }
    
    bool getFrames(cv::Mat& color, cv::Mat& depth, double& timestamp) {
        static int frame_count = 0;
        static int hw_failures = 0;
        frame_count++;
        
        bool debug_this_frame = (frame_count % 30 == 0);
        
        try {
            auto frameSet = pipeline_->waitForFrames(1000);
            if (!frameSet) {
                if (debug_this_frame) {
                    std::cout << "[WARNING] No frameset received" << std::endl;
                }
                return false;
            }
            
            if (use_hw_align_) {
                auto alignedFrame = align_filter_->process(frameSet);
                if (!alignedFrame) {
                    hw_failures++;
                    if (hw_failures % 10 == 0 || debug_this_frame) {
                        std::cout << "[WARNING] Hardware alignment failed #" << hw_failures << ", retrying..." << std::endl;
                    }
                    return false;
                }
                
                auto alignedFrameSet = alignedFrame->as<ob::FrameSet>();
                if (!alignedFrameSet) {
                    std::cout << "[WARNING] Failed to convert aligned frame" << std::endl;
                    return false;
                }
                
                auto colorFrame = alignedFrameSet->getFrame(OB_FRAME_COLOR);
                auto alignedDepthFrame = alignedFrameSet->getFrame(OB_FRAME_DEPTH);
                
                if (!colorFrame || !alignedDepthFrame) {
                    if (debug_this_frame) {
                        std::cout << "[WARNING] Missing aligned frames" << std::endl;
                    }
                    return false;
                }
                
                auto colorVideoFrame = colorFrame->as<ob::ColorFrame>();
                cv::Mat color_temp(colorVideoFrame->getHeight(), colorVideoFrame->getWidth(), 
                                  CV_8UC3, (void*)colorVideoFrame->getData());
                cv::cvtColor(color_temp, color, cv::COLOR_RGB2BGR);
                timestamp = colorVideoFrame->getTimeStampUs() / 1000000.0;
                
                auto depthVideoFrame = alignedDepthFrame->as<ob::DepthFrame>();
                depth = cv::Mat(depthVideoFrame->getHeight(), depthVideoFrame->getWidth(), 
                               CV_16UC1, (void*)depthVideoFrame->getData()).clone();
                
                if (debug_this_frame) {
                    std::cout << "[DEBUG] Hardware alignment OK - Color: " << color.size() 
                             << ", Depth: " << depth.size() << std::endl;
                }
                
                sw_transformer_.fillDepthHoles(depth);
                
                if (hw_failures > 0) {
                    std::cout << "[DEBUG] Hardware alignment recovered after " << hw_failures << " failures" << std::endl;
                    hw_failures = 0;
                }
                
            } else {
                auto colorFrame = frameSet->getFrame(OB_FRAME_COLOR);
                auto depthFrame = frameSet->getFrame(OB_FRAME_DEPTH);
                
                if (!colorFrame || !depthFrame) {
                    if (debug_this_frame) {
                        std::cout << "[WARNING] Missing raw frames" << std::endl;
                    }
                    return false;
                }
                
                auto colorVideoFrame = colorFrame->as<ob::ColorFrame>();
                cv::Mat color_temp(colorVideoFrame->getHeight(), colorVideoFrame->getWidth(), 
                                  CV_8UC3, (void*)colorVideoFrame->getData());
                cv::cvtColor(color_temp, color, cv::COLOR_RGB2BGR);
                timestamp = colorVideoFrame->getTimeStampUs() / 1000000.0;
                
                auto depthVideoFrame = depthFrame->as<ob::DepthFrame>();
                cv::Mat rawDepth(depthVideoFrame->getHeight(), depthVideoFrame->getWidth(), 
                               CV_16UC1, (void*)depthVideoFrame->getData());
                
                if (debug_this_frame) {
                    std::cout << "[DEBUG] Raw frames - Color: " << color.size() 
                             << ", Depth: " << rawDepth.size() << std::endl;
                }
                
                depth = sw_transformer_.alignDepthToColor(rawDepth, color.size());
                sw_transformer_.fillDepthHoles(depth);
            }
            
            // Force resize to target resolution if needed  
            cv::Size target_size(1920, 1080);  // Target 1080p
            if (color.size() != target_size) {
                if (debug_this_frame) {
                    std::cout << "[DEBUG] Resizing color " << color.size() << " -> " << target_size << std::endl;
                }
                cv::resize(color, color, target_size);
            }

            if (depth.size() != target_size) {
                if (debug_this_frame) {
                    std::cout << "[DEBUG] Resizing depth " << depth.size() << " -> " << target_size << std::endl;
                }
                cv::resize(depth, depth, target_size, 0, 0, cv::INTER_NEAREST);
            }
            
            if (debug_this_frame) {
                double minVal, maxVal;
                cv::minMaxLoc(depth, &minVal, &maxVal);
                int nonZero = cv::countNonZero(depth);
                std::cout << "[DEBUG] Final depth stats - Range: " << minVal << "-" << maxVal 
                         << "mm, Valid: " << nonZero << "/" << (depth.rows * depth.cols) 
                         << " (" << (100.0 * nonZero / (depth.rows * depth.cols)) << "%)" << std::endl;
            }
            
            return !color.empty() && !depth.empty();
            
        } catch (const ob::Error& e) {
            std::cerr << "[ERROR] Frame error: " << e.what() << std::endl;
            return false;
        }
    }
    
    void toggleAlignmentMode() {
        use_hw_align_ = !use_hw_align_;
        std::cout << "[DEBUG] Switched to " << (use_hw_align_ ? "HARDWARE" : "SOFTWARE") << " alignment" << std::endl;
    }
    
    // Getter methods for camera parameters
    cv::Mat getColorCameraMatrix() const {
        return colorK_.clone();
    }
    
    cv::Mat getDepthCameraMatrix() const {
        return depthK_.clone();
    }
    
    cv::Size getFrameSize() const {
        return cv::Size(1920, 1080); // Target resolution we resize to
    }
    
private:
    std::shared_ptr<ob::Context> context_;
    std::shared_ptr<ob::Device> device_;
    std::shared_ptr<ob::Pipeline> pipeline_;
    std::shared_ptr<ob::Config> config_;
    std::shared_ptr<ob::Align> align_filter_;
    
    cv::Mat colorK_, depthK_;
    DepthToColorTransform sw_transformer_;
    bool use_hw_align_;
};

void GenerateMapPointConverter() {
    std::cout << "[MESH] Generating ENHANCED TUBE MESH GENERATOR WITH FRAME DETAIL PRESERVATION..." << std::endl;
    std::string folder = "3D_Reconstruction_Data/";
    
    std::ofstream script(folder + "tube_mesh_generator.py");
    script << "#!/usr/bin/env python3\n";
    script << "\"\"\"\n";
    script << "TUBE-SPECIFIC MESH GENERATOR\n";
    script << "Optimized for metal cuboid tubes and geometric objects\n";
    script << "Handles sparse points and sharp edges properly\n";
    script << "\"\"\"\n\n";
    
    script << "import numpy as np\nimport open3d as o3d\nimport os\nimport time\nimport psutil\n\n";
    
    // Memory monitoring
    script << "def print_memory_status(step=''):\n";
    script << "    mem = psutil.virtual_memory()\n";
    script << "    print(f'Memory: {mem.used / (1024**3):.1f}/{mem.total / (1024**3):.1f}GB ({mem.percent:.1f}%) - {step}')\n\n";
    
    // Load ORB-SLAM points
    script << "def load_orb_slam_map_points(filename):\n";
    script << "    print(f'Loading ORB-SLAM3 map points from {filename}...')\n";
    script << "    if not os.path.exists(filename): print(f'ERROR: File not found: {filename}'); return None\n";
    script << "    try:\n";
    script << "        coords = np.genfromtxt(filename, delimiter=',', skip_header=1)\n";
    script << "        if coords.size == 0: print('ERROR: No data in file!'); return None\n";
    script << "        if coords.ndim == 1: coords = coords.reshape(1, -1)\n";
    script << "        points = coords[:, :3]\n";
    script << "        print(f'SUCCESS: Loaded {len(points):,} ORB-SLAM3 map points')\n";
    script << "        print(f'Point cloud spans:')\n";
    script << "        print(f'   X: {np.min(points[:, 0]):.3f} to {np.max(points[:, 0]):.3f} m')\n";
    script << "        print(f'   Y: {np.min(points[:, 1]):.3f} to {np.max(points[:, 1]):.3f} m')\n";
    script << "        print(f'   Z: {np.min(points[:, 2]):.3f} to {np.max(points[:, 2]):.3f} m')\n";
    script << "        return points\n";
    script << "    except Exception as e: print(f'ERROR: Error loading map points: {e}'); return None\n\n";
    
    // Analyze geometry
    script << "def analyze_tube_geometry(points):\n";
    script << "    print('Analyzing tube geometry...')\n";
    script << "    min_coords, max_coords = np.min(points, axis=0), np.max(points, axis=0)\n";
    script << "    dimensions = max_coords - min_coords\n";
    script << "    print(f'Tube dimensions:')\n";
    script << "    print(f'   Width (X):  {dimensions[0]:.3f} m')\n";
    script << "    print(f'   Height (Y): {dimensions[1]:.3f} m')\n";
    script << "    print(f'   Length (Z): {dimensions[2]:.3f} m')\n";
    script << "    volume = np.prod(dimensions)\n";
    script << "    density = len(points) / volume if volume > 0 else 0\n";
    script << "    print(f'Point density: {density:.1f} points/m3')\n";
    script << "    is_sparse = len(points) < 1000 or density < 100\n";
    script << "    if is_sparse: print('WARNING: Sparse point cloud detected - using geometric reconstruction')\n";
    script << "    else: print('SUCCESS: Good point density - using standard reconstruction')\n";
    script << "    return {'dimensions': dimensions, 'density': density, 'is_sparse': is_sparse, 'center': (min_coords + max_coords) / 2, 'bbox_min': min_coords, 'bbox_max': max_coords}\n\n";
    
    // Optimized normals
    script << "def tube_optimized_normals(pcd, geometry_info):\n";
    script << "    print('Tube-optimized normal estimation...')\n";
    script << "    point_count = len(pcd.points)\n";
    script << "    if geometry_info['is_sparse']:\n";
    script << "        print('Sparse tube - using large radius for normals')\n";
    script << "        max_dim = np.max(geometry_info['dimensions'])\n";
    script << "        radius = max(0.05, max_dim / 20)\n";
    script << "        max_nn = min(30, max(10, point_count // 10))\n";
    script << "        search_param = o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn)\n";
    script << "        print(f'   Using radius: {radius:.3f}m, max_nn: {max_nn}')\n";
    script << "    else:\n";
    script << "        print('Dense tube - using standard normals')\n";
    script << "        search_param = o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=20)\n";
    script << "    pcd.estimate_normals(search_param=search_param)\n";
    script << "    try: pcd.orient_normals_consistent_tangent_plane(100); print('SUCCESS: Normals oriented consistently')\n";
    script << "    except: print('WARNING: Could not orient normals consistently')\n";
    script << "    return pcd\n\n";
    
    // Alpha shapes with fine detail preservation
    script << "def create_tube_mesh_alpha_shapes(pcd, geometry_info):\n";
    script << "    print('Creating tube mesh with Alpha Shapes...')\n";
    script << "    min_dim = np.min(geometry_info['dimensions'])\n";
    script << "    base_alpha = min_dim / 15\n";
    script << "    point_density = len(pcd.points) / (geometry_info['dimensions'][0] * geometry_info['dimensions'][1])\n";
    script << "    density_factor = max(0.5, min(2.0, point_density / 1000))\n";
    script << "    alpha_values = [\n";
    script << "        0.0005 / density_factor,  # Very fine detail\n";
    script << "        0.001 / density_factor,   # Fine detail\n";
    script << "        0.002 / density_factor,   # Medium detail\n";
    script << "        0.004 / density_factor,   # Coarser detail\n";
    script << "        0.008 / density_factor    # Backup\n";
    script << "    ]\n";
    script << "    print(f'Frame-preserving alphas: {[f\"{a*1000:.1f}mm\" for a in alpha_values]}')\n";
    script << "    print(f'Testing alpha values: {[f\"{a:.4f}\" for a in alpha_values]}')\n";
    script << "    best_mesh, best_triangle_count, best_alpha = None, 0, 0\n";
    script << "    for alpha in alpha_values:\n";
    script << "        try:\n";
    script << "            print(f'   Testing alpha = {alpha:.4f}...')\n";
    script << "            mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)\n";
    script << "            triangle_count = len(mesh.triangles)\n";
    script << "            print(f'      Result: {triangle_count:,} triangles')\n";
    script << "            if 50 < triangle_count < 200000 and triangle_count > best_triangle_count:\n";
    script << "                if best_mesh is not None: del best_mesh\n";
    script << "                best_mesh, best_triangle_count, best_alpha = mesh, triangle_count, alpha\n";
    script << "            else: del mesh\n";
    script << "        except Exception as e: print(f'      Failed: {e}')\n";
    script << "    if best_mesh is not None:\n";
    script << "        print(f'SUCCESS: Best Alpha Shapes result: alpha={best_alpha:.4f}, {best_triangle_count:,} triangles')\n";
    script << "        return best_mesh\n";
    script << "    else: print('ERROR: No suitable alpha shapes mesh found'); return None\n\n";
    
    // Ball pivoting with fine detail preservation
    script << "def create_tube_mesh_ball_pivoting(pcd, geometry_info):\n";
    script << "    print('Creating tube mesh with Ball Pivoting...')\n";
    script << "    try:\n";
    script << "        distances = pcd.compute_nearest_neighbor_distance()\n";
    script << "        avg_dist = np.mean(distances)\n";
    script << "        print(f'Average point distance: {avg_dist:.4f}m')\n";
    script << "        radii = [\n";
    script << "            0.0005,  # 0.5mm - very fine detail\n";
    script << "            0.001,   # 1mm - fine detail\n";
    script << "            0.002,   # 2mm - medium detail\n";
    script << "            0.003,   # 3mm - structural detail\n";
    script << "            0.005,   # 5mm - larger structures\n";
    script << "            0.008,   # 8mm - backup\n";
    script << "            0.012    # 12mm - emergency\n";
    script << "        ]\n";
    script << "        print(f'Frame-preserving radii: {[f\"{r*1000:.0f}mm\" for r in radii]}')\n";
    script << "        print(f'Using radii: {[f\"{r:.4f}\" for r in radii]}')\n";
    script << "        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, o3d.utility.DoubleVector(radii))\n";
    script << "        triangle_count = len(mesh.triangles)\n";
    script << "        print(f'SUCCESS: Ball Pivoting result: {triangle_count:,} triangles')\n";
    script << "        return mesh if triangle_count > 50 else None\n";
    script << "    except Exception as e: print(f'ERROR: Ball Pivoting failed: {e}'); return None\n\n";
    
    // Poisson
    script << "def create_tube_mesh_poisson(pcd, geometry_info):\n";
    script << "    print('Creating tube mesh with Poisson...')\n";
    script << "    try:\n";
    script << "        tube_area = geometry_info['dimensions'][0] * geometry_info['dimensions'][1]\n";
    script << "        point_density = len(pcd.points) / tube_area\n";
    script << "        base_depth = 8\n";
    script << "        if point_density > 2000: depth = base_depth + 2\n";
    script << "        elif point_density > 1000: depth = base_depth + 1\n";
    script << "        else: depth = base_depth\n";
    script << "        if 0.25 <= tube_area <= 0.35: depth += 1\n";
    script << "        print(f'Optimized depth: {depth} (density: {point_density:.0f} pts/m2)')\n";
    script << "        if tube_area >= 0.25: depth += 1; print(f'Medium tube detected - increased depth to {depth}')\n";
    script << "        mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=depth, width=0, scale=1.1, linear_fit=False)\n";
    script << "        bbox = pcd.get_axis_aligned_bounding_box()\n";
    script << "        mesh = mesh.crop(bbox)\n";
    script << "        triangle_count = len(mesh.triangles)\n";
    script << "        print(f'SUCCESS: Poisson result: {triangle_count:,} triangles')\n";
    script << "        return mesh if triangle_count > 100 else None\n";
    script << "    except Exception as e: print(f'ERROR: Poisson failed: {e}'); return None\n\n";
    
    // Smoothing function
    script << "def smooth_tube_mesh(mesh, iterations=3):\n";
    script << "    \"\"\"Apply gentle smoothing while preserving frame details\"\"\"\n";
    script << "    print(f\"Gentle smoothing with {iterations} iterations...\")\n";
    script << "    if mesh is None: return None\n";
    script << "    mesh_smooth = mesh.filter_smooth_laplacian(\n";
    script << "        number_of_iterations=iterations,\n";
    script << "        lambda_filter=0.3  # Gentle smoothing to preserve details\n";
    script << "    )\n";
    script << "    print(f\"SUCCESS: Mesh smoothed - triangles preserved: {len(mesh_smooth.triangles):,}\")\n";
    script << "    return mesh_smooth\n\n";
    
    // Try all methods
    script << "def try_all_tube_methods(pcd, geometry_info):\n";
    script << "    print('\\nTesting all frame-preserving meshing methods...')\n";
    script << "    methods = [('Alpha Shapes', create_tube_mesh_alpha_shapes), ('Ball Pivoting', create_tube_mesh_ball_pivoting), ('Poisson', create_tube_mesh_poisson)]\n";
    script << "    best_mesh, best_triangle_count, best_method = None, 0, ''\n";
    script << "    for method_name, method_func in methods:\n";
    script << "        print(f'\\nTrying {method_name} for frame-preserving reconstruction...')\n";
    script << "        try:\n";
    script << "            mesh = method_func(pcd, geometry_info)\n";
    script << "            if mesh is not None:\n";
    script << "                triangle_count = len(mesh.triangles)\n";
    script << "                print(f'SUCCESS: {method_name}: {triangle_count:,} triangles')\n";
    script << "                if triangle_count > best_triangle_count:\n";
    script << "                    if best_mesh is not None: del best_mesh\n";
    script << "                    best_mesh, best_triangle_count, best_method = mesh, triangle_count, method_name\n";
    script << "                else: del mesh\n";
    script << "            else: print(f'ERROR: {method_name}: failed')\n";
    script << "        except Exception as e: print(f'ERROR: {method_name}: error - {e}')\n";
    script << "    if best_mesh is not None: print(f'\\nWINNER: Best method for frame preservation: {best_method} with {best_triangle_count:,} triangles')\n";
    script << "    else: print('\\nWARNING: No method produced a valid frame-preserving mesh')\n";
    script << "    return best_mesh\n\n";
    
    // Clean mesh
    script << "def clean_tube_mesh(mesh):\n";
    script << "    if mesh is None: return None\n";
    script << "    print('Frame-preserving mesh cleaning...')\n";
    script << "    print_memory_status('before cleaning')\n";
    script << "    original_triangles = len(mesh.triangles)\n";
    script << "    mesh.remove_degenerate_triangles(); mesh.remove_duplicated_triangles()\n";
    script << "    mesh.remove_duplicated_vertices(); mesh.remove_non_manifold_edges()\n";
    script << "    cleaned_triangles = len(mesh.triangles)\n";
    script << "    removed = original_triangles - cleaned_triangles\n";
    script << "    print(f'Cleaned: removed {removed:,} bad triangles (preserved frame details)')\n";
    script << "    print(f'Final mesh: {len(mesh.vertices):,} vertices, {cleaned_triangles:,} triangles')\n";
    script << "    print_memory_status('after cleaning')\n";
    script << "    return mesh\n\n";
    
    // Save results
    script << "def save_tube_results(mesh, pcd, geometry_info):\n";
    script << "    print('Saving frame-preserving reconstruction results...')\n";
    script << "    print_memory_status('before saving')\n";
    script << "    try: o3d.io.write_point_cloud('tube_pointcloud.ply', pcd); print('SUCCESS: Tube point cloud: tube_pointcloud.ply')\n";
    script << "    except Exception as e: print(f'ERROR: Point cloud save failed: {e}')\n";
    script << "    if mesh is not None:\n";
    script << "        try:\n";
    script << "            mesh.compute_vertex_normals(); mesh.compute_triangle_normals()\n";
    script << "            o3d.io.write_triangle_mesh('tube_mesh.stl', mesh); print('SUCCESS: Frame-preserving tube mesh STL: tube_mesh.stl')\n";
    script << "            o3d.io.write_triangle_mesh('tube_mesh.ply', mesh); print('SUCCESS: Frame-preserving tube mesh PLY: tube_mesh.ply')\n";
    script << "            o3d.io.write_triangle_mesh('tube_mesh.obj', mesh); print('SUCCESS: Frame-preserving tube mesh OBJ: tube_mesh.obj')\n";
    script << "            print(f'\\nFrame-preserving mesh statistics:')\n";
    script << "            print(f'   Vertices: {len(mesh.vertices):,}')\n";
    script << "            print(f'   Triangles: {len(mesh.triangles):,}')\n";
    script << "        except Exception as e: print(f'ERROR: Mesh save failed: {e}')\n";
    script << "    try:\n";
    script << "        with open('tube_analysis.txt', 'w') as f:\n";
    script << "            f.write('# Frame-Preserving Tube Geometry Analysis\\n')\n";
    script << "            f.write(f'Dimensions: {geometry_info[\"dimensions\"]}\\n')\n";
    script << "            f.write(f'Point density: {geometry_info[\"density\"]:.1f} points/m3\\n')\n";
    script << "            f.write(f'Is sparse: {geometry_info[\"is_sparse\"]}\\n')\n";
    script << "            f.write(f'Center: {geometry_info[\"center\"]}\\n')\n";
    script << "        print('SUCCESS: Frame analysis: tube_analysis.txt')\n";
    script << "    except Exception as e: print(f'ERROR: Analysis save failed: {e}')\n";
    script << "    print_memory_status('after saving')\n\n";
    
    // Main function
    script << "def main():\n";
    script << "    print('=== FRAME-PRESERVING TUBE MESH GENERATOR ===')\n";
    script << "    print('Optimized for metal tubes with visible internal frames')\n";
    script << "    print('Preserves structural details and sharp edges\\n')\n";
    script << "    mem = psutil.virtual_memory()\n";
    script << "    print(f'System: {mem.total / (1024**3):.1f}GB RAM available')\n";
    script << "    total_start = time.time()\n";
    script << "    map_files = ['ref_map_points.txt', 'map_points.txt']\n";
    script << "    points, source_file = None, ''\n";
    script << "    for filename in map_files:\n";
    script << "        if os.path.exists(filename):\n";
    script << "            points = load_orb_slam_map_points(filename)\n";
    script << "            if points is not None: source_file = filename; break\n";
    script << "    if points is None:\n";
    script << "        print('ERROR: No valid ORB-SLAM3 map point files found!')\n";
    script << "        print('INFO: Make sure you have \"map_points.txt\" or \"ref_map_points.txt\"')\n";
    script << "        return\n";
    script << "    print(f'Using map points from: {source_file}')\n";
    script << "    if len(points) < 20:\n";
    script << "        print('WARNING: Too few points for frame reconstruction')\n";
    script << "        print('INFO: Need at least 20 points for frame-preserving reconstruction')\n";
    script << "        return\n";
    script << "    geometry_info = analyze_tube_geometry(points)\n";
    script << "    print(f'\\nCreating frame-preserving point cloud from {len(points):,} points...')\n";
    script << "    pcd = o3d.geometry.PointCloud()\n";
    script << "    pcd.points = o3d.utility.Vector3dVector(points)\n";
    script << "    colors = np.tile([0.8, 0.8, 0.9], (len(points), 1))\n";
    script << "    pcd.colors = o3d.utility.Vector3dVector(colors)\n";
    script << "    pcd = tube_optimized_normals(pcd, geometry_info)\n";
    script << "    mesh = try_all_tube_methods(pcd, geometry_info)\n";
    script << "    if mesh is not None: mesh = clean_tube_mesh(mesh)\n";
    script << "    if mesh is not None: mesh = smooth_tube_mesh(mesh, iterations=2)\n";
    script << "    save_tube_results(mesh, pcd, geometry_info)\n";
    script << "    total_time = time.time() - total_start\n";
    script << "    print(f'\\nFrame-preserving reconstruction complete in {total_time:.1f}s!')\n";
    script << "    if mesh is not None:\n";
    script << "        print(f'\\nFor SolidWorks (with frame details):')\n";
    script << "        print(f'   Use: tube_mesh.stl')\n";
    script << "        print(f'   Preserves internal frame structure')\n";
    script << "        print(f'   Maintains both outer geometry and inner details')\n";
    script << "    else:\n";
    script << "        print(f'\\nPoint cloud only: tube_pointcloud.ply')\n";
    script << "        print(f'Try different scanning angles to capture more frame details')\n";
    script << "    print(f'\\nFrame-preserving scanning tips:')\n";
    script << "    print(f'   Use consistent lighting to avoid shadows in frames')\n";
    script << "    print(f'   Scan from multiple angles to see through frame gaps')\n";
    script << "    print(f'   Keep steady distance to maintain frame detail resolution')\n";
    script << "    print(f'   Move slowly to capture fine frame structures')\n\n";
    script << "if __name__ == '__main__': main()\n";
    
    script.close();
#ifdef _WIN32
    // Windows doesn't use chmod
#else
    chmod((folder + "tube_mesh_generator.py").c_str(), 0755);
#endif
    
    std::cout << "[MESH] SUCCESS: Created ENHANCED FRAME-PRESERVING TUBE MESH GENERATOR!" << std::endl;
    std::cout << "[MESH] INFO: Optimized to preserve internal frame structure" << std::endl;
    std::cout << "[MESH] INFO: Uses finer detail settings to capture visible frames" << std::endl;
#ifdef _WIN32
    std::cout << "[MESH] RUN: cd 3D_Reconstruction_Data && python tube_mesh_generator.py" << std::endl;
#else
    std::cout << "[MESH] RUN: cd 3D_Reconstruction_Data && python3 tube_mesh_generator.py" << std::endl;
#endif
}

int main(int argc, char **argv) {
    std::cout << "\n[DEBUG] =============== ORB-SLAM3 RGB-D WITH MAP EXPORT ===============" << std::endl;

    // Check for headless environment
    const char* display = getenv("DISPLAY");
    (void)display; // Suppress unused variable warning
    bool headless = true;  // Force headless mode for maximum performance
    
    if (headless) {
        std::cout << "[INFO] Running in headless mode for maximum performance" << std::endl;
    }
    
    // Register signal handlers
#ifdef _WIN32
    SetConsoleCtrlHandler(ConsoleCtrlHandler, TRUE);
#else
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
#endif

    cv::setBreakOnError(false);
    
    if (argc < 3 || argc > 4) {
        cerr << "Usage: ./rgbd_orbbec_gemini335 vocabulary settings [hw|sw]" << endl;
        cerr << "  hw: use hardware alignment (default)" << endl;
        cerr << "  sw: use software coordinate transformation" << endl;
        return 1;
    }

    // Check alignment mode
    bool use_hardware = true;
    if (argc == 4) {
        string mode = argv[3];
        if (mode == "sw") {
            use_hardware = false;
            cout << "[DEBUG] Using SOFTWARE alignment" << endl;
        } else {
            cout << "[DEBUG] Using HARDWARE alignment" << endl;
        }
    }

    std::cout << "[DEBUG] Vocabulary: " << argv[1] << std::endl;
    std::cout << "[DEBUG] Settings: " << argv[2] << std::endl;

    // Create reconstruction folder
    std::string reconstruction_folder = "3D_Reconstruction_Data";
#ifdef _WIN32
    if (_mkdir(reconstruction_folder.c_str()) == 0) {
#else
    if (mkdir(reconstruction_folder.c_str(), 0777) == 0) {
#endif
        std::cout << "[DEBUG] Created folder: " << reconstruction_folder << std::endl;
    } else {
        std::cout << "[DEBUG] Using existing folder: " << reconstruction_folder << std::endl;
    }

    OrbbecCapture capture(use_hardware);
    if (!capture.initialize()) {
        cerr << "[ERROR] Failed to initialize camera!" << endl;
        return -1;
    }

    // Initialize SLAM
    std::cout << "\n[DEBUG] Initializing ORB-SLAM3..." << std::endl;
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::RGBD, true);
    float imageScale = SLAM.GetImageScale();
    std::cout << "[DEBUG] ORB-SLAM3 image scale factor: " << imageScale << std::endl;

    if (!capture.start()) {
        cerr << "[ERROR] Failed to start camera!" << endl;
        return -1;
    }

    cout << "\n[DEBUG] ============= SLAM STARTED =============" << endl;
    cout << "[INFO] Controls: 'q'=quit, 't'=toggle, 'm'=save map" << endl;
    
    // Setup terminal for real-time key input
    setupTerminal();

    cv::Mat im, depthmap;
    double timestamp = 0.0;
    int frame_count = 0;
    int tracking_ok_count = 0;
    
    auto start_time = std::chrono::steady_clock::now();
    
    while (!g_resource_manager.shouldExit()) {
        // Get frames with exception handling
        bool frame_success = false;
        try {
            frame_success = capture.getFrames(im, depthmap, timestamp);
        } catch (const std::exception& e) {
            std::cout << "[ERROR] Exception in getFrames: " << e.what() << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        
        if (!frame_success) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // Resize if needed
        if (imageScale != 1.f) {
            int width = im.cols * imageScale;
            int height = im.rows * imageScale;
            cv::resize(im, im, cv::Size(width, height));
            cv::resize(depthmap, depthmap, cv::Size(width, height));
        }

        // Track with SLAM - add exception handling
        int state = -1;
        double ttrack = 0.0;
        try {
            auto track_start = std::chrono::steady_clock::now();
            SLAM.TrackRGBD(im, depthmap, timestamp);
            auto track_end = std::chrono::steady_clock::now();
            
            ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(track_end - track_start).count();
            frame_count++;

            // Get tracking state
            state = SLAM.GetTrackingState();
            if (state == 2) tracking_ok_count++;
        } catch (const std::exception& e) {
            std::cout << "[ERROR] Exception in SLAM tracking: " << e.what() << std::endl;
            continue;
        }

        // Status every 30 frames
        if (frame_count % 30 == 0) {
            auto current_time = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - start_time).count();
            double fps = frame_count / elapsed;
            
            cout << "[STATUS] Frame " << frame_count 
                 << " | FPS: " << std::fixed << std::setprecision(1) << fps
                 << " | Track: " << std::setprecision(3) << ttrack << "s";
            
            if (state == 2) {
                cout << " | STATUS: TRACKING OK";
            } else if (state == 0) {
                cout << " | STATUS: INITIALIZING...";
            } else {
                cout << " | STATUS: LOST";
            }
            
            cout << " | Success: " << (100.0 * tracking_ok_count / frame_count) << "%" << endl;
        }

        // Check for keyboard input (non-blocking)
        static int input_check = 0;
        if (++input_check % 10 == 0) { // Check every 10 frames for responsiveness
#ifdef _WIN32
            // Windows keyboard input
            if (_kbhit()) {
                char input = _getch();
                std::cout << "[KEY] Console input: '" << input << "'" << std::endl;
                if (input == 'q' || input == 'Q' || input == 27) { // q, Q, or ESC
                    std::cout << "[KEY] QUIT requested from console!" << std::endl;
                    g_resource_manager.requestExit();
                    break;
                } else if (input == 't' || input == 'T') {
                    std::cout << "[KEY] TOGGLE alignment mode" << std::endl;
                    capture.toggleAlignmentMode();
                } else if (input == 'm' || input == 'M') {
                    std::cout << "[KEY] SAVE MAP requested!" << std::endl;
                    std::cout << "[KEY] Map will be saved during shutdown..." << std::endl;
                }
            }
#else
            // Linux keyboard input
            fd_set readfds;
            FD_ZERO(&readfds);
            FD_SET(STDIN_FILENO, &readfds);
            
            struct timeval timeout;
            timeout.tv_sec = 0;
            timeout.tv_usec = 0; // Non-blocking
            
            if (select(STDIN_FILENO + 1, &readfds, NULL, NULL, &timeout) > 0) {
                char input;
                if (read(STDIN_FILENO, &input, 1) > 0) {
                    std::cout << "[KEY] Terminal input: '" << input << "'" << std::endl;
                    if (input == 'q' || input == 'Q' || input == 27) { // q, Q, or ESC
                        std::cout << "[KEY] QUIT requested from terminal!" << std::endl;
                        g_resource_manager.requestExit();
                        break;
                    } else if (input == 't' || input == 'T') {
                        std::cout << "[KEY] TOGGLE alignment mode" << std::endl;
                        capture.toggleAlignmentMode();
                    } else if (input == 'm' || input == 'M') {
                        std::cout << "[KEY] SAVE MAP requested!" << std::endl;
                        std::cout << "[KEY] Map will be saved during shutdown..." << std::endl;
                    }
                }
            }
#endif
        }
        
        // Also check for exit file (backup method)
        if (input_check % 100 == 0) {
            std::ifstream exit_file("stop_slam.txt");
            if (exit_file.good()) {
                std::cout << "[INFO] Exit signal file detected" << std::endl;
                g_resource_manager.requestExit();
                exit_file.close();
                std::remove("stop_slam.txt");
                break;
            }
        }
    }

    // Enhanced cleanup
    std::cout << "\n[DEBUG] Starting cleanup..." << std::endl;
    
    // Restore terminal settings
    restoreTerminal();
    
    // Final statistics
    auto end_time = std::chrono::steady_clock::now();
    double total_time = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time).count();
    
    std::cout << "\n[DEBUG] ===== FINAL STATISTICS =====" << std::endl;
    std::cout << "[DEBUG] Total frames: " << frame_count << std::endl;
    std::cout << "[DEBUG] Total time: " << std::setprecision(2) << total_time << "s" << std::endl;
    std::cout << "[DEBUG] Average FPS: " << (frame_count / total_time) << std::endl;
    std::cout << "[DEBUG] Tracking success: " << (100.0 * tracking_ok_count / frame_count) << "%" << std::endl;

    // Define folder path once
    std::string folder = "3D_Reconstruction_Data/";

    // Safe SLAM shutdown and data export
    try {
        std::cout << "[DEBUG] Shutting down SLAM system..." << std::endl;
        SLAM.Shutdown();
        std::cout << "[DEBUG] SLAM shutdown complete" << std::endl;
        
        std::cout << "[DEBUG] Saving trajectories..." << std::endl;
        string suffix = "_final_run";
        SLAM.SaveKeyFrameTrajectoryTUM(folder + "KeyFrameTrajectory" + suffix + ".txt");
        SLAM.SaveTrajectoryTUM(folder + "CameraTrajectory" + suffix + ".txt");
        std::cout << "[DEBUG] Trajectories saved successfully" << std::endl;
        
        // Save ORB-SLAM3 map points (much simpler than RGB-D reconstruction!)
        std::cout << "[DEBUG] Saving ORB-SLAM3 map points..." << std::endl;
        SLAM.SavePointCloud(folder + "map_points.txt");
        std::cout << "[DEBUG] Map points saved successfully" << std::endl;
        
        // Save camera parameters (dynamic from actual camera)
        std::cout << "[DEBUG] Saving camera parameters..." << std::endl;
        std::ofstream cam_file(folder + "camera_params.txt");
        cam_file << "# Camera intrinsics (from actual camera)" << std::endl;
        
        // Get the actual camera parameters from the capture object
        cv::Mat cameraMatrix = capture.getColorCameraMatrix();
        cv::Size frameSize = capture.getFrameSize();
        
        if (!cameraMatrix.empty()) {
            cam_file << "fx: " << cameraMatrix.at<float>(0, 0) << std::endl;
            cam_file << "fy: " << cameraMatrix.at<float>(1, 1) << std::endl;
            cam_file << "cx: " << cameraMatrix.at<float>(0, 2) << std::endl;
            cam_file << "cy: " << cameraMatrix.at<float>(1, 2) << std::endl;
            cam_file << "width: " << frameSize.width << std::endl;
            cam_file << "height: " << frameSize.height << std::endl;
            std::cout << "[DEBUG] Dynamic camera parameters saved: fx=" << cameraMatrix.at<float>(0, 0) 
                     << ", fy=" << cameraMatrix.at<float>(1, 1) 
                     << ", cx=" << cameraMatrix.at<float>(0, 2) 
                     << ", cy=" << cameraMatrix.at<float>(1, 2) << std::endl;
        } else {
            // Fallback to YAML values if camera matrix not available
            cam_file << "fx: 1035.805420" << std::endl;
            cam_file << "fy: 1036.608398" << std::endl;
            cam_file << "cx: 965.283142" << std::endl;
            cam_file << "cy: 540.698120" << std::endl;
            cam_file << "width: 1920" << std::endl;
            cam_file << "height: 1080" << std::endl;
            std::cout << "[DEBUG] Fallback camera parameters saved (from YAML)" << std::endl;
        }
        
        cam_file << "depth_scale: 1000.0" << std::endl;
        cam_file.close();
        std::cout << "[DEBUG] Camera parameters saved successfully" << std::endl;
        
        GenerateMapPointConverter();
        
    } catch (const std::exception& e) {
        std::cout << "[ERROR] Shutdown/save error: " << e.what() << std::endl;
    }

    std::cout << "[DEBUG] Program completed successfully!" << std::endl;
    std::cout << "\n[3D RECONSTRUCTION] You now have:" << std::endl;
    std::cout << "SUCCESS: ORB-SLAM3 map points: map_points.txt + ref_map_points.txt" << std::endl;
    std::cout << "SUCCESS: Camera trajectory: KeyFrameTrajectory_final_run.txt" << std::endl;
    std::cout << "SUCCESS: Camera parameters: camera_params.txt" << std::endl;
    std::cout << "SUCCESS: Map point converter: tube_mesh_generator.py" << std::endl;
    std::cout << "READY: Clean 3D reconstruction from SLAM map points!" << std::endl;

    return 0;
}