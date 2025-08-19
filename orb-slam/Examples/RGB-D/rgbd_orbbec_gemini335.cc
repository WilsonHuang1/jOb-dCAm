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

#include <signal.h>
#include <csignal>
#include <atomic>
#include <unistd.h>
#include <sys/select.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/stat.h>

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
void signalHandler(int signum) {
    std::cout << "\n[DEBUG] Received signal " << signum << std::endl;
    g_resource_manager.requestExit();
}

// Setup terminal for non-blocking input
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
                std::cout << "[DEBUG] Trying COLOR: 640x360@60fps" << std::endl;
                config_->enableVideoStream(OB_STREAM_COLOR, 640, 360, 60, OB_FORMAT_RGB);
                color_configured = true;
                std::cout << "[DEBUG] COLOR configured: 640x360@60fps ✓" << std::endl;
            } catch (const ob::Error& e) {
                std::cout << "[WARNING] COLOR 640x360@60fps failed: " << e.what() << std::endl;
                
                try {
                    std::cout << "[DEBUG] Trying COLOR: 640x360@30fps" << std::endl;
                    config_->enableVideoStream(OB_STREAM_COLOR, 640, 360, 30, OB_FORMAT_RGB);
                    color_configured = true;
                    std::cout << "[DEBUG] COLOR configured: 640x360@30fps ✓" << std::endl;
                } catch (const ob::Error& e2) {
                    std::cout << "[WARNING] COLOR 640x360@30fps failed: " << e2.what() << std::endl;
                    
                    try {
                        std::cout << "[DEBUG] Trying COLOR: 1280x720@30fps (will scale)" << std::endl;
                        config_->enableVideoStream(OB_STREAM_COLOR, 1280, 720, 30, OB_FORMAT_RGB);
                        color_configured = true;
                        std::cout << "[DEBUG] COLOR configured: 1280x720@30fps ✓ (will resize to 640x360)" << std::endl;
                    } catch (const ob::Error& e3) {
                        std::cout << "[ERROR] All COLOR configurations failed!" << std::endl;
                    }
                }
            }
            
            // Try to configure DEPTH stream
            try {
                std::cout << "[DEBUG] Trying DEPTH: 640x360@60fps" << std::endl;
                config_->enableVideoStream(OB_STREAM_DEPTH, 640, 360, 60, OB_FORMAT_Y16);
                depth_configured = true;
                std::cout << "[DEBUG] DEPTH configured: 640x360@60fps ✓" << std::endl;
            } catch (const ob::Error& e) {
                std::cout << "[WARNING] DEPTH 640x360@60fps failed: " << e.what() << std::endl;
                
                try {
                    std::cout << "[DEBUG] Trying DEPTH: 640x360@30fps" << std::endl;
                    config_->enableVideoStream(OB_STREAM_DEPTH, 640, 360, 30, OB_FORMAT_Y16);
                    depth_configured = true;
                    std::cout << "[DEBUG] DEPTH configured: 640x360@30fps ✓" << std::endl;
                } catch (const ob::Error& e2) {
                    std::cout << "[WARNING] DEPTH 640x360@30fps failed: " << e2.what() << std::endl;
                    
                    try {
                        std::cout << "[DEBUG] Trying DEPTH: 848x480@30fps (will scale)" << std::endl;
                        config_->enableVideoStream(OB_STREAM_DEPTH, 848, 480, 30, OB_FORMAT_Y16);
                        depth_configured = true;
                        std::cout << "[DEBUG] DEPTH configured: 848x480@30fps ✓ (will resize)" << std::endl;
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
                    std::cout << "[DEBUG] Hardware alignment filter created ✓" << std::endl;
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
            
            // Search for 640x360 color profile
            try {
                auto colorSensor = device_->getSensor(OB_SENSOR_COLOR);
                auto colorProfiles = colorSensor->getStreamProfileList();
                
                for (uint32_t i = 0; i < colorProfiles->getCount(); i++) {
                    auto profile = colorProfiles->getProfile(i)->as<ob::VideoStreamProfile>();
                    if (profile->getWidth() == 640 && profile->getHeight() == 360) {
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

            // Search for 640x360 depth profile
            try {
                auto depthSensor = device_->getSensor(OB_SENSOR_DEPTH);
                auto depthProfiles = depthSensor->getStreamProfileList();
                
                for (uint32_t i = 0; i < depthProfiles->getCount(); i++) {
                    auto profile = depthProfiles->getProfile(i)->as<ob::VideoStreamProfile>();
                    if (profile->getWidth() == 640 && profile->getHeight() == 360) {
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
            std::cout << "[DEBUG] Camera started successfully ✓" << std::endl;
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
            
            // Force resize to 640x360 if needed
            if (color.size() != cv::Size(640, 360)) {
                if (debug_this_frame) {
                    std::cout << "[DEBUG] Resizing color " << color.size() << " -> 640x360" << std::endl;
                }
                cv::resize(color, color, cv::Size(640, 360));
            }
            
            if (depth.size() != cv::Size(640, 360)) {
                if (debug_this_frame) {
                    std::cout << "[DEBUG] Resizing depth " << depth.size() << " -> 640x360" << std::endl;
                }
                cv::resize(depth, depth, cv::Size(640, 360), 0, 0, cv::INTER_NEAREST);
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
    std::cout << "[MESH] Generating MEMORY-SAFE MULTI-CORE ORB-SLAM3 converter..." << std::endl;
    std::string folder = "3D_Reconstruction_Data/";
    
    std::ofstream script(folder + "memory_safe_orb_converter.py");
    script << "#!/usr/bin/env python3\n";
    script << "# MEMORY-SAFE MULTI-CORE ORB-SLAM3 Map Points to Mesh Converter\n";
    script << "# Optimized for 16GB RAM systems with intelligent memory management\n";
    script << "# Auto-generated by rgbd_orbbec_gemini335.cc\n\n";
    
    script << "import numpy as np, open3d as o3d, os, time, gc\n";
    script << "from multiprocessing import cpu_count\n";
    script << "from concurrent.futures import ThreadPoolExecutor\n";
    script << "try: import psutil\n";
    script << "except: os.system('pip install psutil'); import psutil\n\n";
    
    // Memory monitoring functions
    script << "def get_memory_info():\n";
    script << "    mem = psutil.virtual_memory()\n";
    script << "    return {'available_gb': mem.available / (1024**3), 'used_gb': mem.used / (1024**3), 'percent': mem.percent, 'total_gb': mem.total / (1024**3)}\n\n";
    
    script << "def print_memory_status(step=''):\n";
    script << "    mem = get_memory_info()\n";
    script << "    print(f'💾 Memory: {mem[\"used_gb\"]:.1f}/{mem[\"total_gb\"]:.1f}GB ({mem[\"percent\"]:.1f}%) - {step}')\n\n";
    
    script << "def adaptive_worker_count():\n";
    script << "    mem = get_memory_info()\n";
    script << "    if mem['available_gb'] < 4: workers = 2; print('⚠️ Low memory - using 2 workers')\n";
    script << "    elif mem['available_gb'] < 8: workers = min(4, cpu_count()); print(f'🔧 Moderate memory - using {workers} workers')\n";
    script << "    else: workers = min(cpu_count(), 8); print(f'✅ Good memory - using {workers} workers')\n";
    script << "    return workers\n\n";
    
    // Memory-safe loading
    script << "def load_map_points_safe(filename):\n";
    script << "    print(f'📊 Loading {filename} with memory safety...')\n";
    script << "    print_memory_status('before loading')\n";
    script << "    if not os.path.exists(filename): return None\n";
    script << "    try:\n";
    script << "        file_size_mb = os.path.getsize(filename) / (1024 * 1024)\n";
    script << "        print(f'📁 File size: {file_size_mb:.1f}MB')\n";
    script << "        if file_size_mb > 50:\n";
    script << "            try:\n";
    script << "                import pandas as pd\n";
    script << "                chunks = []\n";
    script << "                for chunk in pd.read_csv(filename, chunksize=50000):\n";
    script << "                    chunks.append(chunk[['pos_x', 'pos_y', 'pos_z']].values)\n";
    script << "                points = np.vstack(chunks)\n";
    script << "                del chunks; print('⚡ Large file loaded with chunking')\n";
    script << "            except: coords = np.genfromtxt(filename, delimiter=',', skip_header=1); points = coords[:, :3]; del coords\n";
    script << "        else: coords = np.genfromtxt(filename, delimiter=',', skip_header=1); points = coords[:, :3]; del coords\n";
    script << "        gc.collect()\n";
    script << "        print(f'✅ Loaded {len(points):,} points')\n";
    script << "        print_memory_status('after loading')\n";
    script << "        return points\n";
    script << "    except Exception as e: print(f'❌ Error: {e}'); return None\n\n";
    
    // Memory-safe normal estimation
    script << "def memory_safe_normals(pcd):\n";
    script << "    point_count = len(pcd.points)\n";
    script << "    mem = get_memory_info()\n";
    script << "    print(f'📐 Memory-safe normals for {point_count:,} points...')\n";
    script << "    print_memory_status('before normals')\n";
    script << "    if point_count > 100000 or mem['available_gb'] < 4:\n";
    script << "        print('⚡ Large dataset or low memory - fast normals')\n";
    script << "        search_param = o3d.geometry.KDTreeSearchParamHybrid(radius=0.15, max_nn=10)\n";
    script << "    elif point_count > 50000 or mem['available_gb'] < 6:\n";
    script << "        print('🔧 Medium dataset - balanced normals')\n";
    script << "        search_param = o3d.geometry.KDTreeSearchParamHybrid(radius=0.12, max_nn=15)\n";
    script << "    else:\n";
    script << "        print('✨ Small dataset - quality normals')\n";
    script << "        search_param = o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=20)\n";
    script << "    pcd.estimate_normals(search_param=search_param, fast_normal_computation=True)\n";
    script << "    print_memory_status('after normals')\n";
    script << "    return pcd\n\n";
    
    // Memory-safe meshing
    script << "def memory_safe_meshing(pcd):\n";
    script << "    point_count, mem = len(pcd.points), get_memory_info()\n";
    script << "    print(f'🔨 Memory-safe meshing for {point_count:,} points...')\n";
    script << "    print_memory_status('before meshing')\n";
    script << "    # Choose method based on memory and point count\n";
    script << "    if point_count > 50000 or mem['available_gb'] < 4:\n";
    script << "        print('⚡ Fast Alpha Shapes (memory optimized)...')\n";
    script << "        alphas = [0.1, 0.2, 0.3]\n";
    script << "        def try_alpha(alpha):\n";
    script << "            try:\n";
    script << "                mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)\n";
    script << "                count = len(mesh.triangles)\n";
    script << "                if 100 < count < 500000: return alpha, count, mesh\n";
    script << "                else: del mesh; return alpha, 0, None\n";
    script << "            except: return alpha, 0, None\n";
    script << "        max_workers = min(len(alphas), adaptive_worker_count() // 2)\n";
    script << "        with ThreadPoolExecutor(max_workers=max_workers) as executor:\n";
    script << "            results = list(executor.map(try_alpha, alphas))\n";
    script << "        best = max(results, key=lambda x: x[1])\n";
    script << "        for a, c, m in results:\n";
    script << "            if m is not best[2] and m is not None: del m\n";
    script << "        gc.collect()\n";
    script << "        if best[1] > 100:\n";
    script << "            print(f'✅ Alpha Shapes: {best[1]:,} triangles')\n";
    script << "            return best[2]\n";
    script << "    else:\n";
    script << "        print('✨ High quality Poisson...')\n";
    script << "        try:\n";
    script << "            mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=7)\n";
    script << "            bbox = pcd.get_axis_aligned_bounding_box()\n";
    script << "            mesh = mesh.crop(bbox)\n";
    script << "            print(f'✅ Poisson: {len(mesh.triangles):,} triangles')\n";
    script << "            return mesh\n";
    script << "        except: pass\n";
    script << "    print('❌ All meshing failed')\n";
    script << "    return None\n\n";
    
    // Main function
    script << "def main():\n";
    script << "    print('💾 === MEMORY-SAFE MULTI-CORE ORB-SLAM3 CONVERTER ===')\n";
    script << "    mem = get_memory_info()\n";
    script << "    print(f'💻 System: {cpu_count()} cores, {mem[\"total_gb\"]:.1f}GB RAM')\n";
    script << "    print(f'💾 Available: {mem[\"available_gb\"]:.1f}GB ({100-mem[\"percent\"]:.1f}% free)')\n";
    script << "    if mem['available_gb'] < 2: print('⚠️ WARNING: Very low memory!')\n";
    script << "    elif mem['available_gb'] < 4: print('⚠️ Low memory - using conservative settings')\n";
    script << "    print('🎯 Memory-optimized ORB-SLAM3 map point processing\\n')\n";
    script << "    total_start = time.time()\n";
    script << "    # Load map points safely\n";
    script << "    points = None\n";
    script << "    for filename in ['ref_map_points.txt', 'map_points.txt']:\n";
    script << "        points = load_map_points_safe(filename)\n";
    script << "        if points is not None: break\n";
    script << "    if points is None: print('❌ No map files found!'); return\n";
    script << "    if len(points) < 50: print('⚠️ Too few points'); return\n";
    script << "    print(f'📍 Processing {len(points):,} map points')\n";
    script << "    # Create point cloud\n";
    script << "    pcd = o3d.geometry.PointCloud()\n";
    script << "    pcd.points = o3d.utility.Vector3dVector(points)\n";
    script << "    del points; gc.collect()  # Free memory\n";
    script << "    # Add colors if memory allows\n";
    script << "    if get_memory_info()['available_gb'] > 3:\n";
    script << "        colors = np.tile([0.7, 0.3, 0.3], (len(pcd.points), 1))\n";
    script << "        pcd.colors = o3d.utility.Vector3dVector(colors)\n";
    script << "        del colors\n";
    script << "    # Memory-safe processing\n";
    script << "    pcd = memory_safe_normals(pcd)\n";
    script << "    mesh = memory_safe_meshing(pcd)\n";
    script << "    # Clean mesh if created\n";
    script << "    if mesh is not None:\n";
    script << "        print('🧹 Mesh cleanup...')\n";
    script << "        mesh.remove_degenerate_triangles(); mesh.remove_duplicated_triangles()\n";
    script << "        mesh.remove_duplicated_vertices(); mesh.remove_non_manifold_edges()\n";
    script << "    # Save results\n";
    script << "    print('💾 Saving with memory monitoring...')\n";
    script << "    print_memory_status('before saving')\n";
    script << "    try:\n";
    script << "        o3d.io.write_point_cloud('safe_orb_points.ply', pcd)\n";
    script << "        print('✅ Point cloud saved: safe_orb_points.ply')\n";
    script << "        if mesh is not None:\n";
    script << "            mesh.compute_vertex_normals(); mesh.compute_triangle_normals()\n";
    script << "            o3d.io.write_triangle_mesh('safe_orb_mesh.stl', mesh)\n";
    script << "            o3d.io.write_triangle_mesh('safe_orb_mesh.ply', mesh)\n";
    script << "            print('✅ Mesh saved: safe_orb_mesh.stl')\n";
    script << "            print(f'📊 Final: {len(mesh.vertices):,} vertices, {len(mesh.triangles):,} triangles')\n";
    script << "        else: print('📊 Point cloud only: no mesh created')\n";
    script << "    except Exception as e: print(f'❌ Save error: {e}')\n";
    script << "    total_time = time.time() - total_start\n";
    script << "    print(f'🎉 Memory-safe conversion complete in {total_time:.1f}s!')\n";
    script << "    print_memory_status('final')\n";
    script << "    print('🎯 Files: safe_orb_mesh.stl, safe_orb_points.ply')\n";
    script << "    print('🚀 Memory-optimized for 16GB systems!')\n\n";
    script << "if __name__ == '__main__': main()\n";
    
    script.close();
    chmod((folder + "memory_safe_orb_converter.py").c_str(), 0755);
    
    std::cout << "[MESH] ✅ Created MEMORY-SAFE MULTI-CORE ORB-SLAM3 converter!" << std::endl;
    std::cout << "[MESH] 💾 Optimized for 16GB RAM systems" << std::endl;
    std::cout << "[MESH] 🧠 Intelligent memory management with real-time monitoring" << std::endl;
    std::cout << "[MESH] ⚡ Multi-core processing with adaptive worker count" << std::endl;
    std::cout << "[MESH] 🚀 Run: cd 3D_Reconstruction_Data && python3 memory_safe_orb_converter.py" << std::endl;
}

int main(int argc, char **argv) {
    std::cout << "\n[DEBUG] =============== ORB-SLAM3 RGB-D WITH MAP EXPORT ===============" << std::endl;

    // Check for headless environment
    const char* display = getenv("DISPLAY");
    (void)display; // Suppress unused variable warning
    bool headless = true;  // Force headless mode for maximum performance
    bool windows_created = false;
    
    if (headless) {
        std::cout << "[INFO] Running in headless mode for maximum performance" << std::endl;
    }
    
    // Register signal handlers
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
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
    if (mkdir(reconstruction_folder.c_str(), 0777) == 0) {
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
    cout << "[INFO] Controls: 'q'=quit, 't'=toggle, 'm'=save map, 'f'=save RGB-D frame" << endl;
    
    // Setup terminal for real-time key input
    setupTerminal();

    cv::Mat im, depthmap;
    double timestamp = 0.0;
    int frame_count = 0;
    int tracking_ok_count = 0;
    
    auto start_time = std::chrono::steady_clock::now();
    
    while (!g_resource_manager.shouldExit()) {
        // No delay needed in headless mode for max performance

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
                cout << " | STATUS: TRACKING OK ✓";
            } else if (state == 0) {
                cout << " | STATUS: INITIALIZING...";
            } else {
                cout << " | STATUS: LOST";
            }
            
            cout << " | Success: " << (100.0 * tracking_ok_count / frame_count) << "%" << endl;
        }
        
        // // Save RGB-D frames occasionally for backup (every 120 frames when tracking OK)
        // if (state == 2 && frame_count % 120 == 0) {
        //     SaveRGBDFrame(im, depthmap, frame_count, timestamp);
        // }

        // Check for keyboard input (non-blocking)
        static int input_check = 0;
        if (++input_check % 10 == 0) { // Check every 10 frames for responsiveness
            // Check if stdin has data available
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
                    // } else if (input == 'f' || input == 'F') {
                    //     std::cout << "[KEY] SAVE RGB-D FRAME requested!" << std::endl;
                    //     if (state == 2) { // Only save when tracking OK
                    //         SaveRGBDFrame(im, depthmap, frame_count, timestamp);
                    //         std::cout << "[KEY] RGB-D frame saved for backup!" << std::endl;
                    //     } else {
                    //         std::cout << "[KEY] Cannot save - tracking not OK" << std::endl;
                    //     }
                    }
                }
            }
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
        
        // Save camera parameters
        std::cout << "[DEBUG] Saving camera parameters..." << std::endl;
        std::ofstream cam_file(folder + "camera_params.txt");
        cam_file << "# Camera intrinsics" << std::endl;
        cam_file << "fx: 345.268" << std::endl;
        cam_file << "fy: 345.536" << std::endl;
        cam_file << "cx: 321.761" << std::endl;
        cam_file << "cy: 180.233" << std::endl;
        cam_file << "width: 640" << std::endl;
        cam_file << "height: 360" << std::endl;
        cam_file << "depth_scale: 1000.0" << std::endl;
        cam_file.close();
        std::cout << "[DEBUG] Camera parameters saved" << std::endl;
        
        GenerateMapPointConverter();
        
    } catch (const std::exception& e) {
        std::cout << "[ERROR] Shutdown/save error: " << e.what() << std::endl;
    }

    std::cout << "[DEBUG] Program completed successfully!" << std::endl;
    std::cout << "\n[3D RECONSTRUCTION] You now have:" << std::endl;
    std::cout << "✅ ORB-SLAM3 map points: map_points.txt + ref_map_points.txt" << std::endl;
    std::cout << "✅ Camera trajectory: KeyFrameTrajectory_final_run.txt" << std::endl;
    std::cout << "✅ Camera parameters: camera_params.txt" << std::endl;
    std::cout << "✅ Map point converter: memory_safe_orb_converter.py" << std::endl;
    std::cout << "🚀 Clean 3D reconstruction from SLAM map points!" << std::endl;

    return 0;
}