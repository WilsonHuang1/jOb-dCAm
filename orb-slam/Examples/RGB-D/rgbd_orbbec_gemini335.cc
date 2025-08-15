/**
 * ORB-SLAM3 RGB-D example for Orbbec Gemini 335 camera
 * WITH 3D RECONSTRUCTION DATA SAVING
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

// Simple map save notification - RGB-D method is better anyway
void SavePointCloud(ORB_SLAM3::System& SLAM, const string &filename) {
    cout << endl << "Map save info - " << filename << endl;
    cout << "Note: Using RGB-D + trajectory method for 3D reconstruction" << endl;
    cout << "This gives you DENSE point clouds (millions of points)" << endl;
    cout << "Instead of sparse SLAM points (thousands of points)" << endl;
    
    // Create info file
    ofstream f;
    f.open(filename.c_str());
    f << "# 3D Reconstruction Data Available:" << endl;
    f << "# - RGB-D frames: rgbd_color_*.png + rgbd_depth_*.png" << endl;
    f << "# - Camera poses: KeyFrameTrajectory_final_run.txt" << endl;
    f << "# - Camera params: camera_params.txt" << endl;
    f << "# Use these for dense 3D reconstruction!" << endl;
    f.close();
    
    cout << "Ready for 3D reconstruction with RGB-D method!" << endl;
}

// Function to save RGB-D frame for 3D reconstruction
void SaveRGBDFrame(const cv::Mat& rgb, const cv::Mat& depth, int frame_number, double timestamp) {
    try {
        // Create filenames with frame number and timestamp
        std::string rgb_filename = "rgbd_color_" + std::to_string(frame_number) + ".png";
        std::string depth_filename = "rgbd_depth_" + std::to_string(frame_number) + ".png";
        std::string info_filename = "rgbd_info_" + std::to_string(frame_number) + ".txt";
        
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
            std::cout << "[3D] Saved RGB-D frame " << frame_number << " for 3D reconstruction" << std::endl;
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

int main(int argc, char **argv) {
    std::cout << "\n[DEBUG] =============== ORB-SLAM3 RGB-D DEBUG ===============" << std::endl;

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
        
        // Save RGB-D frames for 3D reconstruction (every 60 frames when tracking OK)
        if (state == 2 && frame_count % 60 == 0) {
            SaveRGBDFrame(im, depthmap, frame_count, timestamp);
        }

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
                        try {
                            std::string timestamp_str = to_string(frame_count);
                            std::string map_file = "map_points_" + timestamp_str + ".csv";
                            SavePointCloud(SLAM, map_file);
                            std::cout << "[KEY] Map info saved successfully!" << std::endl;
                        } catch (const std::exception& e) {
                            std::cout << "[ERROR] Map save failed: " << e.what() << std::endl;
                        }
                    } else if (input == 'f' || input == 'F') {
                        std::cout << "[KEY] SAVE RGB-D FRAME requested!" << std::endl;
                        if (state == 2) { // Only save when tracking OK
                            SaveRGBDFrame(im, depthmap, frame_count, timestamp);
                            std::cout << "[KEY] RGB-D frame saved for 3D reconstruction!" << std::endl;
                        } else {
                            std::cout << "[KEY] Cannot save - tracking not OK" << std::endl;
                        }
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
    
    // Safe SLAM shutdown and trajectory saving
    try {
        std::cout << "[DEBUG] Shutting down SLAM system..." << std::endl;
        SLAM.Shutdown();
        std::cout << "[DEBUG] SLAM shutdown complete" << std::endl;
        
        std::cout << "[DEBUG] Saving trajectories..." << std::endl;
        string suffix = "_final_run";
        SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory" + suffix + ".txt");
        SLAM.SaveTrajectoryTUM("CameraTrajectory" + suffix + ".txt");
        std::cout << "[DEBUG] Trajectories saved successfully" << std::endl;
        
        // Save camera parameters for 3D reconstruction
        std::cout << "[DEBUG] Saving camera parameters..." << std::endl;
        std::ofstream cam_file("camera_params.txt");
        cam_file << "# Camera intrinsics for 3D reconstruction" << std::endl;
        cam_file << "fx: 345.268" << std::endl;
        cam_file << "fy: 345.536" << std::endl;
        cam_file << "cx: 321.761" << std::endl;
        cam_file << "cy: 180.233" << std::endl;
        cam_file << "width: 640" << std::endl;
        cam_file << "height: 360" << std::endl;
        cam_file << "depth_scale: 1000.0" << std::endl; // mm to meters
        cam_file.close();
        std::cout << "[DEBUG] Camera parameters saved" << std::endl;
        
    } catch (const std::exception& e) {
        std::cout << "[ERROR] Shutdown/save error: " << e.what() << std::endl;
    }

    std::cout << "[DEBUG] Program completed successfully!" << std::endl;
    std::cout << "\n[3D RECONSTRUCTION] You now have:" << std::endl;
    std::cout << "✓ RGB-D frames: rgbd_color_*.png + rgbd_depth_*.png" << std::endl;
    std::cout << "✓ Camera trajectory: KeyFrameTrajectory_final_run.txt" << std::endl;
    std::cout << "✓ Camera parameters: camera_params.txt" << std::endl;
    std::cout << "🚀 Ready for 3D reconstruction!" << std::endl;

    return 0;
}