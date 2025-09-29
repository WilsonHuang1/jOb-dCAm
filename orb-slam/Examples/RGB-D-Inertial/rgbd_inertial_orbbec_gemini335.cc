/**
 * ORB-SLAM3 RGB-D with IMU for Orbbec Gemini 335 camera
 * Based on working RGB-D code with IMU integration
 */

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <thread>
#include <iomanip>
#include <set>
#include <queue>
#include <mutex>
#include <atomic>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include <System.h>
#include <ImuTypes.h>

// Orbbec SDK includes
#include "libobsensor/ObSensor.hpp"

#include <signal.h>
#include <csignal>
#include <unistd.h>
#include <sys/select.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/stat.h>

using namespace std;

// YAML file verification function
bool verifyYAMLFile(const std::string& filename) {
    cv::FileStorage fs;
    try {
        fs.open(filename, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            std::cout << "[ERROR] Cannot open YAML file: " << filename << std::endl;
            return false;
        }
        
        // Test reading a simple parameter
        float test_fx;
        fs["Camera.fx"] >> test_fx;
        
        if (test_fx > 0) {
            std::cout << "[DEBUG] YAML file readable, Camera.fx = " << test_fx << std::endl;
            fs.release();
            return true;
        } else {
            std::cout << "[ERROR] YAML file format error - cannot read Camera.fx" << std::endl;
            fs.release();
            return false;
        }
    } catch (const cv::Exception& e) {
        std::cout << "[ERROR] YAML parsing exception: " << e.what() << std::endl;
        if (fs.isOpened()) fs.release();
        return false;
    }
}

// Synchronized IMU data buffer
class ImuSyncBuffer {
private:
    struct ImuMeasurement {
        Eigen::Vector3f accel;
        Eigen::Vector3f gyro;
        double accel_time;
        double gyro_time;
        bool has_accel = false;
        bool has_gyro = false;
    };
    
    std::queue<ImuMeasurement> pending_measurements_;
    std::mutex buffer_mutex_;
    
public:
    void addAccel(const Eigen::Vector3f& accel, double timestamp) {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        
        const double TIME_TOLERANCE = 0.01;
        auto temp_queue = pending_measurements_;
        std::queue<ImuMeasurement> new_queue;
        
        bool paired = false;
        while (!temp_queue.empty()) {
            auto meas = temp_queue.front();
            temp_queue.pop();
            
            if (!meas.has_accel && meas.has_gyro && 
                std::abs(meas.gyro_time - timestamp) < TIME_TOLERANCE) {
                meas.accel = accel;
                meas.accel_time = timestamp;
                meas.has_accel = true;
                paired = true;
            }
            new_queue.push(meas);
        }
        
        pending_measurements_ = new_queue;
        
        if (!paired) {
            ImuMeasurement new_meas;
            new_meas.accel = accel;
            new_meas.accel_time = timestamp;
            new_meas.has_accel = true;
            pending_measurements_.push(new_meas);
        }
    }
    
    void addGyro(const Eigen::Vector3f& gyro, double timestamp) {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        
        const double TIME_TOLERANCE = 0.01;
        auto temp_queue = pending_measurements_;
        std::queue<ImuMeasurement> new_queue;
        
        bool paired = false;
        while (!temp_queue.empty()) {
            auto meas = temp_queue.front();
            temp_queue.pop();
            
            if (meas.has_accel && !meas.has_gyro && 
                std::abs(meas.accel_time - timestamp) < TIME_TOLERANCE) {
                meas.gyro = gyro;
                meas.gyro_time = timestamp;
                meas.has_gyro = true;
                paired = true;
            }
            new_queue.push(meas);
        }
        
        pending_measurements_ = new_queue;
        
        if (!paired) {
            ImuMeasurement new_meas;
            new_meas.gyro = gyro;
            new_meas.gyro_time = timestamp;
            new_meas.has_gyro = true;
            pending_measurements_.push(new_meas);
        }
    }
    
    std::vector<ORB_SLAM3::IMU::Point> getCompleteMeasurements() {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        std::vector<ORB_SLAM3::IMU::Point> complete_measurements;
        
        std::queue<ImuMeasurement> remaining;
        double current_time = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        
        while (!pending_measurements_.empty()) {
            auto meas = pending_measurements_.front();
            pending_measurements_.pop();
            
            if (meas.has_accel && meas.has_gyro) {
                double avg_timestamp = (meas.accel_time + meas.gyro_time) / 2.0;
                
                ORB_SLAM3::IMU::Point imu_point(
                    meas.accel.x(), meas.accel.y(), meas.accel.z(),
                    meas.gyro.x(), meas.gyro.y(), meas.gyro.z(),
                    avg_timestamp
                );
                
                complete_measurements.push_back(imu_point);
            } else {
                double meas_time = meas.has_accel ? meas.accel_time : meas.gyro_time;
                if (current_time - meas_time < 0.5) {
                    remaining.push(meas);
                }
            }
        }
        
        pending_measurements_ = remaining;
        return complete_measurements;
    }
};

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
    term.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &term);
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
}

void restoreTerminal() {
    struct termios term;
    tcgetattr(STDIN_FILENO, &term);
    term.c_lflag |= (ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &term);
    fcntl(STDIN_FILENO, F_SETFL, 0);
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

// Add this function before OrbbecCapture class
bool verifyPipelineState(std::shared_ptr<ob::Pipeline> pipeline, const std::string& name) {
    if (!pipeline) {
        std::cout << "[ERROR] " << name << " pipeline is null" << std::endl;
        return false;
    }
    
    try {
        // Try to get a profile list to verify pipeline is valid
        auto profiles = pipeline->getStreamProfileList(OB_SENSOR_COLOR);
        if (!profiles || profiles->getCount() == 0) {
            std::cout << "[WARNING] " << name << " pipeline has no profiles" << std::endl;
            return false;
        }
        std::cout << "[DEBUG] " << name << " pipeline state verified" << std::endl;
        return true;
    } catch (const ob::Error& e) {
        std::cout << "[ERROR] " << name << " pipeline verification failed: " << e.what() << std::endl;
        return false;
    }
}

// Add this function before OrbbecCapture class
bool extractFrameDataDirect(std::shared_ptr<ob::Frame> frame, cv::Mat& output, bool is_color) {
    try {
        std::cout << "[DEBUG] Trying direct frame data access..." << std::endl;
        
        // Try direct data access through base Frame interface
        void* rawData = nullptr;
        int frame_width = 848;   // Use configured resolution
        int frame_height = 480;
        
        try {
            // Access raw data directly through Frame interface
            rawData = (void*)frame->getData();
            std::cout << "[DEBUG] Raw data pointer obtained: " << rawData << std::endl;
            
        } catch (const std::exception& e) {
            std::cout << "[ERROR] getData() failed: " << e.what() << std::endl;
            return false;
        }
        
        if (!rawData) {
            std::cout << "[ERROR] Null data pointer" << std::endl;
            return false;
        }
        
        std::cout << "[DEBUG] Creating OpenCV matrix: " << frame_width << "x" << frame_height << std::endl;
        
        try {
            if (is_color) {
                // Create RGB matrix (3 channels, 8-bit) and convert to BGR
                cv::Mat temp(frame_height, frame_width, CV_8UC3, rawData);
                if (temp.empty()) {
                    std::cout << "[ERROR] Failed to create color matrix" << std::endl;
                    return false;
                }
                std::cout << "[DEBUG] Color matrix created, converting RGB to BGR..." << std::endl;
                cv::cvtColor(temp, output, cv::COLOR_RGB2BGR);
                std::cout << "[DEBUG] Color conversion successful" << std::endl;
            } else {
                // Create 16-bit depth matrix (1 channel)
                cv::Mat temp(frame_height, frame_width, CV_16UC1, rawData);
                if (temp.empty()) {
                    std::cout << "[ERROR] Failed to create depth matrix" << std::endl;
                    return false;
                }
                std::cout << "[DEBUG] Depth matrix created, cloning..." << std::endl;
                output = temp.clone();
                std::cout << "[DEBUG] Depth clone successful" << std::endl;
            }
        } catch (const cv::Exception& e) {
            std::cout << "[ERROR] OpenCV matrix creation failed: " << e.what() << std::endl;
            return false;
        }
        
        return true;
        
    } catch (const std::exception& e) {
        std::cout << "[ERROR] Direct extraction failed: " << e.what() << std::endl;
        return false;
    }
}

// Add this function before OrbbecCapture class to debug IMU buffer
void debugIMUBuffer(const std::vector<ORB_SLAM3::IMU::Point>& buffer) {
    static int debug_call_count = 0;
    if (++debug_call_count % 30 == 0) {
        std::cout << "[IMU_BUFFER_DEBUG] Buffer size: " << buffer.size() << std::endl;
        if (!buffer.empty()) {
            const auto& newest = buffer.back();
            const auto& oldest = buffer.front();
            std::cout << "[IMU_BUFFER_DEBUG] Time range: " << oldest.t 
                     << " to " << newest.t << " (span: " << (newest.t - oldest.t) << "s)" << std::endl;
        }
    }
}

class OrbbecCapture {
public:
    OrbbecCapture(bool use_hardware_align = true) 
        : pipeline_(nullptr), config_(nullptr), use_hw_align_(use_hardware_align), imu_running_(false) {
        std::cout << "[DEBUG] Using " << (use_hw_align_ ? "HARDWARE" : "SOFTWARE") << " alignment" << std::endl;
    }
    
    ~OrbbecCapture() {
        // Stop IMU thread first
        if (imu_running_) {
            imu_running_ = false;
            if (imu_thread_.joinable()) {
                imu_thread_.join();
            }
            std::cout << "[DEBUG] IMU thread stopped" << std::endl;
        }
        
        // Stop pipelines
        if (imu_pipeline_) {
            imu_pipeline_->stop();
            std::cout << "[DEBUG] IMU pipeline stopped" << std::endl;
        }
        if (pipeline_) {
            pipeline_->stop();
            std::cout << "[DEBUG] Video pipeline stopped" << std::endl;
        }
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
            
            // Check IMU availability
            auto sensorList = device_->getSensorList();
            bool has_accel = false, has_gyro = false;
            
            for (uint32_t i = 0; i < sensorList->getCount(); i++) {
                auto sensor = sensorList->getSensor(i);
                if (sensor->type() == OB_SENSOR_ACCEL) {
                    has_accel = true;
                    std::cout << "[DEBUG] Accelerometer found" << std::endl;
                }
                if (sensor->type() == OB_SENSOR_GYRO) {
                    has_gyro = true;
                    std::cout << "[DEBUG] Gyroscope found" << std::endl;
                }
            }
            
            pipeline_ = std::make_shared<ob::Pipeline>(device_);
            config_ = std::make_shared<ob::Config>();
            
            // Configure video streams - try multiple resolution options
            std::cout << "\n[DEBUG] Configuring video streams..." << std::endl;
            
            bool color_configured = false;
            bool depth_configured = false;
            
            // Try to configure COLOR stream
            try {
                std::cout << "[DEBUG] Trying COLOR: 848x480@60fps" << std::endl;
                config_->enableVideoStream(OB_STREAM_COLOR, 848, 480, 60, OB_FORMAT_RGB);
                color_configured = true;
                std::cout << "[DEBUG] COLOR configured: 848x480@60fps ✓" << std::endl;
            } catch (const ob::Error& e) {
                std::cout << "[WARNING] COLOR 848x480@60fps failed: " << e.what() << std::endl;
                
                try {
                    std::cout << "[DEBUG] Trying COLOR: 1280x720@30fps" << std::endl;
                    config_->enableVideoStream(OB_STREAM_COLOR, 1280, 720, 30, OB_FORMAT_RGB);
                    color_configured = true;
                    std::cout << "[DEBUG] COLOR configured: 1280x720@30fps ✓" << std::endl;
                } catch (const ob::Error& e2) {
                    std::cout << "[ERROR] All COLOR configurations failed!" << std::endl;
                }
            }
            
            // Try to configure DEPTH stream
            try {
                std::cout << "[DEBUG] Trying DEPTH: 848x480@60fps" << std::endl;
                config_->enableVideoStream(OB_STREAM_DEPTH, 848, 480, 60, OB_FORMAT_Y16);
                depth_configured = true;
                std::cout << "[DEBUG] DEPTH configured: 848x480@60fps ✓" << std::endl;
            } catch (const ob::Error& e) {
                std::cout << "[WARNING] DEPTH 848x480@60fps failed: " << e.what() << std::endl;
                
                try {
                    std::cout << "[DEBUG] Trying DEPTH: 848x480@30fps" << std::endl;
                    config_->enableVideoStream(OB_STREAM_DEPTH, 848, 480, 30, OB_FORMAT_Y16);
                    depth_configured = true;
                    std::cout << "[DEBUG] DEPTH configured: 848x480@30fps ✓" << std::endl;
                } catch (const ob::Error& e2) {
                    std::cout << "[ERROR] All DEPTH configurations failed!" << std::endl;
                }
            }
            
            if (!color_configured || !depth_configured) {
                std::cout << "[ERROR] Failed to configure required video streams!" << std::endl;
                return false;
            }
            
            // Configure separate IMU pipeline if sensors are available
            if (has_accel && has_gyro) {
                std::cout << "\n[DEBUG] Setting up separate IMU pipeline..." << std::endl;
                imu_pipeline_ = std::make_shared<ob::Pipeline>(device_);
                imu_config_ = std::make_shared<ob::Config>();
                
                try {
                    // Try 1000Hz first (matching your YAML), then fall back to lower rates
                    std::cout << "[DEBUG] Trying IMU at 1000Hz..." << std::endl;
                    imu_config_->enableAccelStream(OB_ACCEL_FS_4g, OB_SAMPLE_RATE_1_KHZ );  // Try highest available
                    imu_config_->enableGyroStream(OB_GYRO_FS_1000dps, OB_SAMPLE_RATE_1_KHZ );
                    std::cout << "[DEBUG] IMU configured at 1000Hz ✓" << std::endl;
                } catch (const ob::Error& e) {
                    std::cout << "[WARNING] IMU 1000Hz failed: " << e.what() << std::endl;
                    try {
                        std::cout << "[DEBUG] Trying IMU at 200Hz..." << std::endl;
                        imu_config_->enableAccelStream(OB_ACCEL_FS_4g, OB_SAMPLE_RATE_200_HZ);
                        imu_config_->enableGyroStream(OB_GYRO_FS_1000dps, OB_SAMPLE_RATE_200_HZ);
                        std::cout << "[DEBUG] IMU configured at 200Hz ✓" << std::endl;
                    } catch (const ob::Error& e2) {
                        std::cout << "[WARNING] IMU configuration failed: " << e2.what() << std::endl;
                        std::cout << "[WARNING] Falling back to RGB-D only mode" << std::endl;
                        imu_pipeline_.reset();
                        imu_config_.reset();
                    }
                }
            } else {
                std::cout << "[WARNING] IMU sensors not available - RGB-D only mode" << std::endl;
            }
            
            // Enable frame synchronization for video
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

            try {
                auto colorSensor = device_->getSensor(OB_SENSOR_COLOR);
                auto colorProfiles = colorSensor->getStreamProfileList();
                
                for (uint32_t i = 0; i < colorProfiles->getCount(); i++) {
                    auto profile = std::dynamic_pointer_cast<ob::VideoStreamProfile>(colorProfiles->getProfile(i));
                    if (profile->getWidth() == 848 && profile->getHeight() == 480) {
                        colorProfile = profile;
                        std::cout << "[DEBUG] Found matching color profile: " << profile->getWidth() 
                                << "x" << profile->getHeight() << "@" << profile->getFps() << "fps" << std::endl;
                        break;
                    }
                }
                
                if (!colorProfile) {
                    std::cout << "[WARNING] No exact color profile found, using default" << std::endl;
                    auto profiles = pipeline_->getStreamProfileList(OB_SENSOR_COLOR);
                    colorProfile = std::dynamic_pointer_cast<ob::VideoStreamProfile>(profiles->getProfile(0));
                }
            } catch (const ob::Error& e) {
                std::cout << "[WARNING] Error finding color profile: " << e.what() << std::endl;
                auto profiles = pipeline_->getStreamProfileList(OB_SENSOR_COLOR);
                colorProfile = std::dynamic_pointer_cast<ob::VideoStreamProfile>(profiles->getProfile(0));
            }

            try {
                // Get the ACTIVE stream profiles from the pipeline config instead of querying all available
                auto depthSensor = device_->getSensor(OB_SENSOR_DEPTH);
                auto depthProfiles = depthSensor->getStreamProfileList();

                for (uint32_t i = 0; i < depthProfiles->getCount(); i++) {
                    auto profile = std::dynamic_pointer_cast<ob::VideoStreamProfile>(depthProfiles->getProfile(i));
                    if (profile && profile->getWidth() == 848 && profile->getHeight() == 480) {
                        depthProfile = profile;
                        std::cout << "[DEBUG] Found matching depth profile: " << depthProfile->getWidth() 
                                << "x" << depthProfile->getHeight() << "@" << depthProfile->getFps() << "fps" << std::endl;
                        break;
                    }
                }
                
                if (!depthProfile) {
                    std::cout << "[WARNING] No active depth profile found, using default" << std::endl;
                    auto allProfiles = pipeline_->getStreamProfileList(OB_SENSOR_DEPTH);
                    depthProfile = std::dynamic_pointer_cast<ob::VideoStreamProfile>(allProfiles->getProfile(0));
                }
            } catch (const ob::Error& e) {
                std::cout << "[WARNING] Error getting active depth profile: " << e.what() << std::endl;
                auto profiles = pipeline_->getStreamProfileList(OB_SENSOR_DEPTH);
                depthProfile = std::dynamic_pointer_cast<ob::VideoStreamProfile>(profiles->getProfile(0));
            }

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

    bool start();
    bool getFrames(cv::Mat& color, cv::Mat& depth, double& timestamp);
    void toggleAlignmentMode();
    std::vector<ORB_SLAM3::IMU::Point> getIMUMeasurements(double t_start, double t_end);

public:
    // Made public for state verification
    std::shared_ptr<ob::Pipeline> pipeline_;
    std::atomic<bool> imu_running_;
    std::vector<ORB_SLAM3::IMU::Point> imu_buffer_;
    std::mutex imu_buffer_mutex_;
    
private:
    std::shared_ptr<ob::Context> context_;
    std::shared_ptr<ob::Device> device_;
    std::shared_ptr<ob::Config> config_;
    std::shared_ptr<ob::Align> align_filter_;
    
    // IMU-related members
    std::shared_ptr<ob::Pipeline> imu_pipeline_;
    std::shared_ptr<ob::Config> imu_config_;
    ImuSyncBuffer imu_sync_buffer_;
    std::thread imu_thread_;
    
    cv::Mat colorK_, depthK_;
    DepthToColorTransform sw_transformer_;
    bool use_hw_align_;
    
    void imuThreadFunc();
};

bool OrbbecCapture::start() {
    try {
        std::cout << "[DEBUG] Starting video pipeline..." << std::endl;
        pipeline_->start(config_);
        std::cout << "[DEBUG] Video pipeline started successfully ✓" << std::endl;
        
        // Start IMU pipeline if available
        if (imu_pipeline_) {
            try {
                imu_pipeline_->start(imu_config_);
                std::cout << "[DEBUG] IMU pipeline started ✓" << std::endl;
                
                imu_running_ = true;
                imu_thread_ = std::thread(&OrbbecCapture::imuThreadFunc, this);
                std::cout << "[DEBUG] IMU thread created, waiting for startup confirmation..." << std::endl;

                // Give IMU thread time to actually start
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                std::cout << "[DEBUG] IMU thread startup delay complete ✓" << std::endl;
            } catch (const ob::Error& e) {
                std::cout << "[WARNING] IMU start failed: " << e.what() << std::endl;
                std::cout << "[WARNING] Continuing in RGB-D only mode" << std::endl;
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        std::cout << "[DEBUG] All systems started and stabilized" << std::endl;
        return true;
    } catch (const ob::Error& e) {
        std::cerr << "[ERROR] Start error: " << e.what() << std::endl;
        return false;
    }
}

bool OrbbecCapture::getFrames(cv::Mat& color, cv::Mat& depth, double& timestamp) {
    static int frame_count = 0;
    static int hw_failures = 0;
    frame_count++;
    
    bool debug_this_frame = (frame_count % 30 == 0);
    
    try {
        // Add safety checks before accessing pipeline
        if (!pipeline_) {
            std::cout << "[ERROR] Pipeline is null!" << std::endl;
            return false;
        }
        
        std::cout << "[DEBUG] About to call waitForFrames..." << std::endl;
        auto frameSet = pipeline_->waitForFrames(100);
        std::cout << "[DEBUG] waitForFrames returned" << std::endl;
        
        if (!frameSet) {
            if (debug_this_frame) {
                std::cout << "[WARNING] No frameset received (timeout)" << std::endl;
            }
            return false;
        }
        
        std::cout << "[DEBUG] Frameset is valid" << std::endl;
        
        if (use_hw_align_) {
            if (!align_filter_) {
                std::cout << "[ERROR] Align filter is null!" << std::endl;
                return false;
            }
            
            std::cout << "[DEBUG] About to process alignment..." << std::endl;
            auto alignedFrame = align_filter_->process(frameSet);
            std::cout << "[DEBUG] Alignment process returned" << std::endl;
            
            if (!alignedFrame) {
                hw_failures++;
                if (hw_failures % 10 == 0 || debug_this_frame) {
                    std::cout << "[WARNING] Hardware alignment failed #" << hw_failures << ", retrying..." << std::endl;
                }
                return false;
            }
            
            std::cout << "[DEBUG] Alignment successful" << std::endl;
            
            // Cast to FrameSet first, then get frames
            auto alignedFrameSet = alignedFrame->as<ob::FrameSet>();
            if (!alignedFrameSet) {
                std::cout << "[WARNING] Failed to convert aligned frame to FrameSet" << std::endl;
                hw_failures++;
                if (hw_failures > 10) {
                    std::cout << "[WARNING] Too many hardware alignment failures, switching to software" << std::endl;
                    use_hw_align_ = false;
                    return getFrames(color, depth, timestamp); // Recursive call with software alignment
                }
                return false;
            }

            // Cast to FrameSet first, then get frames
            auto hwAlignedFrameSet = alignedFrame->as<ob::FrameSet>();
            if (!hwAlignedFrameSet) {
                std::cout << "[WARNING] Failed to convert aligned frame to FrameSet" << std::endl;
                hw_failures++;
                if (hw_failures > 10) {
                    std::cout << "[WARNING] Too many hardware alignment failures, switching to software" << std::endl;
                    use_hw_align_ = false;
                    return getFrames(color, depth, timestamp); // Recursive call with software alignment
                }
                return false;
            }
            
            std::cout << "[DEBUG] Getting color and depth frames from aligned set..." << std::endl;
            auto colorFrame = hwAlignedFrameSet->getFrame(OB_FRAME_COLOR);
            auto alignedDepthFrame = hwAlignedFrameSet->getFrame(OB_FRAME_DEPTH);
                    
            if (!colorFrame || !alignedDepthFrame) {
                if (debug_this_frame) {
                    std::cout << "[WARNING] Missing aligned frames" << std::endl;
                }
                return false;
            }

            std::cout << "[DEBUG] Frames retrieved, analyzing frame types..." << std::endl;

            // Check actual frame types
            std::cout << "[DEBUG] Color frame type: " << colorFrame->getType() << std::endl;
            std::cout << "[DEBUG] Depth frame type: " << alignedDepthFrame->getType() << std::endl;

            // Use Orbbec SDK's as<> method instead of std::dynamic_pointer_cast
            auto colorVideoFrame = colorFrame->as<ob::ColorFrame>();
            auto depthVideoFrame = alignedDepthFrame->as<ob::DepthFrame>();

            if (!colorVideoFrame) {
                std::cout << "[WARNING] Color frame as<ob::ColorFrame> failed, trying direct access..." << std::endl;
            }

            if (!depthVideoFrame) {
                std::cout << "[WARNING] Depth frame as<ob::DepthFrame> failed, trying direct access..." << std::endl;
            }

            // If Orbbec as<> casting fails, try direct data extraction
            if (!colorVideoFrame || !depthVideoFrame) {
                std::cout << "[WARNING] Orbbec as<> casting failed, trying direct extraction..." << std::endl;
                
                bool color_ok = false, depth_ok = false;
                
                if (!colorVideoFrame) {
                    color_ok = extractFrameDataDirect(colorFrame, color, true);
                    std::cout << "[DEBUG] Direct color extraction: " << (color_ok ? "SUCCESS" : "FAILED") << std::endl;
                } else {
                    color_ok = true;
                }
                
                if (!depthVideoFrame) {
                    depth_ok = extractFrameDataDirect(alignedDepthFrame, depth, false);
                    std::cout << "[DEBUG] Direct depth extraction: " << (depth_ok ? "SUCCESS" : "FAILED") << std::endl;
                } else {
                    depth_ok = true;
                }
                
                if (!color_ok || !depth_ok) {
                    std::cout << "[ERROR] All frame access methods failed, switching to software alignment" << std::endl;
                    hw_failures++;
                    if (hw_failures > 3) {
                        std::cout << "[WARNING] Hardware alignment consistently failing, switching permanently to software" << std::endl;
                        use_hw_align_ = false;
                    }
                    return false;
                }
                
                // Use system timestamp for direct method
                timestamp = std::chrono::duration<double>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
                
                std::cout << "[DEBUG] Direct extraction successful, proceeding..." << std::endl;
                
            } else {
                // Normal path when Orbbec casting works
                std::cout << "[DEBUG] Orbbec frame casting successful" << std::endl;
                
                // Continue with normal frame processing...
                // (the rest of your existing successful casting code goes here)
            }

            std::cout << "[DEBUG] Frame casting successful" << std::endl;
            std::cout << "[DEBUG] Color frame: " << colorVideoFrame->getWidth() << "x" << colorVideoFrame->getHeight() << std::endl;
            std::cout << "[DEBUG] Depth frame: " << depthVideoFrame->getWidth() << "x" << depthVideoFrame->getHeight() << std::endl;

            // Safe data pointer validation
            void* colorData = (void*)colorVideoFrame->getData();
            void* depthData = (void*)depthVideoFrame->getData();

            if (!colorData || !depthData) {
                std::cout << "[ERROR] Null frame data pointers" << std::endl;
                return false;
            }

            std::cout << "[DEBUG] Frame data pointers valid, creating OpenCV matrices..." << std::endl;

            // Create OpenCV matrices with error checking
            try {
                cv::Mat color_temp(colorVideoFrame->getHeight(), colorVideoFrame->getWidth(), 
                                CV_8UC3, colorData);
                
                if (color_temp.empty()) {
                    std::cout << "[ERROR] Failed to create color matrix" << std::endl;
                    return false;
                }
                
                std::cout << "[DEBUG] Color matrix created, converting color space..." << std::endl;
                cv::cvtColor(color_temp, color, cv::COLOR_RGB2BGR);
                std::cout << "[DEBUG] Color conversion successful" << std::endl;
                
            } catch (const cv::Exception& e) {
                std::cout << "[ERROR] OpenCV exception in color processing: " << e.what() << std::endl;
                return false;
            }
            try {
                uint64_t timestamp_us = colorVideoFrame->getTimeStampUs();
                if (timestamp_us == 0) {
                    // Use system time as fallback
                    timestamp = std::chrono::duration<double>(
                        std::chrono::steady_clock::now().time_since_epoch()).count();
                    std::cout << "[WARNING] Using system timestamp as fallback" << std::endl;
                } else {
                    timestamp = timestamp_us / 1000000.0;
                }
                std::cout << "[DEBUG] Timestamp: " << std::fixed << std::setprecision(6) << timestamp << std::endl;
            } catch (const std::exception& e) {
                std::cout << "[ERROR] Timestamp extraction failed: " << e.what() << std::endl;
                return false;
            }
            
            try {
                cv::Mat depth_temp(depthVideoFrame->getHeight(), depthVideoFrame->getWidth(), 
                                CV_16UC1, depthData);
                
                if (depth_temp.empty()) {
                    std::cout << "[ERROR] Failed to create depth matrix" << std::endl;
                    return false;
                }
                
                std::cout << "[DEBUG] Depth matrix created, cloning data..." << std::endl;
                depth = depth_temp.clone(); // Safer than direct assignment
                std::cout << "[DEBUG] Depth clone successful" << std::endl;
                
            } catch (const cv::Exception& e) {
                std::cout << "[ERROR] OpenCV exception in depth processing: " << e.what() << std::endl;
                return false;
            }
            
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
            
            auto colorVideoFrame = std::dynamic_pointer_cast<ob::ColorFrame>(colorFrame);
            cv::Mat color_temp(colorVideoFrame->getHeight(), colorVideoFrame->getWidth(), 
                              CV_8UC3, (void*)colorVideoFrame->getData());
            cv::cvtColor(color_temp, color, cv::COLOR_RGB2BGR);
            timestamp = colorVideoFrame->getTimeStampUs() / 1000000.0;
            
            auto depthVideoFrame = std::dynamic_pointer_cast<ob::DepthFrame>(depthFrame);
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
        cv::Size target_size(848, 480);
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

void OrbbecCapture::toggleAlignmentMode() {
    use_hw_align_ = !use_hw_align_;
    std::cout << "[DEBUG] Switched to " << (use_hw_align_ ? "HARDWARE" : "SOFTWARE") << " alignment" << std::endl;
}

std::vector<ORB_SLAM3::IMU::Point> OrbbecCapture::getIMUMeasurements(double t_start, double t_end) {
    std::lock_guard<std::mutex> lock(imu_buffer_mutex_);
    
    static int debug_count = 0;
    bool debug_this_call = (++debug_count % 30 == 0);
    
    if (debug_this_call) {
        std::cout << "[IMU_GET_DEBUG] Requesting IMU data for range: " 
                 << std::fixed << std::setprecision(6) 
                 << t_start << " to " << t_end << " (dt=" << (t_end - t_start) << "s)" << std::endl;
        std::cout << "[IMU_GET_DEBUG] Buffer contains " << imu_buffer_.size() << " total measurements" << std::endl;
    }
    
    std::vector<ORB_SLAM3::IMU::Point> measurements;
    int total_in_range = 0;
    const int MAX_IMU_MEASUREMENTS = 50; // Limit to prevent numerical instability

    for (const auto& imu_point : imu_buffer_) {
        if (imu_point.t >= t_start && imu_point.t <= t_end) {
            measurements.push_back(imu_point);
            total_in_range++;
            
            // Prevent buffer overflow that causes ORB-SLAM3 instability
            if (measurements.size() >= MAX_IMU_MEASUREMENTS) {
                if (debug_this_call) {
                    std::cout << "[IMU_BUFFER_LIMIT] Capped IMU measurements at " << MAX_IMU_MEASUREMENTS 
                            << " to prevent numerical instability" << std::endl;
                }
                break;
            }
        }
    }
    
    if (debug_this_call) {
        std::cout << "[IMU_GET_DEBUG] Found " << measurements.size() << " measurements in range" << std::endl;
        if (!imu_buffer_.empty()) {
            std::cout << "[IMU_GET_DEBUG] Buffer time range: " << imu_buffer_.front().t 
                     << " to " << imu_buffer_.back().t << std::endl;
        }
    }
    
    // Clean old data
    auto it = imu_buffer_.begin();
    while (it != imu_buffer_.end()) {
        if (t_end - it->t > 10.0) {
            it = imu_buffer_.erase(it);
        } else {
            ++it;
        }
    }
    
    return measurements;
}

void OrbbecCapture::imuThreadFunc() {
    std::cout << "[IMU_DEBUG] NaN detection code is ACTIVE" << std::endl;
    std::cout << "[DEBUG] IMU thread started" << std::endl;
    
    auto last_stats_time = std::chrono::steady_clock::now();
    int local_accel_count = 0, local_gyro_count = 0, local_synced_count = 0;
    int consecutive_errors = 0;
    
    while (imu_running_) {
        try {
            if (!imu_pipeline_) {
                std::cout << "[ERROR] IMU pipeline is null, stopping thread" << std::endl;
                break;
            }

            std::shared_ptr<ob::FrameSet> frameSet;
            try {
                static int debug_call_count = 0;
                debug_call_count++;
                
                if (debug_call_count % 100 == 0) {
                    std::cout << "[IMU_THREAD_DEBUG] Attempt " << debug_call_count << " - calling waitForFrameset..." << std::endl;
                }
                
                frameSet = imu_pipeline_->waitForFrameset(50);
                
                if (debug_call_count % 100 == 0) {
                    std::cout << "[IMU_THREAD_DEBUG] waitForFrameset returned, frameSet is " << (frameSet ? "valid" : "null") << std::endl;
                }
                
            } catch (const ob::Error& e) {
                std::cout << "[IMU_ERROR] waitForFrameset failed: " << e.what() << std::endl;
                consecutive_errors++;
                if (consecutive_errors > 50) {
                    std::cout << "[ERROR] Too many consecutive IMU errors, stopping thread" << std::endl;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            if (!frameSet) {
                consecutive_errors++;
                if (consecutive_errors == 1 || consecutive_errors % 50 == 0) {
                    std::cout << "[IMU_WARNING] No IMU frameset received (timeout #" << consecutive_errors << ")" << std::endl;
                }
                if (consecutive_errors > 200) {
                    std::cout << "[ERROR] Too many consecutive IMU timeouts, stopping thread" << std::endl;
                    break;
                }
                continue;
            }
            consecutive_errors = 0;
            
            static int frameset_count = 0;
            if (++frameset_count <= 5) {
                std::cout << "[IMU_SUCCESS] Received IMU frameset #" << frameset_count << std::endl;
            }
            
            bool processed_data = false;
        
            static int process_debug_count = 0;
            if (++process_debug_count <= 10 || process_debug_count % 100 == 0) {
                std::cout << "[IMU_PROCESS_DEBUG] Processing frameset #" << process_debug_count << std::endl;
            }

            // Process accelerometer
            auto accelFrame = frameSet->getFrame(OB_FRAME_ACCEL);
            if (process_debug_count <= 10) {
                std::cout << "[IMU_PROCESS_DEBUG] accelFrame is " << (accelFrame ? "valid" : "null") << std::endl;
            }
            if (accelFrame) {
                auto accelData = accelFrame->as<ob::AccelFrame>();
                if (process_debug_count <= 10) {
                    std::cout << "[IMU_PROCESS_DEBUG] accelData cast " << (accelData ? "SUCCESS" : "FAILED") << std::endl;
                }
                if (accelData) {
                    auto value = accelData->getValue();
                    double timestamp = accelData->getTimeStampUs() / 1000000.0;
                    
                    // CRITICAL: Check for NaN/infinity in raw sensor data
                    if (!std::isfinite(value.x) || !std::isfinite(value.y) || !std::isfinite(value.z) ||
                        !std::isfinite(timestamp)) {
                        
                        static int accel_corruption_count = 0;
                        if (++accel_corruption_count <= 3) {
                            std::cout << "[IMU_CORRUPTION] Frame #" << process_debug_count 
                                    << ": Raw accelerometer NaN detected - accel(" << value.x << "," << value.y << "," << value.z 
                                    << ") time=" << timestamp << std::endl;
                        }
                        continue; // Skip this corrupted frame
                    }
                    
                    float magnitude = sqrt(value.x*value.x + value.y*value.y + value.z*value.z);
                    if (magnitude > 0.1 && magnitude < 50.0) {
                        Eigen::Vector3f accel(value.x, value.y, value.z);
                        imu_sync_buffer_.addAccel(accel, timestamp);
                        local_accel_count++;
                        processed_data = true;
                    }
                }
            }
            
            // Process gyroscope
            auto gyroFrame = frameSet->getFrame(OB_FRAME_GYRO);
            if (process_debug_count <= 10) {
                std::cout << "[IMU_PROCESS_DEBUG] gyroFrame is " << (gyroFrame ? "valid" : "null") << std::endl;
            }
            if (gyroFrame) {
                auto gyroData = gyroFrame->as<ob::GyroFrame>();
                if (process_debug_count <= 10) {
                    std::cout << "[IMU_PROCESS_DEBUG] gyroData cast " << (gyroData ? "SUCCESS" : "FAILED") << std::endl;
                }
                if (gyroData) {
                    auto value = gyroData->getValue();
                    double timestamp = gyroData->getTimeStampUs() / 1000000.0;
                    
                    // CRITICAL: Check for NaN/infinity in raw sensor data
                    if (!std::isfinite(value.x) || !std::isfinite(value.y) || !std::isfinite(value.z) ||
                        !std::isfinite(timestamp)) {
                        
                        static int gyro_corruption_count = 0;
                        if (++gyro_corruption_count <= 3) {
                            std::cout << "[IMU_CORRUPTION] Frame #" << process_debug_count 
                                    << ": Raw gyroscope NaN detected - gyro(" << value.x << "," << value.y << "," << value.z 
                                    << ") time=" << timestamp << std::endl;
                        }
                        continue; // Skip this corrupted frame
                    }
                    
                    Eigen::Vector3f gyro(value.x * M_PI / 180.0f, 
                                        value.y * M_PI / 180.0f, 
                                        value.z * M_PI / 180.0f);
                    
                    // Additional validation after conversion
                    if (std::isfinite(gyro.x()) && std::isfinite(gyro.y()) && std::isfinite(gyro.z())) {
                        imu_sync_buffer_.addGyro(gyro, timestamp);
                        local_gyro_count++;
                        processed_data = true;
                    }
                }
            }

            // Get synchronized measurements
            auto complete_measurements = imu_sync_buffer_.getCompleteMeasurements();
            if (!complete_measurements.empty()) {
                std::lock_guard<std::mutex> lock(imu_buffer_mutex_);
                for (const auto& measurement : complete_measurements) {
                    // Safety check for NaN values before adding to buffer
                    if (std::isfinite(measurement.a.x()) && std::isfinite(measurement.a.y()) && std::isfinite(measurement.a.z()) &&
                        std::isfinite(measurement.w.x()) && std::isfinite(measurement.w.y()) && std::isfinite(measurement.w.z()) &&
                        std::isfinite(measurement.t)) {
                        imu_buffer_.push_back(measurement);
                    } else {
                        static int corrupt_count = 0;
                        if (++corrupt_count <= 5) {
                            std::cout << "[IMU_CORRUPTION] Frame #" << local_synced_count + corrupt_count 
                                    << ": Detected NaN in IMU measurement - accel(" << measurement.a.x() 
                                    << "," << measurement.a.y() << "," << measurement.a.z() 
                                    << ") gyro(" << measurement.w.x() << "," << measurement.w.y() 
                                    << "," << measurement.w.z() << ") time=" << measurement.t << std::endl;
                        }
                    }
                }
                local_synced_count += complete_measurements.size();
                
                // Debug output for first few measurements
                static int debug_count = 0;
                if (debug_count < 5 && !complete_measurements.empty()) {
                    const auto& first_measurement = complete_measurements[0];
                    std::cout << "[IMU_DEBUG] Added measurement " << debug_count 
                            << ": t=" << std::fixed << std::setprecision(6) << first_measurement.t 
                            << ", accel=(" << first_measurement.a.x() << "," << first_measurement.a.y() << "," << first_measurement.a.z() << ")"
                            << ", gyro=(" << first_measurement.w.x() << "," << first_measurement.w.y() << "," << first_measurement.w.z() << ")" << std::endl;
                    debug_count++;
                }
            }
            
            // Print statistics every 5 seconds
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(now - last_stats_time).count() >= 5) {
                std::cout << "[IMU_STATS] Last 5s - Accel: " << local_accel_count 
                          << ", Gyro: " << local_gyro_count << ", Synced: " << local_synced_count << std::endl;
                local_accel_count = local_gyro_count = local_synced_count = 0;
                last_stats_time = now;
            }
            
            if (!processed_data) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
            
        } catch (const ob::Error& e) {
            std::cerr << "[IMU_ERROR] " << e.what() << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    
    std::cout << "[DEBUG] IMU thread stopped" << std::endl;
}

// Add this before main() function
void setupCrashHandler() {
    struct sigaction sa;
    sa.sa_handler = [](int sig) {
        std::cout << "\n[CRASH] Signal " << sig << " caught!" << std::endl;
        std::cout << "[CRASH] Attempting graceful shutdown..." << std::endl;
        g_resource_manager.requestExit();
        
        // Give threads time to cleanup
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        
        // Force exit if still alive
        std::cout << "[CRASH] Force exit" << std::endl;
        _exit(1);
    };
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART;
    
    sigaction(SIGSEGV, &sa, nullptr);
    sigaction(SIGABRT, &sa, nullptr);
    sigaction(SIGFPE, &sa, nullptr);
}

// Add this before main() function
void stopOrbbecViewer() {
    try {
        // The ORB-SLAM3 viewer might be causing issues, try to stop it
        std::cout << "[DEBUG] Attempting to stop any running viewer..." << std::endl;
        // Note: This is a workaround - the "Starting the Viewer" message suggests 
        // ORB-SLAM3 is trying to start a viewer in headless mode
    } catch (...) {
        std::cout << "[DEBUG] Viewer stop completed" << std::endl;
    }
}

// Manual acceleration detection to bypass ORB-SLAM3's restrictive checks
bool detectSufficientMotion(const std::vector<ORB_SLAM3::IMU::Point>& imu_measurements) {
    if (imu_measurements.size() < 10) return false;
    
    // Calculate acceleration variance over the measurement window
    Eigen::Vector3f accel_mean(0, 0, 0);
    for (const auto& imu : imu_measurements) {
        accel_mean += imu.a;
    }
    accel_mean /= imu_measurements.size();
    
    float accel_variance = 0.0f;
    for (const auto& imu : imu_measurements) {
        Eigen::Vector3f diff = imu.a - accel_mean;
        accel_variance += diff.squaredNorm();
    }
    accel_variance /= imu_measurements.size();
    
    // Also check gyro variance
    Eigen::Vector3f gyro_mean(0, 0, 0);
    for (const auto& imu : imu_measurements) {
        gyro_mean += imu.w;
    }
    gyro_mean /= imu_measurements.size();
    
    float gyro_variance = 0.0f;
    for (const auto& imu : imu_measurements) {
        Eigen::Vector3f diff = imu.w - gyro_mean;
        gyro_variance += diff.squaredNorm();
    }
    gyro_variance /= imu_measurements.size();
    
    static int motion_check_count = 0;
    if (++motion_check_count % 10 == 0) {
        std::cout << "[MOTION_DETECT] Accel variance: " << accel_variance 
                 << ", Gyro variance: " << gyro_variance << std::endl;
    }
    
    // Lower thresholds than ORB-SLAM3's conservative defaults
    bool sufficient_motion = (accel_variance > 0.5f) || (gyro_variance > 0.01f);
    
    if (sufficient_motion && motion_check_count % 10 == 0) {
        std::cout << "[MOTION_DETECT] Sufficient motion detected for IMU initialization!" << std::endl;
    }
    
    return sufficient_motion;
}

// Add this function before main() to debug actual IMU values
void debugIMUValues(const std::vector<ORB_SLAM3::IMU::Point>& imu_measurements) {
    if (imu_measurements.empty()) return;
    
    static int debug_count = 0;
    if (++debug_count % 30 != 0) return; // Print every 30 calls
    
    // Calculate stats
    Eigen::Vector3f accel_min(1000, 1000, 1000), accel_max(-1000, -1000, -1000);
    Eigen::Vector3f gyro_min(1000, 1000, 1000), gyro_max(-1000, -1000, -1000);
    Eigen::Vector3f accel_sum(0, 0, 0), gyro_sum(0, 0, 0);
    
    for (const auto& imu : imu_measurements) {
        // Track min/max for range
        for (int i = 0; i < 3; i++) {
            accel_min[i] = std::min(accel_min[i], imu.a[i]);
            accel_max[i] = std::max(accel_max[i], imu.a[i]);
            gyro_min[i] = std::min(gyro_min[i], imu.w[i]);
            gyro_max[i] = std::max(gyro_max[i], imu.w[i]);
        }
        accel_sum += imu.a;
        gyro_sum += imu.w;
    }
    
    Eigen::Vector3f accel_mean = accel_sum / imu_measurements.size();
    Eigen::Vector3f gyro_mean = gyro_sum / imu_measurements.size();
    
    std::cout << "\n[IMU_VALUES_DEBUG] ===== IMU Data Analysis =====" << std::endl;
    std::cout << "[IMU_VALUES_DEBUG] Samples: " << imu_measurements.size() << std::endl;
    std::cout << "[IMU_VALUES_DEBUG] Accel Mean: (" << accel_mean.x() << ", " << accel_mean.y() << ", " << accel_mean.z() << ")" << std::endl;
    std::cout << "[IMU_VALUES_DEBUG] Accel Range: X[" << accel_min.x() << " to " << accel_max.x() << "], Y[" << accel_min.y() << " to " << accel_max.y() << "], Z[" << accel_min.z() << " to " << accel_max.z() << "]" << std::endl;
    std::cout << "[IMU_VALUES_DEBUG] Gyro Mean: (" << gyro_mean.x() << ", " << gyro_mean.y() << ", " << gyro_mean.z() << ")" << std::endl;
    std::cout << "[IMU_VALUES_DEBUG] Gyro Range: X[" << gyro_min.x() << " to " << gyro_max.x() << "], Y[" << gyro_min.y() << " to " << gyro_max.y() << "], Z[" << gyro_min.z() << " to " << gyro_max.z() << "]" << std::endl;
    
    // Calculate actual variance like the motion detection does
    float accel_variance = 0.0f, gyro_variance = 0.0f;
    for (const auto& imu : imu_measurements) {
        Eigen::Vector3f accel_diff = imu.a - accel_mean;
        Eigen::Vector3f gyro_diff = imu.w - gyro_mean;
        accel_variance += accel_diff.squaredNorm();
        gyro_variance += gyro_diff.squaredNorm();
    }
    accel_variance /= imu_measurements.size();
    gyro_variance /= imu_measurements.size();
    
    std::cout << "[IMU_VALUES_DEBUG] Accel Variance: " << accel_variance << " (threshold: 0.5)" << std::endl;
    std::cout << "[IMU_VALUES_DEBUG] Gyro Variance: " << gyro_variance << " (threshold: 0.01)" << std::endl;
    std::cout << "[IMU_VALUES_DEBUG] Motion Sufficient: " << ((accel_variance > 0.5f) || (gyro_variance > 0.01f) ? "YES" : "NO") << std::endl;
    std::cout << "[IMU_VALUES_DEBUG] ================================\n" << std::endl;
}

int main(int argc, char **argv) {
    std::cout << "\n[DEBUG] =============== ORB-SLAM3 RGB-D + IMU ===============" << std::endl;
    
    // Force headless mode for maximum performance
    std::cout << "[INFO] Running in headless mode for maximum performance" << std::endl;
    
    // Register signal handlers
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    setupCrashHandler();
    cv::setBreakOnError(false);
    
    if (argc < 3 || argc > 4) {
        cerr << "Usage: ./rgbd_imu_orbbec_gemini335 vocabulary settings [hw|sw]" << endl;
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

    // // Verify YAML file before proceeding
    // if (!verifyYAMLFile(argv[2])) {
    //     std::cerr << "[ERROR] YAML file verification failed!" << std::endl;
    //     return -1;
    // }

    // Create reconstruction folder
    std::string reconstruction_folder = "3D_Reconstruction_Data";
    std::string folder = "3D_Reconstruction_Data/";
    std::set<std::string> good_map_points;
    
    if (mkdir(reconstruction_folder.c_str(), 0777) == 0) {
        std::cout << "[DEBUG] Created folder: " << reconstruction_folder << std::endl;
    } else {
        std::cout << "[DEBUG] Using existing folder: " << reconstruction_folder << std::endl;
    }

    // Initialize camera
    OrbbecCapture capture(use_hardware);
    if (!capture.initialize()) {
        cerr << "[ERROR] Failed to initialize camera!" << endl;
        return -1;
    }

    // Initialize SLAM in IMU-RGBD mode
    // Verify YAML file before SLAM initialization
    if (!verifyYAMLFile(argv[2])) {
        std::cerr << "[ERROR] YAML file verification failed!" << std::endl;
        return -1;
    }

    // Initialize SLAM in IMU-RGBD mode
    std::cout << "\n[DEBUG] Initializing ORB-SLAM3 in IMU-RGBD mode..." << std::endl;
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_RGBD, true);
    float imageScale = SLAM.GetImageScale();
    std::cout << "[DEBUG] ORB-SLAM3 image scale factor: " << imageScale << std::endl;

    // Verify YAML file was loaded correctly by SLAM
    cv::FileStorage fs(argv[2], cv::FileStorage::READ);
    if (fs.isOpened()) {
        float yaml_fx, yaml_fy, yaml_cx, yaml_cy;
        fs["Camera.fx"] >> yaml_fx;
        fs["Camera.fy"] >> yaml_fy;
        fs["Camera.cx"] >> yaml_cx;
        fs["Camera.cy"] >> yaml_cy;
        fs.release();

        std::cout << "[DEBUG] YAML camera parameters:" << std::endl;
        std::cout << "[DEBUG] fx=" << yaml_fx << ", fy=" << yaml_fy 
                 << ", cx=" << yaml_cx << ", cy=" << yaml_cy << std::endl;
    }

    if (!capture.start()) {
        cerr << "[ERROR] Failed to start camera!" << endl;
        return -1;
    }

    cout << "\n[DEBUG] ============= SLAM STARTED =============" << endl;
    cout << "[INFO] Controls: 'q'=quit, 't'=toggle alignment, 'm'=save map" << endl;

    // Add SLAM initialization monitoring
    std::cout << "\n[SLAM_INIT] Waiting for SLAM initialization..." << std::endl;
    std::cout << "[SLAM_INIT] IMU-RGBD SLAM requires motion and good features to initialize" << std::endl;
    std::cout << "[SLAM_INIT] Move the camera around to help initialization" << std::endl;

    std::cout << "[DEBUG] Validating initial system state..." << std::endl;
    std::cout << "[DEBUG] Resource manager ready: " << (!g_resource_manager.shouldExit() ? "YES" : "NO") << std::endl;

    // Verify pipeline states before attempting frame capture
    if (!verifyPipelineState(capture.pipeline_, "Video")) {
        std::cout << "[FATAL] Video pipeline verification failed" << std::endl;
        return -1;
    }

    // Add a longer stabilization delay
    std::cout << "[DEBUG] Allowing extra time for camera stabilization..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    std::cout << "[DEBUG] Attempting first frame capture..." << std::endl;

    // Test first frame capture with detailed debugging
    cv::Mat test_color, test_depth;
    double test_timestamp;
    bool first_frame_ok = false;

    for (int attempt = 0; attempt < 5; attempt++) {
        std::cout << "[DEBUG] Frame capture attempt " << (attempt + 1) << "/5..." << std::endl;
        try {
            if (capture.getFrames(test_color, test_depth, test_timestamp)) {
                std::cout << "[DEBUG] First frame captured successfully!" << std::endl;
                std::cout << "[DEBUG] Color size: " << test_color.size() << ", Depth size: " << test_depth.size() << std::endl;
                first_frame_ok = true;
                break;
            } else {
                std::cout << "[WARNING] Frame capture returned false on attempt " << (attempt + 1) << std::endl;
            }
        } catch (const std::exception& e) {
            std::cout << "[ERROR] Exception on first frame attempt " << (attempt + 1) << ": " << e.what() << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    if (!first_frame_ok) {
        std::cout << "[FATAL] Cannot capture initial frames - exiting" << std::endl;
        return -1;
    }

    std::cout << "[DEBUG] Starting main processing loop..." << std::endl;
    
    // Setup terminal for real-time key input
    setupTerminal();

    // Initialize OpenCV matrices with safe defaults
    cv::Mat im = cv::Mat::zeros(480, 848, CV_8UC3);
    cv::Mat depthmap = cv::Mat::zeros(480, 848, CV_16UC1);
    double timestamp = 0.0;
    double prev_timestamp = 0.0;

    std::cout << "[DEBUG] Initialized OpenCV matrices safely" << std::endl;
    int frame_count = 0;
    int tracking_ok_count = 0;
    std::vector<ORB_SLAM3::IMU::Point> imu_measurements;
    
    auto start_time = std::chrono::steady_clock::now();
    
    while (!g_resource_manager.shouldExit()) {
        // Add debug heartbeat every 1000 iterations
        static int loop_count = 0;
        if (++loop_count % 1000 == 0) {
            std::cout << "[DEBUG] Main loop heartbeat: iteration " << loop_count << std::endl;
        }
        
        // Get frames with exception handling
        bool frame_success = false;
        try {
            frame_success = capture.getFrames(im, depthmap, timestamp);
        } catch (const std::exception& e) {
            std::cout << "[ERROR] Exception in getFrames: " << e.what() << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        } catch (...) {
            std::cout << "[ERROR] Unknown exception in getFrames" << std::endl;
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

            // Get IMU measurements for this frame if available
            imu_measurements.clear(); // Clear previous measurements
            if (frame_count > 0 && prev_timestamp > 0.0) {
                // Try a wider time window to account for timestamp synchronization issues
                double time_margin = 0.05; // 50ms margin
                double imu_start = prev_timestamp - time_margin;
                double imu_end = timestamp + time_margin;
                
                imu_measurements = capture.getIMUMeasurements(imu_start, imu_end);

                // Validate all IMU measurements before passing to SLAM
                std::vector<ORB_SLAM3::IMU::Point> valid_measurements;
                for (const auto& imu_point : imu_measurements) {
                    // Check for NaN or infinite values
                    if (std::isfinite(imu_point.a.x()) && std::isfinite(imu_point.a.y()) && std::isfinite(imu_point.a.z()) &&
                        std::isfinite(imu_point.w.x()) && std::isfinite(imu_point.w.y()) && std::isfinite(imu_point.w.z()) &&
                        std::isfinite(imu_point.t)) {
                        
                        // Check for reasonable ranges
                        float accel_mag = imu_point.a.norm();
                        float gyro_mag = imu_point.w.norm();
                        
                        if (accel_mag < 50.0f && gyro_mag < 10.0f) {  // Reasonable physical limits
                            valid_measurements.push_back(imu_point);
                        }
                    }
                }

                imu_measurements = valid_measurements;

                if (frame_count <= 50 || frame_count % 30 == 0) {
                    std::cout << "[IMU_DEBUG] Frame " << frame_count << " - Requested IMU data for range: " 
                            << std::fixed << std::setprecision(6) 
                            << prev_timestamp << " -> " << timestamp 
                            << " (dt=" << (timestamp - prev_timestamp) << "s, margin=±" << time_margin << "s)" << std::endl;
                    std::cout << "[IMU_DEBUG] Retrieved " << imu_measurements.size() << " valid IMU measurements" << std::endl;
                    
                    // Show buffer status
                    std::lock_guard<std::mutex> lock(capture.imu_buffer_mutex_);
                    std::cout << "[IMU_DEBUG] Total buffer size: " << capture.imu_buffer_.size() << std::endl;
                    if (!capture.imu_buffer_.empty()) {
                        std::cout << "[IMU_DEBUG] Buffer time range: " << capture.imu_buffer_.front().t 
                                << " to " << capture.imu_buffer_.back().t << std::endl;
                    }
                }
            }

            // Continue IMU-RGBD mode indefinitely - no emergency fallback
            static bool imu_mode_active = true;
            if (frame_count > 200 && tracking_ok_count == 0 && imu_mode_active) {
                std::cout << "[INFO] IMU-RGBD initialization in progress - requires motion and features" << std::endl;
                // Continue with IMU-RGBD mode regardless of initialization time
            }

            // Adaptive IMU-RGBD with automatic fallback
            static int no_tracking_count = 0;
            static bool force_rgbd_only = false;

            // Check if IMU-RGBD initialization is failing
            if (frame_count > 100 && tracking_ok_count == 0) {
                force_rgbd_only = true;
                std::cout << "[ADAPTIVE] Frame " << frame_count << ": IMU-RGBD initialization failed after 100 frames, switching to RGB-D only permanently" << std::endl;
            }

            if (force_rgbd_only) {
                // Use reliable RGB-D only mode
                SLAM.TrackRGBD(im, depthmap, timestamp);
                std::cout << "[TRACK] Frame " << frame_count << ": RGB-D only mode (IMU disabled)" << std::endl;
            } else if (!imu_measurements.empty()) {
                // Try IMU-RGBD mode with validation
                std::vector<ORB_SLAM3::IMU::Point> safe_measurements;
                int corrupted_count = 0;
                
                for (const auto& imu_point : imu_measurements) {
                    if (std::isfinite(imu_point.a.x()) && std::isfinite(imu_point.a.y()) && std::isfinite(imu_point.a.z()) &&
                        std::isfinite(imu_point.w.x()) && std::isfinite(imu_point.w.y()) && std::isfinite(imu_point.w.z()) &&
                        std::isfinite(imu_point.t) && imu_point.t > 0.0) {
                        
                        float accel_mag = imu_point.a.norm();
                        float gyro_mag = imu_point.w.norm();
                        
                        if (accel_mag < 50.0f && gyro_mag < 20.0f) {
                            safe_measurements.push_back(imu_point);
                        } else {
                            corrupted_count++;
                        }
                    } else {
                        corrupted_count++;
                    }
                }
                
                if (safe_measurements.size() >= 10) {
                    // Debug actual IMU values
                    debugIMUValues(safe_measurements);

                    // Check if we have sufficient motion manually
                    bool has_motion = detectSufficientMotion(safe_measurements);
                    
                    if (has_motion) {
                        std::cout << "[IMU_TRACK] Frame " << frame_count << ": Using " << safe_measurements.size() 
                                << " validated IMU measurements (motion detected)" << std::endl;
                        SLAM.TrackRGBD(im, depthmap, timestamp, safe_measurements);
                    } else {
                        std::cout << "[MOTION_TRACK] Frame " << frame_count << ": Motion insufficient for IMU, using RGB-D only" << std::endl;
                        SLAM.TrackRGBD(im, depthmap, timestamp);
                    }
                } else {
                    std::cout << "[TRACK] Frame " << frame_count << ": Too few valid IMU measurements (" << safe_measurements.size() << ") - using RGB-D only" << std::endl;
                    SLAM.TrackRGBD(im, depthmap, timestamp);
                    no_tracking_count++;
                    if (no_tracking_count > 50) {
                        force_rgbd_only = true;
                        std::cout << "[ADAPTIVE] Too many frames without sufficient IMU data, switching to RGB-D only" << std::endl;
                    }
                }
            } else {
                std::cout << "[TRACK] Frame " << frame_count << ": No IMU data - using RGB-D only" << std::endl;
                SLAM.TrackRGBD(im, depthmap, timestamp);
                no_tracking_count++;
            }
            
            auto track_end = std::chrono::steady_clock::now();
            ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(track_end - track_start).count();
            frame_count++;

            // // Get tracking state
            // state = SLAM.GetTrackingState();
            // if (state == 3) tracking_ok_count++; // OK state
            
            // Update previous timestamp
            prev_timestamp = timestamp;
            
            // Get tracking state with detailed logging
            state = SLAM.GetTrackingState();
            if (frame_count <= 20 || frame_count % 10 == 0) {
                string state_str;
                switch (state) {
                    case 0: state_str = "SYSTEM_NOT_READY"; break;
                    case 1: state_str = "NO_IMAGES_YET"; break;
                    case 2: state_str = "NOT_INITIALIZED"; break;
                    case 3: state_str = "OK"; break;
                    case 4: state_str = "RECENTLY_LOST"; break;
                    case 5: state_str = "LOST"; break;
                    default: state_str = "UNKNOWN"; break;
                }
                std::cout << "[TRACKING_STATE] Frame " << frame_count << ": " << state_str << std::endl;
            }
            if (state == 3) tracking_ok_count++; // OK state
            
        } catch (const std::exception& e) {
            std::cout << "[ERROR] Exception in SLAM tracking: " << e.what() << std::endl;
            continue;
        }

        // Detailed IMU thread monitoring every 10 frames
        if (frame_count % 10 == 0) {
            std::cout << "[IMU_HEALTH] Frame " << frame_count << " - Thread running: " << (capture.imu_running_ ? "YES" : "NO") << std::endl;
            if (capture.imu_running_) {
                std::lock_guard<std::mutex> lock(capture.imu_buffer_mutex_);
                std::cout << "[IMU_HEALTH] Buffer contains " << capture.imu_buffer_.size() << " measurements" << std::endl;
            }
        }

        // Status every 30 frames
        if (frame_count % 30 == 0) {
            auto current_time = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - start_time).count();
            double fps = frame_count / elapsed;
            
            cout << "[STATUS] Frame " << frame_count 
                 << " | FPS: " << std::fixed << std::setprecision(1) << fps
                 << " | Track: " << std::setprecision(3) << ttrack << "s";

            // Add IMU count if we have measurements
            if (!imu_measurements.empty()) {
                cout << " | IMU: " << imu_measurements.size() << " meas";
            } else {
                cout << " | IMU: none";
            }
            
            string state_str;
            switch (state) {
                case 0: state_str = "SYSTEM_NOT_READY"; break;
                case 1: state_str = "NO_IMAGES_YET"; break;
                case 2: state_str = "NOT_INITIALIZED"; break;
                case 3: state_str = "OK"; break;
                case 4: state_str = "RECENTLY_LOST"; break;
                case 5: state_str = "LOST"; break;
                default: state_str = "UNKNOWN"; break;
            }
            
            cout << " | STATUS: " << state_str;
            cout << " | Success: " << (100.0 * tracking_ok_count / frame_count) << "%" << endl;
        }
        
        // Save map points periodically when tracking is OK
        if (frame_count % 100 == 0 && state == 3) {
            std::string temp_file = folder + "map_points_frame_" + std::to_string(frame_count) + ".txt";
            SLAM.SavePointCloud(temp_file);
            
            std::ifstream file(temp_file);
            if (file.is_open()) {
                std::string line;
                int actual_count = 0;
                while (std::getline(file, line)) {
                    if (!line.empty() && line[0] != '#') {
                        good_map_points.insert(line);
                        actual_count++;
                    }
                }
                file.close();
                
                if (frame_count % 500 == 0) {
                    std::cout << "[DEBUG] Frame " << frame_count << ": added " 
                            << actual_count << " points, total unique: " << good_map_points.size() << std::endl;
                }
            }
        }

        // Monitor IMU thread health
        static int health_check = 0;
        if (++health_check % 100 == 0) {
            std::cout << "[IMU_HEALTH] Thread running: " << (capture.imu_running_ ? "YES" : "NO") << std::endl;
            if (capture.imu_running_) {
                debugIMUBuffer(capture.imu_buffer_);
            }
        }

        // Check for keyboard input (non-blocking)
        static int input_check = 0;
        if (++input_check % 10 == 0) {
            fd_set readfds;
            FD_ZERO(&readfds);
            FD_SET(STDIN_FILENO, &readfds);
            
            struct timeval timeout;
            timeout.tv_sec = 0;
            timeout.tv_usec = 0;
            
            if (select(STDIN_FILENO + 1, &readfds, NULL, NULL, &timeout) > 0) {
                char input;
                if (read(STDIN_FILENO, &input, 1) > 0) {
                    std::cout << "[KEY] Terminal input: '" << input << "'" << std::endl;
                    if (input == 'q' || input == 'Q' || input == 27) {
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
        }
        
        // Check for exit file (backup method)
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

    // Safe SLAM shutdown and data export
    try {
        std::cout << "[DEBUG] Shutting down SLAM system..." << std::endl;
        SLAM.Shutdown();
        std::cout << "[DEBUG] SLAM shutdown complete" << std::endl;
        
        std::cout << "[DEBUG] Saving trajectories..." << std::endl;
        string suffix = "_imu_rgbd_run";
        SLAM.SaveKeyFrameTrajectoryTUM(folder + "KeyFrameTrajectory" + suffix + ".txt");
        SLAM.SaveTrajectoryTUM(folder + "CameraTrajectory" + suffix + ".txt");
        std::cout << "[DEBUG] Trajectories saved successfully" << std::endl;
        
        // Save ORB-SLAM3 map points
        std::cout << "[DEBUG] Saving map points..." << std::endl;
        SLAM.SavePointCloud(folder + "map_points.txt");

        // Save combined map from all intermediate saves
        std::string combined_file = folder + "map_points_COMBINED.txt";
        std::ofstream combined(combined_file);
        combined << "# Combined map points from all intermediate saves (IMU-RGBD mode)\n";
        combined << "# X,Y,Z coordinates in meters\n";

        for (const auto& point_line : good_map_points) {
            combined << point_line << "\n";
        }
        combined.close();

        std::cout << "[DEBUG] Saved COMBINED map with " << good_map_points.size() 
                << " unique points from all frames" << std::endl;

        // Check file size after saving
        std::ifstream check_file(folder + "map_points.txt");
        if (check_file.is_open()) {
            std::string line;
            int line_count = 0;
            while (std::getline(check_file, line)) {
                if (!line.empty() && line[0] != '#') line_count++;
            }
            check_file.close();
            std::cout << "[DEBUG] Saved " << line_count << " map points to final file" << std::endl;
        } else {
            std::cout << "[WARNING] Could not verify saved map points" << std::endl;
        }

        std::cout << "[DEBUG] Map points saved successfully" << std::endl;
        
        // Generate Python mesh generation script
        std::cout << "[DEBUG] Generating mesh generator script..." << std::endl;
        std::ofstream script(folder + "generate_mesh.py");
        script << "#!/usr/bin/env python3\n";
        script << "import numpy as np\n";
        script << "import open3d as o3d\n";
        script << "import os\n\n";
        
        script << "def load_points(filename):\n";
        script << "    if not os.path.exists(filename):\n";
        script << "        print(f'File not found: {filename}')\n";
        script << "        return None\n";
        script << "    points = np.genfromtxt(filename, delimiter=',', skip_header=1)[:, :3]\n";
        script << "    print(f'Loaded {len(points)} points from {filename}')\n";
        script << "    return points\n\n";
        
        script << "def create_mesh(points):\n";
        script << "    pcd = o3d.geometry.PointCloud()\n";
        script << "    pcd.points = o3d.utility.Vector3dVector(points)\n";
        script << "    pcd.estimate_normals()\n";
        script << "    \n";
        script << "    # Try Poisson reconstruction\n";
        script << "    mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=9)\n";
        script << "    mesh.remove_degenerate_triangles()\n";
        script << "    mesh.remove_duplicated_triangles()\n";
        script << "    mesh.remove_duplicated_vertices()\n";
        script << "    mesh.remove_non_manifold_edges()\n";
        script << "    \n";
        script << "    return pcd, mesh\n\n";
        
        script << "def main():\n";
        script << "    # Try combined file first, then regular file\n";
        script << "    for filename in ['map_points_COMBINED.txt', 'map_points.txt']:\n";
        script << "        points = load_points(filename)\n";
        script << "        if points is not None and len(points) > 100:\n";
        script << "            print(f'Using {filename} with {len(points)} points')\n";
        script << "            pcd, mesh = create_mesh(points)\n";
        script << "            \n";
        script << "            # Save results\n";
        script << "            o3d.io.write_point_cloud('reconstruction_points.ply', pcd)\n";
        script << "            o3d.io.write_triangle_mesh('reconstruction_mesh.stl', mesh)\n";
        script << "            o3d.io.write_triangle_mesh('reconstruction_mesh.ply', mesh)\n";
        script << "            \n";
        script << "            print(f'Saved point cloud and mesh:')\n";
        script << "            print(f'  Points: reconstruction_points.ply')\n";
        script << "            print(f'  Mesh: reconstruction_mesh.stl')\n";
        script << "            print(f'  Mesh: reconstruction_mesh.ply')\n";
        script << "            print(f'  Vertices: {len(mesh.vertices)}')\n";
        script << "            print(f'  Triangles: {len(mesh.triangles)}')\n";
        script << "            break\n";
        script << "    else:\n";
        script << "        print('No suitable point cloud file found or insufficient points')\n\n";
        
        script << "if __name__ == '__main__':\n";
        script << "    main()\n";
        
        script.close();
        chmod((folder + "generate_mesh.py").c_str(), 0755);
        
        std::cout << "[DEBUG] Mesh generator script created" << std::endl;
        
    } catch (const std::exception& e) {
        std::cout << "[ERROR] Shutdown/save error: " << e.what() << std::endl;
    }

    std::cout << "[DEBUG] Program completed successfully!" << std::endl;
    std::cout << "\n[3D RECONSTRUCTION] You now have:" << std::endl;
    std::cout << "✅ ORB-SLAM3 map points: map_points.txt" << std::endl;
    std::cout << "✅ Combined map points: map_points_COMBINED.txt" << std::endl;
    std::cout << "✅ IMU-RGBD trajectory: KeyFrameTrajectory_imu_rgbd_run.txt" << std::endl;
    std::cout << "✅ Camera trajectory: CameraTrajectory_imu_rgbd_run.txt" << std::endl;
    std::cout << "✅ Mesh generator: generate_mesh.py" << std::endl;
    std::cout << "🚀 Run: cd " << reconstruction_folder << " && python3 generate_mesh.py" << std::endl;
    std::cout << "\n[IMU-RGBD SLAM] Complete RGB-D + IMU SLAM with 3D reconstruction!" << std::endl;

    return 0;
}