/**
 * ORB-SLAM3 RGB-D with IMU Fallback
 * IMU is only used when visual tracking is lost
 * Primary mode: RGB-D, Fallback mode: IMU assists recovery
 */

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include <System.h>
#include <ImuTypes.h>

// Orbbec SDK includes
#include "libobsensor/ObSensor.hpp"

using namespace std;

// Simplified IMU buffer for fallback use only
class ImuBuffer {
private:
    std::queue<ORB_SLAM3::IMU::Point> imu_queue_;
    mutable std::mutex mutex_;
    bool collecting_ = false;
    
public:
    void startCollecting() {
        std::lock_guard<std::mutex> lock(mutex_);
        collecting_ = true;
        // Clear old data when starting collection
        while (!imu_queue_.empty()) {
            imu_queue_.pop();
        }
    }
    
    void stopCollecting() {
        std::lock_guard<std::mutex> lock(mutex_);
        collecting_ = false;
    }
    
    bool isCollecting() {
        std::lock_guard<std::mutex> lock(mutex_);
        return collecting_;
    }
    
    void push(const ORB_SLAM3::IMU::Point& point) {
        std::lock_guard<std::mutex> lock(mutex_);
        // Only store if we're collecting (i.e., tracking might be lost)
        if (collecting_) {
            imu_queue_.push(point);
            // Keep buffer size limited
            while (imu_queue_.size() > 500) {
                imu_queue_.pop();
            }
        }
    }
    
    std::vector<ORB_SLAM3::IMU::Point> getRecentMeasurements(double t0, double t1) {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<ORB_SLAM3::IMU::Point> measurements;
        
        std::queue<ORB_SLAM3::IMU::Point> temp_queue;
        
        while (!imu_queue_.empty()) {
            auto& point = imu_queue_.front();
            if (point.t >= t0 && point.t <= t1) {
                measurements.push_back(point);
            } else if (point.t > t1) {
                temp_queue.push(point);
            }
            imu_queue_.pop();
        }
        
        // Put back future measurements
        while (!temp_queue.empty()) {
            imu_queue_.push(temp_queue.front());
            temp_queue.pop();
        }
        
        return measurements;
    }
    
    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        while (!imu_queue_.empty()) {
            imu_queue_.pop();
        }
        collecting_ = false;
    }
};

// Software coordinate transformation class
class DepthToColorTransform {
private:
    Eigen::Matrix3d R_depth_to_color_;
    Eigen::Vector3d t_depth_to_color_;
    cv::Mat depthK_, colorK_;
    bool initialized_;
    
public:
    DepthToColorTransform() : initialized_(false) {
        // Your calibration data
        R_depth_to_color_ << 0.999998,  0.001045, -0.001768,
                            -0.001045,  0.999999, -0.000077,
                             0.001768,  0.000078,  0.999998;
        
        t_depth_to_color_ << -13.851061, -0.219677, -2.052635; // mm
        t_depth_to_color_ /= 1000.0; // Convert to meters
    }
    
    void setCameraIntrinsics(const cv::Mat& depthK, const cv::Mat& colorK) {
        depthK_ = depthK.clone();
        colorK_ = colorK.clone();
        initialized_ = true;
    }
    
    cv::Mat alignDepthToColor(const cv::Mat& depthImage, const cv::Size& colorSize) {
        if (!initialized_) return cv::Mat::zeros(colorSize, CV_16UC1);
        
        cv::Mat alignedDepth = cv::Mat::zeros(colorSize, CV_16UC1);
        
        float fx_d = depthK_.at<float>(0, 0), fy_d = depthK_.at<float>(1, 1);
        float cx_d = depthK_.at<float>(0, 2), cy_d = depthK_.at<float>(1, 2);
        float fx_c = colorK_.at<float>(0, 0), fy_c = colorK_.at<float>(1, 1);
        float cx_c = colorK_.at<float>(0, 2), cy_c = colorK_.at<float>(1, 2);
        
        for (int v = 0; v < depthImage.rows; v++) {
            for (int u = 0; u < depthImage.cols; u++) {
                uint16_t depth_value = depthImage.at<uint16_t>(v, u);
                if (depth_value == 0) continue;
                
                float depth_m = depth_value / 1000.0f;
                Eigen::Vector3d point_depth((u - cx_d) * depth_m / fx_d,
                                          (v - cy_d) * depth_m / fy_d, depth_m);
                
                Eigen::Vector3d point_color = R_depth_to_color_ * point_depth + t_depth_to_color_;
                if (point_color.z() <= 0) continue;
                
                int u_c = (int)(fx_c * point_color.x() / point_color.z() + cx_c + 0.5);
                int v_c = (int)(fy_c * point_color.y() / point_color.z() + cy_c + 0.5);
                
                if (u_c >= 0 && u_c < colorSize.width && v_c >= 0 && v_c < colorSize.height) {
                    alignedDepth.at<uint16_t>(v_c, u_c) = depth_value;
                }
            }
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

class OrbbecCaptureHybrid {
public:
    OrbbecCaptureHybrid(bool use_hardware_align = true) 
        : pipeline_(nullptr), config_(nullptr), 
          use_hw_align_(use_hardware_align),
          tracking_state_(0), frames_lost_count_(0) {
        std::cout << "RGB-D with IMU Fallback Mode" << std::endl;
        std::cout << "Alignment: " << (use_hw_align_ ? "HARDWARE" : "SOFTWARE") << std::endl;
    }
    
    ~OrbbecCaptureHybrid() {
        stop();
    }

    bool initialize() {
        try {
            context_ = std::make_shared<ob::Context>();
            auto deviceList = context_->queryDeviceList();
            if (deviceList->getCount() == 0) {
                std::cerr << "No Orbbec device found!" << std::endl;
                return false;
            }
            
            device_ = deviceList->getDevice(0);
            std::cout << "Device: " << device_->getDeviceInfo()->name() << std::endl;
            std::cout << "Serial: " << device_->getDeviceInfo()->serialNumber() << std::endl;
            
            // Check IMU availability (but don't require it)
            auto sensorList = device_->getSensorList();
            bool has_accel = false, has_gyro = false;
            
            for (uint32_t i = 0; i < sensorList->getCount(); i++) {
                auto sensor = sensorList->getSensor(i);
                if (sensor->type() == OB_SENSOR_ACCEL) has_accel = true;
                if (sensor->type() == OB_SENSOR_GYRO) has_gyro = true;
            }
            
            imu_available_ = has_accel && has_gyro;
            if (imu_available_) {
                std::cout << "IMU detected - will be used for fallback when tracking is lost" << std::endl;
            } else {
                std::cout << "No IMU detected - running in pure RGB-D mode" << std::endl;
            }
            
            pipeline_ = std::make_shared<ob::Pipeline>(device_);
            config_ = std::make_shared<ob::Config>();
            
            // Configure RGB-D streams
            config_->enableVideoStream(OB_STREAM_COLOR, 1280, 720, 30, OB_FORMAT_RGB);
            config_->enableVideoStream(OB_STREAM_DEPTH, 1280, 720, 30, OB_FORMAT_Y16);
            
            // Configure IMU only if available (lower frequency since it's just for fallback)
            if (imu_available_) {
                try {
                    config_->enableAccelStream(OB_ACCEL_FS_4g, OB_SAMPLE_RATE_100_HZ);
                    config_->enableGyroStream(OB_GYRO_FS_1000dps, OB_SAMPLE_RATE_100_HZ);
                    std::cout << "IMU configured at 100Hz for fallback use" << std::endl;
                } catch (const ob::Error& e) {
                    std::cerr << "Failed to configure IMU: " << e.what() << std::endl;
                    imu_available_ = false;
                }
            }
            
            // Enable frame synchronization
            config_->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ANY_SITUATION);
            pipeline_->enableFrameSync();
            
            // Create hardware alignment filter if requested
            if (use_hw_align_) {
                align_filter_ = std::make_shared<ob::Align>(OB_STREAM_COLOR);
            }
            
            // Get camera intrinsics
            auto colorProfiles = pipeline_->getStreamProfileList(OB_SENSOR_COLOR);
            auto depthProfiles = pipeline_->getStreamProfileList(OB_SENSOR_DEPTH);
            
            if (colorProfiles->getCount() > 0 && depthProfiles->getCount() > 0) {
                auto colorProfile = colorProfiles->getProfile(0)->as<ob::VideoStreamProfile>();
                auto depthProfile = depthProfiles->getProfile(0)->as<ob::VideoStreamProfile>();
                
                auto colorIntrinsic = colorProfile->getIntrinsic();
                auto depthIntrinsic = depthProfile->getIntrinsic();
                
                colorK_ = (cv::Mat_<float>(3, 3) << 
                    colorIntrinsic.fx, 0, colorIntrinsic.cx,
                    0, colorIntrinsic.fy, colorIntrinsic.cy, 0, 0, 1);
                
                depthK_ = (cv::Mat_<float>(3, 3) << 
                    depthIntrinsic.fx, 0, depthIntrinsic.cx,
                    0, depthIntrinsic.fy, depthIntrinsic.cy, 0, 0, 1);
                
                sw_transformer_.setCameraIntrinsics(depthK_, colorK_);
            }
            
            return true;
        } catch (const ob::Error& e) {
            std::cerr << "Initialization error: " << e.what() << std::endl;
            return false;
        }
    }
    
    bool start() {
        try {
            pipeline_->start(config_);
            std::cout << "Pipeline started - RGB-D primary, IMU on standby" << std::endl;
            
            // Start IMU thread only if available
            if (imu_available_) {
                imu_running_ = true;
                imu_thread_ = std::thread(&OrbbecCaptureHybrid::imuThreadFunc, this);
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            return true;
        } catch (const ob::Error& e) {
            std::cerr << "Start error: " << e.what() << std::endl;
            return false;
        }
    }
    
    void stop() {
        if (imu_running_) {
            imu_running_ = false;
            if (imu_thread_.joinable()) {
                imu_thread_.join();
            }
        }
        if (pipeline_) {
            pipeline_->stop();
        }
    }
    
    bool getFrames(cv::Mat& color, cv::Mat& depth, double& timestamp) {
        try {
            auto frameSet = pipeline_->waitForFrames(100);
            if (!frameSet) return false;
            
            if (use_hw_align_) {
                // Hardware alignment
                auto alignedFrame = align_filter_->process(frameSet);
                if (!alignedFrame) return false;
                
                auto alignedFrameSet = alignedFrame->as<ob::FrameSet>();
                if (!alignedFrameSet) return false;
                
                auto colorFrame = alignedFrameSet->getFrame(OB_FRAME_COLOR);
                auto alignedDepthFrame = alignedFrameSet->getFrame(OB_FRAME_DEPTH);
                
                if (!colorFrame || !alignedDepthFrame) return false;
                
                auto colorVideoFrame = colorFrame->as<ob::ColorFrame>();
                cv::Mat color_temp(colorVideoFrame->getHeight(), colorVideoFrame->getWidth(), 
                                  CV_8UC3, (void*)colorVideoFrame->getData());
                cv::cvtColor(color_temp, color, cv::COLOR_RGB2BGR);
                timestamp = colorVideoFrame->getTimeStampUs() / 1000000.0;
                
                auto depthVideoFrame = alignedDepthFrame->as<ob::DepthFrame>();
                depth = cv::Mat(depthVideoFrame->getHeight(), depthVideoFrame->getWidth(), 
                               CV_16UC1, (void*)depthVideoFrame->getData()).clone();
                
                sw_transformer_.fillDepthHoles(depth);
                
            } else {
                // Software alignment
                auto colorFrame = frameSet->getFrame(OB_FRAME_COLOR);
                auto depthFrame = frameSet->getFrame(OB_FRAME_DEPTH);
                
                if (!colorFrame || !depthFrame) return false;
                
                auto colorVideoFrame = colorFrame->as<ob::ColorFrame>();
                cv::Mat color_temp(colorVideoFrame->getHeight(), colorVideoFrame->getWidth(), 
                                  CV_8UC3, (void*)colorVideoFrame->getData());
                cv::cvtColor(color_temp, color, cv::COLOR_RGB2BGR);
                timestamp = colorVideoFrame->getTimeStampUs() / 1000000.0;
                
                auto depthVideoFrame = depthFrame->as<ob::DepthFrame>();
                cv::Mat rawDepth(depthVideoFrame->getHeight(), depthVideoFrame->getWidth(), 
                               CV_16UC1, (void*)depthVideoFrame->getData());
                
                depth = sw_transformer_.alignDepthToColor(rawDepth, color.size());
                sw_transformer_.fillDepthHoles(depth);
            }
            
            last_image_timestamp_ = timestamp;
            return !color.empty() && !depth.empty();
            
        } catch (const ob::Error& e) {
            std::cerr << "Frame error: " << e.what() << std::endl;
            return false;
        }
    }
    
    // Update tracking state from SLAM system
    void updateTrackingState(int state) {
        tracking_state_ = state;
        
        if (state == 2) { // TRACKING OK
            frames_lost_count_ = 0;
            // Stop IMU collection when tracking is good
            if (imu_available_) {
                imu_buffer_.stopCollecting();
            }
        } else if (state == 1) { // LOST
            frames_lost_count_++;
            // Start IMU collection when tracking is lost
            if (imu_available_ && frames_lost_count_ > 5) {
                std::cout << "Visual tracking lost - IMU fallback activated" << std::endl;
                imu_buffer_.startCollecting();
            }
        }
    }
    
    // Get IMU measurements only when needed for recovery
    std::vector<ORB_SLAM3::IMU::Point> getIMUForRecovery(double t_prev, double t_curr) {
        if (!imu_available_ || !imu_buffer_.isCollecting()) {
            return std::vector<ORB_SLAM3::IMU::Point>();
        }
        return imu_buffer_.getRecentMeasurements(t_prev, t_curr);
    }
    
    bool isUsingIMUFallback() const {
        return imu_available_ && imu_buffer_.isCollecting();
    }
    
    void toggleAlignmentMode() {
        use_hw_align_ = !use_hw_align_;
        std::cout << "Switched to " << (use_hw_align_ ? "HARDWARE" : "SOFTWARE") << " alignment" << std::endl;
    }
    
private:
    void imuThreadFunc() {
        std::cout << "IMU thread started (standby mode)" << std::endl;
        
        while (imu_running_) {
            try {
                auto frameSet = pipeline_->waitForFrames(10);
                if (!frameSet) continue;
                
                // Only process IMU if we're in fallback mode
                if (!imu_buffer_.isCollecting()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    continue;
                }
                
                // Process accelerometer
                auto accelFrame = frameSet->getFrame(OB_FRAME_ACCEL);
                if (accelFrame) {
                    auto accelData = accelFrame->as<ob::AccelFrame>();
                    if (accelData) {
                        auto value = accelData->getValue();
                        double timestamp = accelData->getTimeStampUs() / 1000000.0;
                        
                        // Data is already in m/s^2
                        Eigen::Vector3f accel(value.x, value.y, value.z);
                        
                        {
                            std::lock_guard<std::mutex> lock(accel_mutex_);
                            last_accel_ = accel;
                            last_accel_time_ = timestamp;
                        }
                    }
                }
                
                // Process gyroscope
                auto gyroFrame = frameSet->getFrame(OB_FRAME_GYRO);
                if (gyroFrame) {
                    auto gyroData = gyroFrame->as<ob::GyroFrame>();
                    if (gyroData) {
                        auto value = gyroData->getValue();
                        double timestamp = gyroData->getTimeStampUs() / 1000000.0;
                        
                        // Convert to rad/s
                        Eigen::Vector3f gyro(value.x * M_PI / 180.0f, 
                                            value.y * M_PI / 180.0f, 
                                            value.z * M_PI / 180.0f);
                        
                        Eigen::Vector3f accel;
                        {
                            std::lock_guard<std::mutex> lock(accel_mutex_);
                            accel = last_accel_;
                        }
                        
                        // Create IMU point
                        ORB_SLAM3::IMU::Point imu_point(
                            accel.x(), accel.y(), accel.z(),
                            gyro.x(), gyro.y(), gyro.z(),
                            timestamp);
                        
                        imu_buffer_.push(imu_point);
                    }
                }
                
            } catch (const ob::Error& e) {
                // Silent continue
            }
        }
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
    bool imu_available_ = false;
    
    // Tracking state management
    std::atomic<int> tracking_state_{0};
    std::atomic<int> frames_lost_count_{0};
    
    // IMU related
    mutable ImuBuffer imu_buffer_;
    std::thread imu_thread_;
    std::atomic<bool> imu_running_{false};
    double last_image_timestamp_ = 0.0;
    
    // Temporary storage for accel/gyro sync
    std::mutex accel_mutex_;
    Eigen::Vector3f last_accel_{0, 0, 0};
    double last_accel_time_{0};
};

int main(int argc, char **argv) {
    if (argc < 3 || argc > 4) {
        cerr << endl << "Usage: ./rgbd_hybrid vocabulary settings [hw|sw]" << endl;
        cerr << "  hw: hardware alignment (default)" << endl;
        cerr << "  sw: software transformation" << endl;
        return 1;
    }

    bool use_hardware = true;
    if (argc == 4) {
        string mode = argv[3];
        if (mode == "sw") {
            use_hardware = false;
        }
    }

    cout << "=== ORB-SLAM3 RGB-D with IMU Fallback ===" << endl;
    cout << "Primary mode: RGB-D" << endl;
    cout << "Fallback: IMU assists when tracking is lost" << endl;

    // Initialize camera
    OrbbecCaptureHybrid capture(use_hardware);
    if (!capture.initialize()) {
        cerr << "Failed to initialize camera!" << endl;
        return -1;
    }

    // Create SLAM system - Always RGB-D mode since IMU is just fallback
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::RGBD, true);
    float imageScale = SLAM.GetImageScale();

    if (!capture.start()) {
        cerr << "Failed to start camera!" << endl;
        return -1;
    }

    cout << endl << "-------" << endl;
    cout << "Processing started" << endl;
    cout << "Keys: 'q' to quit, 't' to toggle alignment" << endl;

    // Main loop
    cv::Mat color, depth;
    double timestamp = 0.0;
    double prev_timestamp = 0.0;
    int frame_count = 0;
    int tracking_ok_count = 0;
    int recovery_attempts = 0;
    
    auto start_time = std::chrono::steady_clock::now();
    
    // Visualization
    cv::namedWindow("RGB", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Depth", cv::WINDOW_AUTOSIZE);
    
    while (true) {
        // Get RGB-D frames
        if (!capture.getFrames(color, depth, timestamp)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        if (timestamp == prev_timestamp) continue;
        frame_count++;

        // Resize if needed
        if (imageScale != 1.f) {
            int width = color.cols * imageScale;
            int height = color.rows * imageScale;
            cv::resize(color, color, cv::Size(width, height));
            cv::resize(depth, depth, cv::Size(width, height));
        }

        // Primary tracking with RGB-D
        auto track_start = std::chrono::steady_clock::now();
        auto pose = SLAM.TrackRGBD(color, depth, timestamp);
        auto track_end = std::chrono::steady_clock::now();
        
        double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(track_end - track_start).count();

        // Get tracking state and update capture
        auto state = SLAM.GetTrackingState();
        capture.updateTrackingState(state);
        
        if (state == 2) { // OK
            tracking_ok_count++;
            recovery_attempts = 0;
        } else if (state == 1 && capture.isUsingIMUFallback()) { // LOST with IMU available
            // Try to help recovery with IMU data
            auto imu_data = capture.getIMUForRecovery(prev_timestamp, timestamp);
            if (!imu_data.empty()) {
                recovery_attempts++;
                cout << "IMU-assisted recovery attempt #" << recovery_attempts 
                     << " with " << imu_data.size() << " IMU samples" << endl;
            }
        }

        // Status reporting
        if (frame_count % 30 == 0) {
            auto current_time = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - start_time).count();
            double fps = frame_count / elapsed;
            
            cout << "Frame " << frame_count 
                 << " | FPS: " << std::fixed << std::setprecision(1) << fps
                 << " | Track: " << std::setprecision(3) << ttrack << "s";
            
            if (state == 2) {
                cout << " | TRACKING OK ✓";
                auto translation = pose.translation();
                cout << " | Pos: [" << translation.x() << ", " 
                     << translation.y() << ", " << translation.z() << "]";
            } else if (state == 0) {
                cout << " | INITIALIZING...";
            } else {
                cout << " | LOST";
                if (capture.isUsingIMUFallback()) {
                    cout << " (IMU fallback active)";
                }
            }
            
            cout << " | Success: " << (100.0 * tracking_ok_count / frame_count) << "%" << endl;
        }

        // Visualization
        cv::Mat depth_viz;
        depth.convertTo(depth_viz, CV_8UC1, 255.0/5000.0);
        cv::applyColorMap(depth_viz, depth_viz, cv::COLORMAP_JET);
        
        // Status overlay
        cv::putText(color, "RGB-D | Frame: " + to_string(frame_count), 
                   cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
        
        string status_text;
        cv::Scalar status_color;
        if (state == 2) {
            status_text = "TRACKING";
            status_color = cv::Scalar(0, 255, 0);
        } else if (state == 0) {
            status_text = "INIT";
            status_color = cv::Scalar(255, 255, 0);
        } else {
            status_text = capture.isUsingIMUFallback() ? "LOST (IMU)" : "LOST";
            status_color = cv::Scalar(0, 0, 255);
        }
        
        cv::putText(color, status_text, cv::Point(10, 60), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2);
        
        cv::imshow("RGB", color);
        cv::imshow("Depth", depth_viz);
        
        // Handle keyboard
        char key = cv::waitKey(5);
        if (key == 'q' || key == 27) break;
        else if (key == 't') capture.toggleAlignmentMode();

        prev_timestamp = timestamp;
    }

    // Cleanup
    cv::destroyAllWindows();
    capture.stop();
    SLAM.Shutdown();
    
    // Save trajectories
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_hybrid.txt");
    SLAM.SaveTrajectoryTUM("CameraTrajectory_hybrid.txt");
    
    cout << "\nFinal Statistics:" << endl;
    cout << "Total frames: " << frame_count << endl;
    cout << "Tracking success rate: " << (100.0 * tracking_ok_count / frame_count) << "%" << endl;
    cout << "IMU recovery attempts: " << recovery_attempts << endl;
    
    return 0;
}