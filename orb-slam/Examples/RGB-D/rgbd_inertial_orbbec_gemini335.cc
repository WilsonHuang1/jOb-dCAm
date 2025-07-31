/**
 * ORB-SLAM3 RGB-D-Inertial example for Orbbec Gemini 335 camera
 * Based on rgbd_inertial_realsense_D435i.cc but adapted for OrbbecSDK
 */

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <opencv2/opencv.hpp>

#include <System.h>

// Orbbec SDK includes
#include "libobsensor/ObSensor.hpp"

#include "ImuTypes.h"

using namespace std;

class OrbbecIMUCapture {
public:
    struct IMUData {
        double timestamp;
        float gyro[3];
        float accel[3];
    };
    
    OrbbecIMUCapture() : pipeline_(nullptr), config_(nullptr), 
                         frame_ready_(false), imu_enabled_(false) {}
    
    ~OrbbecIMUCapture() {
        if (pipeline_) {
            pipeline_->stop();
        }
    }
    
    bool initialize() {
        try {
            // Create context and get device
            context_ = std::make_shared<ob::Context>();
            auto deviceList = context_->queryDeviceList();
            
            if (deviceList->getCount() == 0) {
                std::cerr << "No Orbbec device found!" << std::endl;
                return false;
            }
            
            device_ = deviceList->getDevice(0);
            std::cout << "Device found: " << device_->getDeviceInfo()->name() << std::endl;
            std::cout << "Serial Number: " << device_->getDeviceInfo()->serialNumber() << std::endl;
            std::cout << "Firmware Version: " << device_->getDeviceInfo()->firmwareVersion() << std::endl;
            
            // Create pipeline
            pipeline_ = std::make_shared<ob::Pipeline>(device_);
            config_ = std::make_shared<ob::Config>();
            
            // Configure color and depth streams (640x480 for smaller display)
            config_->enableVideoStream(OB_STREAM_COLOR, 640, 480, 30, OB_FORMAT_RGB);
            config_->enableVideoStream(OB_STREAM_DEPTH, 640, 480, 30, OB_FORMAT_Y16);
            
            // Enable frame synchronization
            config_->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);
            pipeline_->enableFrameSync();
            
            // Enable IMU streams
            try {
                config_->enableAccelStream(OB_ACCEL_FS_4g, OB_SAMPLE_RATE_200_HZ);
                config_->enableGyroStream(OB_GYRO_FS_1000dps, OB_SAMPLE_RATE_200_HZ);
                imu_enabled_ = true;
                std::cout << "IMU streams enabled" << std::endl;
            } catch (const ob::Error& e) {
                std::cerr << "IMU not available: " << e.what() << std::endl;
                imu_enabled_ = false;
            }
            
            // Create alignment filter (depth to color)
            align_filter_ = std::make_shared<ob::Align>(OB_STREAM_COLOR);
            
            return true;
        } catch (const ob::Error& e) {
            std::cerr << "Orbbec initialization error: " << e.what() << std::endl;
            return false;
        }
    }
    
    bool start() {
        try {
            // Start with callback for synchronized frame and IMU data
            pipeline_->start(config_, [this](std::shared_ptr<ob::FrameSet> frameSet) {
                processFrameSet(frameSet);
            });
            
            std::cout << "Orbbec pipeline started successfully" << std::endl;
            return true;
        } catch (const ob::Error& e) {
            std::cerr << "Failed to start Orbbec pipeline: " << e.what() << std::endl;
            return false;
        }
    }
    
    bool waitForFrames(cv::Mat& color, cv::Mat& depth, double& timestamp,
                       std::vector<IMUData>& gyro_data, std::vector<IMUData>& accel_data) {
        std::unique_lock<std::mutex> lock(frame_mutex_);
        
        // Wait for new frame data
        if (!frame_ready_) {
            frame_condition_.wait(lock, [this] { return frame_ready_; });
        }
        
        if (current_color_.empty() || current_depth_.empty()) {
            frame_ready_ = false;
            return false;
        }
        
        // Copy frame data
        color = current_color_.clone();
        depth = current_depth_.clone();
        timestamp = current_timestamp_;
        
        // Copy IMU data accumulated since last frame  
        gyro_data = gyro_buffer_;
        accel_data = accel_buffer_;
        
        // Only clear buffers after copying (keep some history)
        if (gyro_buffer_.size() > 50) {
            gyro_buffer_.erase(gyro_buffer_.begin(), gyro_buffer_.begin() + 25);
        }
        if (accel_buffer_.size() > 50) {
            accel_buffer_.erase(accel_buffer_.begin(), accel_buffer_.begin() + 25);
        }
        frame_ready_ = false;
        
        return true;
    }
    
private:
    void processFrameSet(std::shared_ptr<ob::FrameSet> frameSet) {
        if (!frameSet) return;
        
        std::lock_guard<std::mutex> lock(frame_mutex_);
        
        // Process video frames
        auto colorFrame = frameSet->getFrame(OB_FRAME_COLOR);
        auto depthFrame = frameSet->getFrame(OB_FRAME_DEPTH);
        
        if (colorFrame && depthFrame) {
            // Color frame
            auto colorVideoFrame = colorFrame->as<ob::ColorFrame>();
            int color_width = colorVideoFrame->getWidth();
            int color_height = colorVideoFrame->getHeight();
            
            cv::Mat color_temp(color_height, color_width, CV_8UC3, (void*)colorVideoFrame->getData());
            cv::cvtColor(color_temp, current_color_, cv::COLOR_RGB2BGR);
            
            // Depth frame
            auto depthVideoFrame = depthFrame->as<ob::DepthFrame>();
            int depth_width = depthVideoFrame->getWidth();
            int depth_height = depthVideoFrame->getHeight();
            
            current_depth_ = cv::Mat(depth_height, depth_width, CV_16UC1, (void*)depthVideoFrame->getData()).clone();
            
            // Use color frame timestamp as reference
            current_timestamp_ = colorVideoFrame->getTimeStampUs() / 1000000.0; // Convert to seconds
            
            // Resize depth to match color if needed (simple approach)
            if (current_depth_.size() != current_color_.size()) {
                cv::resize(current_depth_, current_depth_, current_color_.size(), 0, 0, cv::INTER_NEAREST);
            }
        }
        
        // Process IMU data
        if (imu_enabled_) {
            auto gyroFrame = frameSet->getFrame(OB_FRAME_GYRO);
            auto accelFrame = frameSet->getFrame(OB_FRAME_ACCEL);
            
            if (gyroFrame) {
                auto gyro = gyroFrame->as<ob::GyroFrame>();
                auto gyroValue = gyro->getValue();
                
                IMUData gyro_data;
                gyro_data.timestamp = gyro->getTimeStampUs() / 1000000.0; // Convert to seconds
                gyro_data.gyro[0] = gyroValue.x;
                gyro_data.gyro[1] = gyroValue.y;
                gyro_data.gyro[2] = gyroValue.z;
                
                gyro_buffer_.push_back(gyro_data);
            }
            
            if (accelFrame) {
                auto accel = accelFrame->as<ob::AccelFrame>();
                auto accelValue = accel->getValue();
                
                IMUData accel_data;
                accel_data.timestamp = accel->getTimeStampUs() / 1000000.0; // Convert to seconds
                accel_data.accel[0] = accelValue.x;
                accel_data.accel[1] = accelValue.y;
                accel_data.accel[2] = accelValue.z;
                
                accel_buffer_.push_back(accel_data);
            }
        }
        
        frame_ready_ = true;
        frame_condition_.notify_one();
    }
    
    std::shared_ptr<ob::Context> context_;
    std::shared_ptr<ob::Device> device_;
    std::shared_ptr<ob::Pipeline> pipeline_;
    std::shared_ptr<ob::Config> config_;
    std::shared_ptr<ob::Align> align_filter_;
    
    bool imu_enabled_;
    
    // Frame synchronization
    std::mutex frame_mutex_;
    std::condition_variable frame_condition_;
    bool frame_ready_;
    
    cv::Mat current_color_;
    cv::Mat current_depth_;
    double current_timestamp_;
    
    // IMU data buffers
    std::vector<IMUData> gyro_buffer_;
    std::vector<IMUData> accel_buffer_;
};

int main(int argc, char **argv) {
    if (argc != 3 && argc != 4) {
        cerr << endl << "Usage: ./rgbd_inertial_orbbec_gemini335 path_to_vocabulary path_to_settings [save_trajectory_file]" << endl;
        return 1;
    }

    string save_file = "";
    if (argc == 4) {
        save_file = string(argv[3]);
    }

    // Initialize Orbbec camera with IMU
    OrbbecIMUCapture capture;
    if (!capture.initialize()) {
        cerr << "Failed to initialize Orbbec camera!" << endl;
        return -1;
    }

    // Create SLAM system (RGB-D-Inertial mode)
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_RGBD, true, 0, save_file);
    float imageScale = SLAM.GetImageScale();

    // Start camera
    if (!capture.start()) {
        cerr << "Failed to start Orbbec camera!" << endl;
        return -1;
    }

    cout << endl << "-------" << endl;
    cout << "Start processing sequence with IMU..." << endl;
    cout << "Press 'q' to quit" << endl;

    // Main loop
    cv::Mat color, depth;
    double timestamp = 0.0;
    std::vector<OrbbecIMUCapture::IMUData> gyro_data, accel_data;
    
    int frame_count = 0;
    
    while (true) {
        // Get frames and IMU data from camera
        if (!capture.waitForFrames(color, depth, timestamp, gyro_data, accel_data)) {
            continue;
        }

        // Skip frames with invalid data
        if (color.empty() || depth.empty()) {
            cout << "Empty frames received, skipping..." << endl;
            continue;
        }

        // Resize images if needed
        if (imageScale != 1.f) {
            int width = color.cols * imageScale;
            int height = color.rows * imageScale;
            cv::resize(color, color, cv::Size(width, height), 0, 0, cv::INTER_LINEAR);
            cv::resize(depth, depth, cv::Size(width, height), 0, 0, cv::INTER_NEAREST);
        }

        // Build IMU measurement vector for ORB-SLAM3
        vector<ORB_SLAM3::IMU::Point> vImuMeas;
        
        // Only process if we have both gyro and accel data
        if (!gyro_data.empty() && !accel_data.empty()) {
            // Create combined IMU measurements
            // Use the minimum count to avoid index out of bounds
            size_t min_count = std::min(gyro_data.size(), accel_data.size());
            
            for (size_t i = 0; i < min_count; i++) {
                const auto& gyro = gyro_data[i];
                const auto& accel = accel_data[i];
                
                // Create IMU measurement (accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, timestamp)
                // Use the gyro timestamp as reference
                ORB_SLAM3::IMU::Point imu_point(accel.accel[0], accel.accel[1], accel.accel[2],
                                               gyro.gyro[0], gyro.gyro[1], gyro.gyro[2], 
                                               gyro.timestamp);
                vImuMeas.push_back(imu_point);
            }
        }

        // Pass the images and IMU data to the SLAM system
        auto start = std::chrono::steady_clock::now();
        SLAM.TrackRGBD(color, depth, timestamp, vImuMeas);
        auto end = std::chrono::steady_clock::now();
        
        double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count();

        frame_count++;
        if (frame_count % 30 == 0) {  // Print every 30 frames
            cout << "Frame " << frame_count << " processed in " << ttrack << " seconds" << endl;
            cout << "IMU data: " << gyro_data.size() << " gyro, " << accel_data.size() << " accel samples" << endl;
            if (!vImuMeas.empty()) {
                cout << "Sent " << vImuMeas.size() << " IMU measurements to SLAM" << endl;
            }
        }

        // Check for quit key
        char key = cv::waitKey(1);
        if (key == 'q' || key == 'Q') {
            break;
        }
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}