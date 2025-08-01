/**
 * ORB-SLAM3 RGB-D-Inertial example for Orbbec Gemini 335 camera
 * Based on the working rgbd_orbbec_gemini335.cc with MINIMAL IMU support added
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

#include <System.h>
#include "ImuTypes.h"

// Orbbec SDK includes
#include "libobsensor/ObSensor.hpp"

using namespace std;

class OrbbecCapture {
public:
    OrbbecCapture() : pipeline_(nullptr), config_(nullptr), sync_enabled_(true) {}
    
    ~OrbbecCapture() {
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
            
            // Configure color and depth streams with specific resolutions
            config_->enableVideoStream(OB_STREAM_COLOR, 1920, 1080, 30, OB_FORMAT_RGB);
            config_->enableVideoStream(OB_STREAM_DEPTH, 848, 480, 30, OB_FORMAT_Y16);
            
            // Configure IMU streams - ADD THIS FOR IMU SUPPORT
            config_->enableAccelStream(OB_ACCEL_FS_4g, OB_SAMPLE_RATE_200_HZ);
            config_->enableGyroStream(OB_GYRO_FS_1000dps, OB_SAMPLE_RATE_200_HZ);
            
            // Enable frame synchronization
            config_->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);
            pipeline_->enableFrameSync();
            
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
            // Start pipeline - KEEP IT SIMPLE
            pipeline_->start(config_);
            
            std::cout << "Orbbec pipeline started successfully with IMU" << std::endl;
            return true;
        } catch (const ob::Error& e) {
            std::cerr << "Failed to start Orbbec pipeline: " << e.what() << std::endl;
            return false;
        }
    }
    
    bool getFrames(cv::Mat& color, cv::Mat& depth, double& timestamp, vector<ORB_SLAM3::IMU::Point>& vImuMeas) {
        try {
            auto frameSet = pipeline_->waitForFrames(1000); // 1 second timeout
            if (!frameSet) {
                std::cout << "No frameset received, retrying..." << std::endl;
                return false;
            }
            
            // Get color frame
            auto colorFrame = frameSet->getFrame(OB_FRAME_COLOR);
            auto depthFrame = frameSet->getFrame(OB_FRAME_DEPTH);
            
            if (!colorFrame || !depthFrame) {
                std::cout << "Missing color or depth frame, retrying..." << std::endl;
                return false;
            }
            
            // Process color frame
            auto colorVideoFrame = colorFrame->as<ob::ColorFrame>();
            int color_width = colorVideoFrame->getWidth();
            int color_height = colorVideoFrame->getHeight();
            
            // Convert to OpenCV Mat (RGB format)
            cv::Mat color_temp(color_height, color_width, CV_8UC3, (void*)colorVideoFrame->getData());
            cv::cvtColor(color_temp, color, cv::COLOR_RGB2BGR); // Convert to BGR for OpenCV
            
            timestamp = colorVideoFrame->getTimeStampUs() / 1000000.0; // Convert to seconds
            
            // Process depth frame
            auto depthVideoFrame = depthFrame->as<ob::DepthFrame>();
            int depth_width = depthVideoFrame->getWidth();
            int depth_height = depthVideoFrame->getHeight();
            
            // Convert to OpenCV Mat (16-bit depth)
            depth = cv::Mat(depth_height, depth_width, CV_16UC1, (void*)depthVideoFrame->getData()).clone();
            
            // Resize depth to match color if needed
            if (depth.size() != color.size()) {
                cv::resize(depth, depth, color.size(), 0, 0, cv::INTER_NEAREST);
            }
            
            // Process IMU frames - SIMPLE APPROACH
            vImuMeas.clear();
            
            // Get IMU frames
            auto accelFrame = frameSet->getFrame(OB_FRAME_ACCEL);
            auto gyroFrame = frameSet->getFrame(OB_FRAME_GYRO);
            
            // Add current IMU data to accumulated buffer
            if (accelFrame && gyroFrame) {
                auto accel = accelFrame->as<ob::AccelFrame>();
                auto gyro = gyroFrame->as<ob::GyroFrame>();
                
                auto accelValue = accel->getValue();
                auto gyroValue = gyro->getValue();
                double imuTime = gyro->getTimeStampUs() / 1000000.0;
                
                // Add to buffer
                imu_buffer_.push_back(ORB_SLAM3::IMU::Point(accelValue.x, accelValue.y, accelValue.z,
                                                            gyroValue.x, gyroValue.y, gyroValue.z, imuTime));
            }
            
            // Only send IMU data if we have enough measurements
            if (imu_buffer_.size() >= 5) {
                vImuMeas = imu_buffer_;
                imu_buffer_.clear();
            }
            
            std::cout << "Got frames - Color: " << color.size() << ", Depth: " << depth.size() 
                      << ", IMU measurements: " << vImuMeas.size() << std::endl;
            
            return !color.empty() && !depth.empty();
            
        } catch (const ob::Error& e) {
            std::cerr << "Error getting frames: " << e.what() << std::endl;
            return false;
        }
    }
    
private:
    std::shared_ptr<ob::Context> context_;
    std::shared_ptr<ob::Device> device_;
    std::shared_ptr<ob::Pipeline> pipeline_;
    std::shared_ptr<ob::Config> config_;
    std::shared_ptr<ob::Align> align_filter_;
    
    bool sync_enabled_;
    
    // Simple IMU buffer
    vector<ORB_SLAM3::IMU::Point> imu_buffer_;
};

int main(int argc, char **argv) {
    if (argc != 3 && argc != 4) {
        cerr << endl << "Usage: ./rgbd_inertial_orbbec_gemini335 path_to_vocabulary path_to_settings [path_to_sequence_folder_if_live]" << endl;
        return 1;
    }

    // Initialize Orbbec camera
    OrbbecCapture capture;
    if (!capture.initialize()) {
        cerr << "Failed to initialize Orbbec camera!" << endl;
        return -1;
    }

    // Create SLAM system - CHANGE TO IMU_RGBD MODE
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_RGBD, true);
    float imageScale = SLAM.GetImageScale();

    // Start camera
    if (!capture.start()) {
        cerr << "Failed to start Orbbec camera!" << endl;
        return -1;
    }

    cout << endl << "-------" << endl;
    cout << "Start processing sequence with IMU..." << endl;
    cout << "Controls:" << endl;
    cout << "  'q' or ESC: Quit" << endl;
    cout << "  's': Save current trajectory" << endl;
    cout << "  'p': Pause (press any key to continue)" << endl;
    cout << endl;

    // Main loop - KEEP THE SAME STRUCTURE
    cv::Mat color, depth;
    double timestamp = 0.0;
    vector<ORB_SLAM3::IMU::Point> vImuMeas;
    
    while (true) {
        // Get frames from camera
        if (!capture.getFrames(color, depth, timestamp, vImuMeas)) {
            std::cout << "Waiting for frames..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // Skip if no IMU data yet
        if (vImuMeas.empty()) {
            std::cout << "Accumulating IMU data..." << std::endl;
            continue;
        }

        // Resize images if needed
        if (imageScale != 1.f) {
            int width = color.cols * imageScale;
            int height = color.rows * imageScale;
            cv::resize(color, color, cv::Size(width, height), 0, 0, cv::INTER_LINEAR);
            cv::resize(depth, depth, cv::Size(width, height), 0, 0, cv::INTER_NEAREST);
        }

        // Pass the images and IMU data to the SLAM system
        auto start = std::chrono::steady_clock::now();
        SLAM.TrackRGBD(color, depth, timestamp, vImuMeas);
        auto end = std::chrono::steady_clock::now();
        
        double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count();

        // Display some info
        cout << "Frame processed in " << ttrack << " seconds with " << vImuMeas.size() << " IMU measurements" << endl;

        // Check for quit key and controls
        char key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 27) { // 'q' or ESC to quit
            break;
        }
        else if (key == 's') { // 's' to save map
            cout << "Saving map..." << endl;
            SLAM.SaveTrajectoryTUM("trajectory_" + to_string(time(0)) + ".txt");
            cout << "Map saved!" << endl;
        }
        else if (key == 'p') { // 'p' to pause/resume
            cout << "Press any key to continue..." << endl;
            cv::waitKey(0);
        }
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save multiple trajectory formats
    cout << "Saving trajectories..." << endl;
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveTrajectoryKITTI("CameraTrajectory_KITTI.txt");
    
    cout << "Saved files:" << endl;
    cout << "  KeyFrameTrajectory.txt (TUM format)" << endl;
    cout << "  CameraTrajectory.txt (TUM format)" << endl;
    cout << "  CameraTrajectory_KITTI.txt (KITTI format)" << endl;

    return 0;
}