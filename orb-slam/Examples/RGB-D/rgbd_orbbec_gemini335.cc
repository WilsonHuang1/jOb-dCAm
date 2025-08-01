/**
 * ORB-SLAM3 RGB-D example for Orbbec Gemini 335 camera
 * Based on rgbd_realsense_D435i.cc but adapted for OrbbecSDK
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
            // Start pipeline
            pipeline_->start(config_);
            
            std::cout << "Orbbec pipeline started successfully" << std::endl;
            return true;
        } catch (const ob::Error& e) {
            std::cerr << "Failed to start Orbbec pipeline: " << e.what() << std::endl;
            return false;
        }
    }
    
    bool getFrames(cv::Mat& color, cv::Mat& depth, double& timestamp) {
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
            
            std::cout << "Got frames - Color: " << color.size() << ", Depth: " << depth.size() << std::endl;
            
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
};

int main(int argc, char **argv) {
    if (argc != 3 && argc != 4) {
        cerr << endl << "Usage: ./rgbd_orbbec_gemini335 path_to_vocabulary path_to_settings [path_to_sequence_folder_if_live]" << endl;
        return 1;
    }

    // Initialize Orbbec camera
    OrbbecCapture capture;
    if (!capture.initialize()) {
        cerr << "Failed to initialize Orbbec camera!" << endl;
        return -1;
    }

    // Create SLAM system
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::RGBD, true);
    float imageScale = SLAM.GetImageScale();

    // Start camera
    if (!capture.start()) {
        cerr << "Failed to start Orbbec camera!" << endl;
        return -1;
    }

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Press 'q' to quit" << endl;

    // Main loop
    cv::Mat color, depth;
    double timestamp = 0.0;
    
    while (true) {
        // Get frames from camera
        if (!capture.getFrames(color, depth, timestamp)) {
            std::cout << "Waiting for frames..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // Resize images if needed
        if (imageScale != 1.f) {
            int width = color.cols * imageScale;
            int height = color.rows * imageScale;
            cv::resize(color, color, cv::Size(width, height), 0, 0, cv::INTER_LINEAR);
            cv::resize(depth, depth, cv::Size(width, height), 0, 0, cv::INTER_NEAREST);
        }

        // Pass the images to the SLAM system
        auto start = std::chrono::steady_clock::now();
        SLAM.TrackRGBD(color, depth, timestamp);
        auto end = std::chrono::steady_clock::now();
        
        double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count();

        // Display some info
        cout << "Frame processed in " << ttrack << " seconds" << endl;

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