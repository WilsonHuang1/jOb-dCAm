/**
 * ORB-SLAM3 Stereo example for Orbbec Gemini 335 camera
 * This implementation uses the left and right IR cameras for stereo SLAM
 */

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <iomanip>

#include <opencv2/opencv.hpp>
#include <System.h>

// Orbbec SDK includes
#include "libobsensor/ObSensor.hpp"

using namespace std;

class OrbbecStereoCapture {
public:
    OrbbecStereoCapture() 
        : pipeline_(nullptr), config_(nullptr), 
          frame_ready_(false), frame_count_(0) {
        std::cout << "Orbbec Gemini 335 - Stereo Mode" << std::endl;
    }
    
    ~OrbbecStereoCapture() {
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
            
            pipeline_ = std::make_shared<ob::Pipeline>(device_);
            config_ = std::make_shared<ob::Config>();
            
            // Check if device supports stereo IR streams
            auto sensorList = device_->getSensorList();
            bool hasLeftIR = false, hasRightIR = false;
            
            for (uint32_t i = 0; i < sensorList->getCount(); i++) {
                auto sensorType = sensorList->getSensorType(i);
                if (sensorType == OB_SENSOR_IR_LEFT) {
                    hasLeftIR = true;
                    std::cout << "Found left IR sensor" << std::endl;
                }
                if (sensorType == OB_SENSOR_IR_RIGHT) {
                    hasRightIR = true;
                    std::cout << "Found right IR sensor" << std::endl;
                }
            }
            
            if (!hasLeftIR || !hasRightIR) {
                std::cerr << "Device does not support stereo IR streams!" << std::endl;
                std::cerr << "Left IR: " << (hasLeftIR ? "YES" : "NO") << std::endl;
                std::cerr << "Right IR: " << (hasRightIR ? "YES" : "NO") << std::endl;
                return false;
            }
            
            // Configure stereo IR streams
            // Use same resolution and framerate for both cameras
            config_->enableVideoStream(OB_STREAM_IR_LEFT, 640, 480, 30, OB_FORMAT_Y8);
            config_->enableVideoStream(OB_STREAM_IR_RIGHT, 640, 480, 30, OB_FORMAT_Y8);
            
            // Enable frame synchronization
            config_->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ANY_SITUATION);
            pipeline_->enableFrameSync();
            
            // Get camera intrinsics and extrinsics
            if (!getCameraParameters()) {
                return false;
            }
            
            std::cout << "Stereo IR streams configured successfully" << std::endl;
            return true;
            
        } catch (const ob::Error& e) {
            std::cerr << "Orbbec error: " << e.getMessage() << std::endl;
            return false;
        } catch (const std::exception& e) {
            std::cerr << "Standard error: " << e.what() << std::endl;
            return false;
        }
    }
    
    bool getCameraParameters() {
        try {
            // Get stream profiles
            auto leftIRProfiles = pipeline_->getStreamProfileList(OB_SENSOR_IR_LEFT);
            auto rightIRProfiles = pipeline_->getStreamProfileList(OB_SENSOR_IR_RIGHT);
            
            if (leftIRProfiles->getCount() == 0 || rightIRProfiles->getCount() == 0) {
                std::cerr << "No IR stream profiles available" << std::endl;
                return false;
            }
            
            // Get intrinsics for left and right cameras
            auto leftProfile = leftIRProfiles->getProfile(0)->as<ob::VideoStreamProfile>();
            auto rightProfile = rightIRProfiles->getProfile(0)->as<ob::VideoStreamProfile>();
            
            auto leftIntrinsics = leftProfile->getIntrinsic();
            auto rightIntrinsics = rightProfile->getIntrinsic();
            
            std::cout << "\nLeft IR Camera Intrinsics:" << std::endl;
            std::cout << "  fx = " << leftIntrinsics.fx << std::endl;
            std::cout << "  fy = " << leftIntrinsics.fy << std::endl;
            std::cout << "  cx = " << leftIntrinsics.cx << std::endl;
            std::cout << "  cy = " << leftIntrinsics.cy << std::endl;
            std::cout << "  width = " << leftIntrinsics.width << std::endl;
            std::cout << "  height = " << leftIntrinsics.height << std::endl;
            
            std::cout << "\nRight IR Camera Intrinsics:" << std::endl;
            std::cout << "  fx = " << rightIntrinsics.fx << std::endl;
            std::cout << "  fy = " << rightIntrinsics.fy << std::endl;
            std::cout << "  cx = " << rightIntrinsics.cx << std::endl;
            std::cout << "  cy = " << rightIntrinsics.cy << std::endl;
            std::cout << "  width = " << rightIntrinsics.width << std::endl;
            std::cout << "  height = " << rightIntrinsics.height << std::endl;
            
            // Get extrinsics (right to left transformation)
            try {
                auto extrinsics = leftProfile->getExtrinsicTo(rightProfile);
                
                std::cout << "\nStereo Extrinsics (Right to Left):" << std::endl;
                std::cout << "Rotation matrix:" << std::endl;
                for (int i = 0; i < 3; i++) {
                    for (int j = 0; j < 3; j++) {
                        std::cout << extrinsics.rot[i*3 + j] << " ";
                    }
                    std::cout << std::endl;
                }
                
                std::cout << "Translation vector:" << std::endl;
                std::cout << extrinsics.trans[0] << " " << extrinsics.trans[1] << " " << extrinsics.trans[2] << std::endl;
                
                // Calculate baseline (distance between cameras)
                float baseline = sqrt(extrinsics.trans[0]*extrinsics.trans[0] + 
                                    extrinsics.trans[1]*extrinsics.trans[1] + 
                                    extrinsics.trans[2]*extrinsics.trans[2]);
                std::cout << "Baseline: " << baseline << " mm" << std::endl;
                
            } catch (const ob::Error& e) {
                std::cout << "Could not get extrinsics: " << e.getMessage() << std::endl;
                std::cout << "You will need to calibrate the stereo cameras manually" << std::endl;
            }
            
            return true;
            
        } catch (const ob::Error& e) {
            std::cerr << "Error getting camera parameters: " << e.getMessage() << std::endl;
            return false;
        }
    }
    
    bool start() {
        try {
            std::cout << "Starting stereo capture..." << std::endl;
            pipeline_->start(config_);
            std::cout << "Stereo capture started successfully" << std::endl;
            return true;
        } catch (const ob::Error& e) {
            std::cerr << "Error starting pipeline: " << e.getMessage() << std::endl;
            return false;
        }
    }
    
    void stop() {
        if (pipeline_) {
            pipeline_->stop();
            std::cout << "Pipeline stopped" << std::endl;
        }
    }
    
    bool getFrames(cv::Mat& leftImage, cv::Mat& rightImage, double& timestamp) {
        static int debug_frame_count = 0;
        static auto last_debug_time = std::chrono::steady_clock::now();
        static double last_timestamp = 0.0;
        static int timeout_count = 0;
        static int frame_drop_count = 0;
        
        try {
            auto start_time = std::chrono::steady_clock::now();
            
            // Wait for frameset with timeout
            auto frameset = pipeline_->waitForFrameset(100);
            
            auto wait_time = std::chrono::steady_clock::now();
            auto wait_duration = std::chrono::duration_cast<std::chrono::milliseconds>(wait_time - start_time).count();
            
            if (!frameset) {
                timeout_count++;
                if (timeout_count % 10 == 0) {
                    std::cout << "[DEBUG] Frame timeout #" << timeout_count 
                              << " - Wait time: " << wait_duration << "ms" << std::endl;
                }
                return false;
            }
            
            // Reset timeout count on successful frame
            if (timeout_count > 0) {
                std::cout << "[DEBUG] Recovered from " << timeout_count << " timeouts" << std::endl;
                timeout_count = 0;
            }
            
            // Get left and right IR frames
            auto leftFrame = frameset->getFrame(OB_FRAME_IR_LEFT);
            auto rightFrame = frameset->getFrame(OB_FRAME_IR_RIGHT);
            
            if (!leftFrame || !rightFrame) {
                frame_drop_count++;
                std::cout << "[DEBUG] Frame drop #" << frame_drop_count 
                          << " - Left: " << (leftFrame ? "OK" : "NULL") 
                          << ", Right: " << (rightFrame ? "OK" : "NULL") << std::endl;
                return false;
            }
            
            // Cast to VideoFrame to access width/height methods
            auto leftVideoFrame = leftFrame->as<ob::VideoFrame>();
            auto rightVideoFrame = rightFrame->as<ob::VideoFrame>();
            
            // Get frame dimensions and validate
            int width = leftVideoFrame->width();
            int height = leftVideoFrame->height();
            int right_width = rightVideoFrame->width();
            int right_height = rightVideoFrame->height();
            
            if (width != right_width || height != right_height) {
                std::cout << "[DEBUG] Resolution mismatch - Left: " << width << "x" << height 
                          << ", Right: " << right_width << "x" << right_height << std::endl;
                return false;
            }
            
            // Get timestamp and check for frame timing issues
            timestamp = leftVideoFrame->timeStamp() / 1000.0; // Convert to seconds
            double right_timestamp = rightVideoFrame->timeStamp() / 1000.0;
            
            // Check timestamp consistency
            double timestamp_diff = std::abs(timestamp - right_timestamp);
            if (timestamp_diff > 0.005) { // More than 5ms difference
                std::cout << "[DEBUG] Timestamp sync issue - Diff: " << timestamp_diff*1000 << "ms" << std::endl;
            }
            
            // Check for timestamp jumps or duplicates
            if (last_timestamp > 0) {
                double time_delta = timestamp - last_timestamp;
                if (time_delta < 0.001) { // Less than 1ms - possible duplicate
                    std::cout << "[DEBUG] Possible duplicate frame - Delta: " << time_delta*1000 << "ms" << std::endl;
                } else if (time_delta > 0.1) { // More than 100ms - possible jump
                    std::cout << "[DEBUG] Large timestamp jump - Delta: " << time_delta*1000 << "ms" << std::endl;
                }
            }
            last_timestamp = timestamp;
            
            auto frame_process_start = std::chrono::steady_clock::now();
            
            // Convert to OpenCV format with error checking
            try {
                leftImage = cv::Mat(height, width, CV_8UC1, leftVideoFrame->data()).clone();
                rightImage = cv::Mat(height, width, CV_8UC1, rightVideoFrame->data()).clone();
            } catch (const std::exception& e) {
                std::cout << "[DEBUG] OpenCV conversion error: " << e.what() << std::endl;
                return false;
            }
            
            // Validate image data
            if (leftImage.empty() || rightImage.empty()) {
                std::cout << "[DEBUG] Empty images after conversion" << std::endl;
                return false;
            }
            
            auto frame_process_end = std::chrono::steady_clock::now();
            auto process_duration = std::chrono::duration_cast<std::chrono::milliseconds>(frame_process_end - frame_process_start).count();
            auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(frame_process_end - start_time).count();
            
            frame_count_++;
            debug_frame_count++;
            
            // Print periodic debug info
            auto current_time = std::chrono::steady_clock::now();
            auto debug_elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_debug_time).count();
            
            if (debug_elapsed >= 5) { // Every 5 seconds
                double fps = debug_frame_count / double(debug_elapsed);
                std::cout << "[DEBUG] ===== Frame Statistics =====" << std::endl;
                std::cout << "[DEBUG] FPS: " << std::fixed << std::setprecision(2) << fps << std::endl;
                std::cout << "[DEBUG] Total frames: " << frame_count_ << std::endl;
                std::cout << "[DEBUG] Frame drops: " << frame_drop_count << std::endl;
                std::cout << "[DEBUG] Timeouts: " << timeout_count << std::endl;
                std::cout << "[DEBUG] Last wait time: " << wait_duration << "ms" << std::endl;
                std::cout << "[DEBUG] Last process time: " << process_duration << "ms" << std::endl;
                std::cout << "[DEBUG] Last total time: " << total_duration << "ms" << std::endl;
                std::cout << "[DEBUG] Image size: " << width << "x" << height << std::endl;
                std::cout << "[DEBUG] Timestamp: " << std::fixed << std::setprecision(6) << timestamp << std::endl;
                std::cout << "[DEBUG] =========================" << std::endl;
                
                debug_frame_count = 0;
                last_debug_time = current_time;
            }
            
            // Warn about slow processing
            if (total_duration > 50) { // More than 50ms
                std::cout << "[DEBUG] Slow frame processing: " << total_duration << "ms" << std::endl;
            }
            
            return true;
            
        } catch (const ob::Error& e) {
            std::cerr << "[DEBUG] OrbbecSDK error in getFrames: " << e.getMessage() << std::endl;
            std::cerr << "[DEBUG] Error function: " << e.getFunction() << std::endl;
            std::cerr << "[DEBUG] Error type: " << e.getExceptionType() << std::endl;
            return false;
        } catch (const std::exception& e) {
            std::cerr << "[DEBUG] Standard error in getFrames: " << e.what() << std::endl;
            return false;
        }
    }
    
    int getFrameCount() const { return frame_count_; }

private:
    std::shared_ptr<ob::Context> context_;
    std::shared_ptr<ob::Device> device_;
    std::shared_ptr<ob::Pipeline> pipeline_;
    std::shared_ptr<ob::Config> config_;
    
    bool frame_ready_;
    int frame_count_;
};

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);

int main(int argc, char **argv) {
    if (argc != 3) {
        cerr << endl << "Usage: ./stereo_orbbec_gemini335 path_to_vocabulary path_to_settings" << endl;
        return 1;
    }

    cout << endl << "-------" << endl;
    cout << "ORB-SLAM3 Stereo Mode with Orbbec Gemini 335" << endl;
    cout << "-------" << endl;

    // Initialize Orbbec camera
    OrbbecStereoCapture capture;
    if (!capture.initialize()) {
        cerr << "Failed to initialize Orbbec camera!" << endl;
        return -1;
    }

    // Create SLAM system
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, true);
    float imageScale = SLAM.GetImageScale();

    if (!capture.start()) {
        cerr << "Failed to start camera capture!" << endl;
        return -1;
    }

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: LIVE STREAM" << endl << endl;

    // Variables for frame processing
    cv::Mat imLeft, imRight;
    double tframe;
    
    // Main processing loop
    // Main processing loop
    std::cout << "[DEBUG] Starting main processing loop..." << std::endl;
    
    // Timing variables for main loop debugging
    auto loop_start_time = std::chrono::steady_clock::now();
    int total_processed_frames = 0;
    int failed_frame_gets = 0;
    int slam_failures = 0;
    
    while (true) {
        auto frame_start = std::chrono::steady_clock::now();
        
        // Get stereo frames
        if (!capture.getFrames(imLeft, imRight, tframe)) {
            failed_frame_gets++;
            if (failed_frame_gets % 100 == 0) {
                std::cout << "[DEBUG] Failed to get frames " << failed_frame_gets << " times" << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        if (imLeft.empty() || imRight.empty()) {
            std::cerr << "[DEBUG] Empty images received" << std::endl;
            continue;
        }

        auto frame_get_time = std::chrono::steady_clock::now();
        auto frame_get_duration = std::chrono::duration_cast<std::chrono::milliseconds>(frame_get_time - frame_start).count();

        // Resize images if needed
        if (imageScale != 1.f) {
            auto resize_start = std::chrono::steady_clock::now();
            int width = imLeft.cols * imageScale;
            int height = imLeft.rows * imageScale;
            cv::resize(imLeft, imLeft, cv::Size(width, height));
            cv::resize(imRight, imRight, cv::Size(width, height));
            auto resize_end = std::chrono::steady_clock::now();
            auto resize_duration = std::chrono::duration_cast<std::chrono::milliseconds>(resize_end - resize_start).count();
            
            if (resize_duration > 10) {
                std::cout << "[DEBUG] Slow resize operation: " << resize_duration << "ms" << std::endl;
            }
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the images to the SLAM system
        try {
            SLAM.TrackStereo(imLeft, imRight, tframe);
        } catch (const std::exception& e) {
            slam_failures++;
            std::cerr << "[DEBUG] SLAM tracking failed: " << e.what() << std::endl;
            if (slam_failures % 10 == 0) {
                std::cout << "[DEBUG] SLAM failures: " << slam_failures << std::endl;
            }
            continue;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
        
        auto total_frame_time = std::chrono::steady_clock::now();
        auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(total_frame_time - frame_start).count();

        total_processed_frames++;

        // Display frame rate and processing time with more details
        if (total_processed_frames % 30 == 0) { // Every 30 frames
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed_total = std::chrono::duration_cast<std::chrono::seconds>(current_time - loop_start_time).count();
            double avg_fps = total_processed_frames / double(elapsed_total);
            
            std::cout << "[DEBUG] ===== MAIN LOOP STATS =====" << std::endl;
            std::cout << "[DEBUG] Frame " << capture.getFrameCount() 
                      << " | Processed: " << total_processed_frames << std::endl;
            std::cout << "[DEBUG] Average FPS: " << std::fixed << std::setprecision(2) << avg_fps << std::endl;
            std::cout << "[DEBUG] Frame get: " << frame_get_duration << "ms" << std::endl;
            std::cout << "[DEBUG] SLAM track: " << std::fixed << std::setprecision(3) << ttrack*1000 << "ms" << std::endl;
            std::cout << "[DEBUG] Total frame: " << total_duration << "ms" << std::endl;
            std::cout << "[DEBUG] Failed gets: " << failed_frame_gets << std::endl;
            std::cout << "[DEBUG] SLAM failures: " << slam_failures << std::endl;
            std::cout << "[DEBUG] =========================" << std::endl;
        }

        // Warn about slow processing
        if (ttrack > 0.1) { // More than 100ms
            std::cout << "[DEBUG] Very slow SLAM tracking: " << ttrack*1000 << "ms" << std::endl;
        }
        
        if (total_duration > 100) { // More than 100ms total
            std::cout << "[DEBUG] Very slow total processing: " << total_duration << "ms" << std::endl;
        }

        // Check for exit condition (you can modify this)
        char key = cv::waitKey(1) & 0xFF;
        if (key == 27 || key == 'q') { // ESC or 'q' to quit
            cout << "[DEBUG] Exit requested by user" << endl;
            break;
        }
        
        // Emergency exit if too many failures
        if (failed_frame_gets > 1000) {
            std::cerr << "[DEBUG] Too many frame failures, exiting..." << std::endl;
            break;
        }
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    capture.stop();
    cout << "Stereo SLAM completed successfully" << endl;
    
    return 0;
}