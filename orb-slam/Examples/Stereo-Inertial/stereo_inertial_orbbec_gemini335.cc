/**
 * ORB-SLAM3 Stereo-Inertial example for Orbbec Gemini 335 camera
 * This implementation uses the left and right IR cameras + IMU for stereo-inertial SLAM
 */

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <iomanip>

#include <opencv2/opencv.hpp>
#include <System.h>

// Orbbec SDK includes
#include "libobsensor/ObSensor.hpp"

using namespace std;

// IMU data structure
struct IMUData {
    double timestamp;
    cv::Point3f acc;
    cv::Point3f gyr;
    
    IMUData(double t, cv::Point3f a, cv::Point3f g) : timestamp(t), acc(a), gyr(g) {}
};

class OrbbecStereoInertialCapture {
public:
    OrbbecStereoInertialCapture() 
        : camera_pipeline_(nullptr), imu_pipeline_(nullptr), 
          camera_config_(nullptr), imu_config_(nullptr),
          frame_ready_(false), frame_count_(0), imu_count_(0) {
        std::cout << "Orbbec Gemini 335 - Stereo-Inertial Mode" << std::endl;
    }
    
    ~OrbbecStereoInertialCapture() {
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
            
            // Initialize camera pipeline
            camera_pipeline_ = std::make_shared<ob::Pipeline>(device_);
            camera_config_ = std::make_shared<ob::Config>();
            
            // Initialize IMU pipeline (separate pipeline for better performance)
            imu_pipeline_ = std::make_shared<ob::Pipeline>(device_);
            imu_config_ = std::make_shared<ob::Config>();
            
            // Check available sensors
            auto sensorList = device_->getSensorList();
            bool hasLeftIR = false, hasRightIR = false, hasAccel = false, hasGyro = false;
            
            for (uint32_t i = 0; i < sensorList->getCount(); i++) {
                auto sensorType = sensorList->getSensorType(i);
                switch (sensorType) {
                    case OB_SENSOR_IR_LEFT:
                        hasLeftIR = true;
                        std::cout << "Found left IR sensor" << std::endl;
                        break;
                    case OB_SENSOR_IR_RIGHT:
                        hasRightIR = true;
                        std::cout << "Found right IR sensor" << std::endl;
                        break;
                    case OB_SENSOR_ACCEL:
                        hasAccel = true;
                        std::cout << "Found accelerometer" << std::endl;
                        break;
                    case OB_SENSOR_GYRO:
                        hasGyro = true;
                        std::cout << "Found gyroscope" << std::endl;
                        break;
                }
            }
            
            if (!hasLeftIR || !hasRightIR) {
                std::cerr << "Device does not support stereo IR streams!" << std::endl;
                return false;
            }
            
            if (!hasAccel || !hasGyro) {
                std::cerr << "Device does not support IMU sensors!" << std::endl;
                std::cerr << "Accelerometer: " << (hasAccel ? "YES" : "NO") << std::endl;
                std::cerr << "Gyroscope: " << (hasGyro ? "YES" : "NO") << std::endl;
                return false;
            }
            
            // Configure camera streams (use same resolution as working stereo)
            camera_config_->enableVideoStream(OB_STREAM_IR_LEFT, 1280, 800, 30, OB_FORMAT_Y8);
            camera_config_->enableVideoStream(OB_STREAM_IR_RIGHT, 1280, 800, 30, OB_FORMAT_Y8);
            camera_config_->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ANY_SITUATION);
            camera_pipeline_->enableFrameSync();
            
            // Configure IMU streams
            imu_config_->enableAccelStream();
            imu_config_->enableGyroStream();
            
            // Get camera parameters (same as stereo)
            if (!getCameraParameters()) {
                return false;
            }
            
            // Get IMU parameters
            getIMUParameters();
            
            std::cout << "Stereo-Inertial configuration completed successfully" << std::endl;
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
            auto leftIRProfiles = camera_pipeline_->getStreamProfileList(OB_SENSOR_IR_LEFT);
            auto rightIRProfiles = camera_pipeline_->getStreamProfileList(OB_SENSOR_IR_RIGHT);
            
            if (leftIRProfiles->getCount() == 0 || rightIRProfiles->getCount() == 0) {
                std::cerr << "No IR stream profiles available" << std::endl;
                return false;
            }
            
            auto leftProfile = leftIRProfiles->getProfile(0)->as<ob::VideoStreamProfile>();
            auto rightProfile = rightIRProfiles->getProfile(0)->as<ob::VideoStreamProfile>();
            
            auto leftIntrinsics = leftProfile->getIntrinsic();
            auto rightIntrinsics = rightProfile->getIntrinsic();
            
            std::cout << "\nLeft IR Camera Intrinsics:" << std::endl;
            std::cout << "  fx = " << leftIntrinsics.fx << std::endl;
            std::cout << "  fy = " << leftIntrinsics.fy << std::endl;
            std::cout << "  cx = " << leftIntrinsics.cx << std::endl;
            std::cout << "  cy = " << leftIntrinsics.cy << std::endl;
            
            std::cout << "\nRight IR Camera Intrinsics:" << std::endl;
            std::cout << "  fx = " << rightIntrinsics.fx << std::endl;
            std::cout << "  fy = " << rightIntrinsics.fy << std::endl;
            std::cout << "  cx = " << rightIntrinsics.cx << std::endl;
            std::cout << "  cy = " << rightIntrinsics.cy << std::endl;
            
            return true;
            
        } catch (const ob::Error& e) {
            std::cerr << "Error getting camera parameters: " << e.getMessage() << std::endl;
            return false;
        }
    }
    
    void getIMUParameters() {
        try {
            std::cout << "\n=== IMU PARAMETERS ===" << std::endl;
            
            // Get IMU stream profiles
            auto accelProfiles = imu_pipeline_->getStreamProfileList(OB_SENSOR_ACCEL);
            auto gyroProfiles = imu_pipeline_->getStreamProfileList(OB_SENSOR_GYRO);
            
            if (accelProfiles->getCount() > 0) {
                auto accelProfile = accelProfiles->getProfile(0)->as<ob::AccelStreamProfile>();
                std::cout << "Accelerometer available - Sample rate: " << accelProfile->getSampleRate() << " Hz" << std::endl;
            }
            
            if (gyroProfiles->getCount() > 0) {
                auto gyroProfile = gyroProfiles->getProfile(0)->as<ob::GyroStreamProfile>();
                std::cout << "Gyroscope available - Sample rate: " << gyroProfile->getSampleRate() << " Hz" << std::endl;
            }
            
            std::cout << "Note: IMU intrinsic parameters should be calibrated manually" << std::endl;
            std::cout << "Using default noise characteristics from YAML file" << std::endl;
            
        } catch (const ob::Error& e) {
            std::cout << "Warning: Could not get IMU parameters: " << e.getMessage() << std::endl;
        }
    }
    
    bool start() {
        try {
            std::cout << "\nStarting camera pipeline..." << std::endl;
            camera_pipeline_->start(camera_config_);
            
            std::cout << "Starting IMU pipeline..." << std::endl;
            
            // Start IMU with callback for real-time data
            imu_pipeline_->start(imu_config_, [this](std::shared_ptr<ob::FrameSet> frameSet) {
                this->imuCallback(frameSet);
            });
            
            std::cout << "Stereo-Inertial capture started successfully" << std::endl;
            return true;
            
        } catch (const ob::Error& e) {
            std::cerr << "Error starting pipelines: " << e.getMessage() << std::endl;
            return false;
        }
    }
    
    void stop() {
        if (camera_pipeline_) {
            camera_pipeline_->stop();
            std::cout << "Camera pipeline stopped" << std::endl;
        }
        if (imu_pipeline_) {
            imu_pipeline_->stop();
            std::cout << "IMU pipeline stopped" << std::endl;
        }
    }
    
    void imuCallback(std::shared_ptr<ob::FrameSet> frameSet) {
        static int imu_debug_count = 0;
        static auto last_imu_debug = std::chrono::steady_clock::now();
        
        try {
            if (!frameSet) return;
            
            // Process accelerometer data
            auto accelFrame = frameSet->getFrame(OB_FRAME_ACCEL);
            if (accelFrame) {
                auto accel = accelFrame->as<ob::AccelFrame>();
                auto accelValue = accel->value();
                double timestamp = accel->timeStamp() / 1000.0; // Convert to seconds
                
                cv::Point3f acc(accelValue.x, accelValue.y, accelValue.z);
                
                {
                    std::lock_guard<std::mutex> lock(imu_mutex_);
                    imu_buffer_.emplace(timestamp, acc, cv::Point3f(0,0,0)); // Placeholder for gyro
                }
                
                imu_count_++;
            }
            
            // Process gyroscope data
            auto gyroFrame = frameSet->getFrame(OB_FRAME_GYRO);
            if (gyroFrame) {
                auto gyro = gyroFrame->as<ob::GyroFrame>();
                auto gyroValue = gyro->value();
                double timestamp = gyro->timeStamp() / 1000.0; // Convert to seconds
                
                cv::Point3f gyr(gyroValue.x, gyroValue.y, gyroValue.z);
                
                {
                    std::lock_guard<std::mutex> lock(imu_mutex_);
                    // In real implementation, you'd match accel and gyro by timestamp
                    // For now, we'll create separate entries
                    imu_buffer_.emplace(timestamp, cv::Point3f(0,0,0), gyr); // Placeholder for accel
                }
            }
            
            // Debug output every 5 seconds
            imu_debug_count++;
            auto current_time = std::chrono::steady_clock::now();
            auto debug_elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_imu_debug).count();
            
            if (debug_elapsed >= 5) {
                double imu_rate = imu_debug_count / double(debug_elapsed);
                std::cout << "[DEBUG IMU] Rate: " << std::fixed << std::setprecision(1) << imu_rate << " Hz" 
                          << " | Buffer size: " << imu_buffer_.size() << std::endl;
                imu_debug_count = 0;
                last_imu_debug = current_time;
            }
            
        } catch (const ob::Error& e) {
            std::cerr << "[DEBUG IMU] Error in IMU callback: " << e.getMessage() << std::endl;
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
            auto frameset = camera_pipeline_->waitForFrameset(100);
            
            auto wait_time = std::chrono::steady_clock::now();
            auto wait_duration = std::chrono::duration_cast<std::chrono::milliseconds>(wait_time - start_time).count();
            
            if (!frameset) {
                timeout_count++;
                if (timeout_count % 10 == 0) {
                    std::cout << "[DEBUG CAMERA] Frame timeout #" << timeout_count 
                              << " - Wait time: " << wait_duration << "ms" << std::endl;
                }
                return false;
            }
            
            // Reset timeout count on successful frame
            if (timeout_count > 0) {
                std::cout << "[DEBUG CAMERA] Recovered from " << timeout_count << " timeouts" << std::endl;
                timeout_count = 0;
            }
            
            // Get left and right IR frames
            auto leftFrame = frameset->getFrame(OB_FRAME_IR_LEFT);
            auto rightFrame = frameset->getFrame(OB_FRAME_IR_RIGHT);
            
            if (!leftFrame || !rightFrame) {
                frame_drop_count++;
                std::cout << "[DEBUG CAMERA] Frame drop #" << frame_drop_count 
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
                std::cout << "[DEBUG CAMERA] Resolution mismatch - Left: " << width << "x" << height 
                          << ", Right: " << right_width << "x" << right_height << std::endl;
                return false;
            }
            
            // Get timestamp and check for frame timing issues
            timestamp = leftVideoFrame->timeStamp() / 1000.0; // Convert to seconds
            double right_timestamp = rightVideoFrame->timeStamp() / 1000.0;
            
            // Check timestamp consistency
            double timestamp_diff = std::abs(timestamp - right_timestamp);
            if (timestamp_diff > 0.005) { // More than 5ms difference
                std::cout << "[DEBUG CAMERA] Timestamp sync issue - Diff: " << timestamp_diff*1000 << "ms" << std::endl;
            }
            
            // Convert to OpenCV format with error checking
            try {
                leftImage = cv::Mat(height, width, CV_8UC1, leftVideoFrame->data()).clone();
                rightImage = cv::Mat(height, width, CV_8UC1, rightVideoFrame->data()).clone();
            } catch (const std::exception& e) {
                std::cout << "[DEBUG CAMERA] OpenCV conversion error: " << e.what() << std::endl;
                return false;
            }
            
            // Validate image data
            if (leftImage.empty() || rightImage.empty()) {
                std::cout << "[DEBUG CAMERA] Empty images after conversion" << std::endl;
                return false;
            }
            
            frame_count_++;
            debug_frame_count++;
            
            // Print periodic debug info
            auto current_time = std::chrono::steady_clock::now();
            auto debug_elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_debug_time).count();
            
            if (debug_elapsed >= 5) { // Every 5 seconds
                double fps = debug_frame_count / double(debug_elapsed);
                
                std::lock_guard<std::mutex> lock(imu_mutex_);
                
                std::cout << "[DEBUG STEREO-INERTIAL] ===== Statistics =====" << std::endl;
                std::cout << "[DEBUG STEREO-INERTIAL] Camera FPS: " << std::fixed << std::setprecision(2) << fps << std::endl;
                std::cout << "[DEBUG STEREO-INERTIAL] Total frames: " << frame_count_ << std::endl;
                std::cout << "[DEBUG STEREO-INERTIAL] Total IMU: " << imu_count_ << std::endl;
                std::cout << "[DEBUG STEREO-INERTIAL] IMU buffer: " << imu_buffer_.size() << std::endl;
                std::cout << "[DEBUG STEREO-INERTIAL] Frame drops: " << frame_drop_count << std::endl;
                std::cout << "[DEBUG STEREO-INERTIAL] =========================" << std::endl;
                
                debug_frame_count = 0;
                last_debug_time = current_time;
            }
            
            return true;
            
        } catch (const ob::Error& e) {
            std::cerr << "[DEBUG CAMERA] OrbbecSDK error: " << e.getMessage() << std::endl;
            return false;
        } catch (const std::exception& e) {
            std::cerr << "[DEBUG CAMERA] Standard error: " << e.what() << std::endl;
            return false;
        }
    }
    
    // Get IMU data between two timestamps for ORB-SLAM3
    std::vector<IMUData> getIMUData(double start_time, double end_time) {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        std::vector<IMUData> result;
        
        // Clean old data (older than 1 second)
        while (!imu_buffer_.empty() && (end_time - imu_buffer_.front().timestamp) > 1.0) {
            imu_buffer_.pop();
        }
        
        // Extract data in time range
        std::queue<IMUData> temp_buffer = imu_buffer_;
        while (!temp_buffer.empty()) {
            const auto& imu_data = temp_buffer.front();
            if (imu_data.timestamp >= start_time && imu_data.timestamp <= end_time) {
                result.push_back(imu_data);
            }
            temp_buffer.pop();
        }
        
        return result;
    }
    
    int getFrameCount() const { return frame_count_; }
    int getIMUCount() const { return imu_count_; }

private:
    std::shared_ptr<ob::Context> context_;
    std::shared_ptr<ob::Device> device_;
    std::shared_ptr<ob::Pipeline> camera_pipeline_;
    std::shared_ptr<ob::Pipeline> imu_pipeline_;
    std::shared_ptr<ob::Config> camera_config_;
    std::shared_ptr<ob::Config> imu_config_;
    
    bool frame_ready_;
    int frame_count_;
    int imu_count_;
    
    // IMU data buffer and synchronization
    std::queue<IMUData> imu_buffer_;
    std::mutex imu_mutex_;
};

int main(int argc, char **argv) {
    if (argc != 3) {
        cerr << endl << "Usage: ./stereo_inertial_orbbec_gemini335 path_to_vocabulary path_to_settings" << endl;
        return 1;
    }

    cout << endl << "-------" << endl;
    cout << "ORB-SLAM3 Stereo-Inertial Mode with Orbbec Gemini 335" << endl;
    cout << "-------" << endl;

    // Initialize Orbbec camera
    OrbbecStereoInertialCapture capture;
    if (!capture.initialize()) {
        cerr << "Failed to initialize Orbbec camera!" << endl;
        return -1;
    }

    // Create SLAM system for stereo-inertial
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_STEREO, true);
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
    double prev_timestamp = 0.0;
    
    // Main processing loop
    std::cout << "[DEBUG] Starting stereo-inertial processing loop..." << std::endl;
    
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

        // Resize images if needed
        if (imageScale != 1.f) {
            int width = imLeft.cols * imageScale;
            int height = imLeft.rows * imageScale;
            cv::resize(imLeft, imLeft, cv::Size(width, height));
            cv::resize(imRight, imRight, cv::Size(width, height));
        }

        // Get IMU data for this frame
        std::vector<IMUData> imu_data;
        if (prev_timestamp > 0) {
            imu_data = capture.getIMUData(prev_timestamp, tframe);
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        try {
            if (imu_data.empty() && prev_timestamp > 0) {
                // No IMU data available, skip this frame or use stereo-only tracking
                SLAM.TrackStereo(imLeft, imRight, tframe);
            } else {
                // Convert IMU data to ORB-SLAM3 format and track
                std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
                for (const auto& imu : imu_data) {
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(
                        imu.acc.x, imu.acc.y, imu.acc.z,
                        imu.gyr.x, imu.gyr.y, imu.gyr.z,
                        imu.timestamp
                    ));
                }
                
                if (!vImuMeas.empty()) {
                    SLAM.TrackStereo(imLeft, imRight, tframe, vImuMeas);
                } else {
                    SLAM.TrackStereo(imLeft, imRight, tframe);
                }
            }
        } catch (const std::exception& e) {
            slam_failures++;
            std::cerr << "[DEBUG] SLAM tracking failed: " << e.what() << std::endl;
            continue;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
        
        total_processed_frames++;
        prev_timestamp = tframe;

        // Display frame rate and processing time with more details
        if (total_processed_frames % 30 == 0) { // Every 30 frames
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed_total = std::chrono::duration_cast<std::chrono::seconds>(current_time - loop_start_time).count();
            double avg_fps = total_processed_frames / double(elapsed_total);
            
            std::cout << "[DEBUG STEREO-INERTIAL] ===== MAIN LOOP STATS =====" << std::endl;
            std::cout << "[DEBUG STEREO-INERTIAL] Frame " << capture.getFrameCount() 
                      << " | Processed: " << total_processed_frames << std::endl;
            std::cout << "[DEBUG STEREO-INERTIAL] Average FPS: " << std::fixed << std::setprecision(2) << avg_fps << std::endl;
            std::cout << "[DEBUG STEREO-INERTIAL] SLAM track: " << std::fixed << std::setprecision(3) << ttrack*1000 << "ms" << std::endl;
            std::cout << "[DEBUG STEREO-INERTIAL] IMU measurements: " << imu_data.size() << std::endl;
            std::cout << "[DEBUG STEREO-INERTIAL] Total IMU: " << capture.getIMUCount() << std::endl;
            std::cout << "[DEBUG STEREO-INERTIAL] Failed gets: " << failed_frame_gets << std::endl;
            std::cout << "[DEBUG STEREO-INERTIAL] SLAM failures: " << slam_failures << std::endl;
            std::cout << "[DEBUG STEREO-INERTIAL] =============================" << std::endl;
        }

        // Check for exit condition
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
    cout << "Stereo-Inertial SLAM completed successfully" << endl;
    
    return 0;
}