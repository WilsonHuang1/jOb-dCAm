/**
 * ORB-SLAM3 RGB-D example for Orbbec Gemini 335 camera
 * MERGED VERSION - Combines working hardware alignment with optional software transformation
 */

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <thread>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include <System.h>

// Orbbec SDK includes
#include "libobsensor/ObSensor.hpp"

using namespace std;

// Software coordinate transformation class (for comparison/debugging)
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
        
        std::cout << "Software coordinate transformer initialized." << std::endl;
    }
    
    void setCameraIntrinsics(const cv::Mat& depthK, const cv::Mat& colorK) {
        depthK_ = depthK.clone();
        colorK_ = colorK.clone();
        initialized_ = true;
        std::cout << "Transformer ready for software alignment." << std::endl;
    }
    
    cv::Mat alignDepthToColor(const cv::Mat& depthImage, const cv::Size& colorSize) {
        if (!initialized_) return cv::Mat::zeros(colorSize, CV_16UC1);
        
        cv::Mat alignedDepth = cv::Mat::zeros(colorSize, CV_16UC1);
        
        float fx_d = depthK_.at<float>(0, 0), fy_d = depthK_.at<float>(1, 1);
        float cx_d = depthK_.at<float>(0, 2), cy_d = depthK_.at<float>(1, 2);
        float fx_c = colorK_.at<float>(0, 0), fy_c = colorK_.at<float>(1, 1);
        float cx_c = colorK_.at<float>(0, 2), cy_c = colorK_.at<float>(1, 2);
        
        int valid_points = 0;
        
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
                    valid_points++;
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

class OrbbecCapture {
public:
    OrbbecCapture(bool use_hardware_align = true) 
        : pipeline_(nullptr), config_(nullptr), use_hw_align_(use_hardware_align) {
        std::cout << "Using " << (use_hw_align_ ? "HARDWARE" : "SOFTWARE") << " alignment" << std::endl;
    }
    
    ~OrbbecCapture() {
        if (pipeline_) pipeline_->stop();
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
            
            // Configure streams - use same resolution for both to avoid issues
            config_->enableVideoStream(OB_STREAM_COLOR, 1280, 720, 30, OB_FORMAT_RGB);
            config_->enableVideoStream(OB_STREAM_DEPTH, 1280, 720, 30, OB_FORMAT_Y16);
            
            // Enable frame synchronization
            config_->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ANY_SITUATION);
            pipeline_->enableFrameSync();
            
            // Create hardware alignment filter if requested
            if (use_hw_align_) {
                align_filter_ = std::make_shared<ob::Align>(OB_STREAM_COLOR);
                std::cout << "Hardware alignment filter created" << std::endl;
            }
            
            // Get camera intrinsics for software transformation
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
                
                // Initialize software transformer (even if using hardware align, for comparison)
                sw_transformer_.setCameraIntrinsics(depthK_, colorK_);
                
                std::cout << "Color intrinsics: fx=" << colorIntrinsic.fx << ", fy=" << colorIntrinsic.fy 
                         << ", cx=" << colorIntrinsic.cx << ", cy=" << colorIntrinsic.cy << std::endl;
                std::cout << "Depth intrinsics: fx=" << depthIntrinsic.fx << ", fy=" << depthIntrinsic.fy 
                         << ", cx=" << depthIntrinsic.cx << ", cy=" << depthIntrinsic.cy << std::endl;
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
            std::cout << "Camera started successfully" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            return true;
        } catch (const ob::Error& e) {
            std::cerr << "Start error: " << e.what() << std::endl;
            return false;
        }
    }
    
    bool getFrames(cv::Mat& color, cv::Mat& depth, double& timestamp) {
        try {
            auto frameSet = pipeline_->waitForFrames(1000);
            if (!frameSet) return false;
            
            if (use_hw_align_) {
                // Use hardware alignment
                auto alignedFrame = align_filter_->process(frameSet);
                if (!alignedFrame) {
                    std::cout << "Hardware alignment failed, retrying..." << std::endl;
                    return false;
                }
                
                auto alignedFrameSet = alignedFrame->as<ob::FrameSet>();
                if (!alignedFrameSet) {
                    std::cout << "Failed to convert aligned frame" << std::endl;
                    return false;
                }
                
                auto colorFrame = alignedFrameSet->getFrame(OB_FRAME_COLOR);
                auto alignedDepthFrame = alignedFrameSet->getFrame(OB_FRAME_DEPTH);
                
                if (!colorFrame || !alignedDepthFrame) return false;
                
                // Process color
                auto colorVideoFrame = colorFrame->as<ob::ColorFrame>();
                cv::Mat color_temp(colorVideoFrame->getHeight(), colorVideoFrame->getWidth(), 
                                  CV_8UC3, (void*)colorVideoFrame->getData());
                cv::cvtColor(color_temp, color, cv::COLOR_RGB2BGR);
                timestamp = colorVideoFrame->getTimeStampUs() / 1000000.0;
                
                // Process hardware-aligned depth
                auto depthVideoFrame = alignedDepthFrame->as<ob::DepthFrame>();
                depth = cv::Mat(depthVideoFrame->getHeight(), depthVideoFrame->getWidth(), 
                               CV_16UC1, (void*)depthVideoFrame->getData()).clone();
                
                // Apply hole filling on hardware-aligned depth
                sw_transformer_.fillDepthHoles(depth);
                
            } else {
                // Use software alignment with your transformation matrix
                auto colorFrame = frameSet->getFrame(OB_FRAME_COLOR);
                auto depthFrame = frameSet->getFrame(OB_FRAME_DEPTH);
                
                if (!colorFrame || !depthFrame) return false;
                
                // Process color
                auto colorVideoFrame = colorFrame->as<ob::ColorFrame>();
                cv::Mat color_temp(colorVideoFrame->getHeight(), colorVideoFrame->getWidth(), 
                                  CV_8UC3, (void*)colorVideoFrame->getData());
                cv::cvtColor(color_temp, color, cv::COLOR_RGB2BGR);
                timestamp = colorVideoFrame->getTimeStampUs() / 1000000.0;
                
                // Process raw depth
                auto depthVideoFrame = depthFrame->as<ob::DepthFrame>();
                cv::Mat rawDepth(depthVideoFrame->getHeight(), depthVideoFrame->getWidth(), 
                               CV_16UC1, (void*)depthVideoFrame->getData());
                
                // Apply software coordinate transformation
                depth = sw_transformer_.alignDepthToColor(rawDepth, color.size());
                sw_transformer_.fillDepthHoles(depth);
            }
            
            // Verify sizes match
            if (depth.size() != color.size()) {
                std::cerr << "Size mismatch! Color: " << color.size() << ", Depth: " << depth.size() << std::endl;
                // Resize depth to match color if needed
                cv::resize(depth, depth, color.size(), 0, 0, cv::INTER_NEAREST);
            }
            
            return !color.empty() && !depth.empty();
            
        } catch (const ob::Error& e) {
            std::cerr << "Frame error: " << e.what() << std::endl;
            return false;
        }
    }
    
    void toggleAlignmentMode() {
        use_hw_align_ = !use_hw_align_;
        std::cout << "Switched to " << (use_hw_align_ ? "HARDWARE" : "SOFTWARE") << " alignment" << std::endl;
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
            cout << "Using SOFTWARE coordinate transformation" << endl;
        } else if (mode == "hw") {
            cout << "Using HARDWARE alignment" << endl;
        } else {
            cerr << "Invalid alignment mode. Use 'hw' or 'sw'" << endl;
            return 1;
        }
    }

    OrbbecCapture capture(use_hardware);
    if (!capture.initialize()) {
        cerr << "Failed to initialize camera!" << endl;
        return -1;
    }

    // Initialize SLAM in RGB-D mode
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::RGBD, true);
    float imageScale = SLAM.GetImageScale();

    if (!capture.start()) {
        cerr << "Failed to start camera!" << endl;
        return -1;
    }

    cout << "=== RGB-D SLAM Started ===" << endl;
    cout << "Press 'q' to quit, 't' to toggle alignment mode" << endl;
    cout << "Move camera slowly for initialization" << endl;

    cv::Mat im, depthmap;
    double timestamp = 0.0;
    int frame_count = 0;
    int tracking_ok_count = 0;
    
    auto start_time = std::chrono::steady_clock::now();
    
    // Create visualization windows
    cv::namedWindow("RGB", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Depth", cv::WINDOW_AUTOSIZE);
    
    while (true) {
        // Get frames with alignment
        if (!capture.getFrames(im, depthmap, timestamp)) {
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

        // Track with SLAM
        auto track_start = std::chrono::steady_clock::now();
        auto pose = SLAM.TrackRGBD(im, depthmap, timestamp);
        auto track_end = std::chrono::steady_clock::now();
        
        double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(track_end - track_start).count();
        frame_count++;

        // Get tracking state
        auto state = SLAM.GetTrackingState();
        if (state == 2) { // OK state
            tracking_ok_count++;
        }

        // Status reporting every 30 frames
        if (frame_count % 30 == 0) {
            auto current_time = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - start_time).count();
            double fps = frame_count / elapsed;
            
            cout << "Frame " << frame_count 
                 << " | FPS: " << std::fixed << std::setprecision(1) << fps
                 << " | Track: " << std::setprecision(3) << ttrack << "s";
            
            if (state == 2) { // OK
                cout << " | STATUS: TRACKING OK";
            } else if (state == 0) { // NOT_INITIALIZED
                cout << " | STATUS: INITIALIZING...";
            } else {
                cout << " | STATUS: LOST";
            }
            
            cout << " | Success: " << (100.0 * tracking_ok_count / frame_count) << "%" << endl;
        }

        // Visualization
        cv::Mat depth_viz;
        depthmap.convertTo(depth_viz, CV_8UC1, 255.0/5000.0);
        cv::applyColorMap(depth_viz, depth_viz, cv::COLORMAP_JET);
        
        // Add status text
        cv::putText(im, "Frame: " + to_string(frame_count), cv::Point(10, 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        
        string status_text = (state == 2) ? "TRACKING" : (state == 0) ? "INIT" : "LOST";
        cv::Scalar status_color = (state == 2) ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
        cv::putText(im, status_text, cv::Point(10, 60), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2);
        
        cv::imshow("RGB", im);
        cv::imshow("Depth", depth_viz);
        
        // Check for key press
        char key = cv::waitKey(1);
        if (key == 'q' || key == 27) { // 'q' or ESC
            break;
        } else if (key == 't') {
            capture.toggleAlignmentMode();
        }

        // Save sample frames
        if (frame_count == 100) {
            cv::imwrite("sample_color_frame100.png", im);
            cv::imwrite("sample_depth_frame100.png", depthmap);
            cout << "Saved sample frames at frame 100" << endl;
        }
    }

    // Cleanup
    cv::destroyAllWindows();
    
    // Save trajectory
    SLAM.Shutdown();
    string suffix = use_hardware ? "_hw" : "_sw";
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory" + suffix + ".txt");
    SLAM.SaveTrajectoryTUM("CameraTrajectory" + suffix + ".txt");
    cout << "Trajectories saved!" << endl;

    return 0;
}