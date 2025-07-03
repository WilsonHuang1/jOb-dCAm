// Enhanced Aurora 900 Distance Detection with Tracking and Filtering
// Based on your existing distance_detection_test.cc

// Essential C++ headers
#include <iostream>
#include <iomanip>
#include <thread>
#include <vector>
#include <algorithm>
#include <cmath>
#include <deque>
#include <memory>
#include <string>
#include <chrono>
#include <functional>

// Deptrum SDK headers
#include "deptrum/device.h"
#include "deptrum/stream.h"
#include "deptrum/aurora900_series.h"
#include "functional/base.h"
#include "functional/frame_rate_helper.h"
#include "sample_helper.h"
#include "viewer_helper.hpp"

// OpenCV headers
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/video/background_segm.hpp"

// OpenCV tracking (with fallback if not available)
#ifdef OPENCV_TRACKING_AVAILABLE
#include "opencv2/tracking.hpp"
#define HAS_TRACKING_MODULE 1
#else
#define HAS_TRACKING_MODULE 0
#endif

using namespace std;
using namespace deptrum;
using namespace deptrum::stream;

// Enhanced detection parameters
struct EnhancedDistanceParams {
    float roi_x_ratio = 0.3f;
    float roi_y_ratio = 0.3f;
    uint16_t min_depth = 100;
    uint16_t max_depth = 3000;
    int min_valid_points = 50;
    bool enable_tracking = true;
    bool enable_motion_detection = true;
    bool enable_kalman_filtering = true;
    float tracking_confidence_threshold = 0.7f;
} enhanced_params;

// Simple Kalman Filter for smoothing measurements
class SimpleKalmanFilter {
private:
    cv::KalmanFilter kf;
    bool initialized = false;
    
public:
    SimpleKalmanFilter() {
        // 4 state variables (x, y, vx, vy), 2 measurements (x, y)
        kf.init(4, 2, 0);
        
        // Transition matrix (constant velocity model)
        kf.transitionMatrix = (cv::Mat_<float>(4, 4) << 
            1, 0, 1, 0,   // x, vx
            0, 1, 0, 1,   // y, vy  
            0, 0, 1, 0,   // vx
            0, 0, 0, 1);  // vy
        
        cv::setIdentity(kf.measurementMatrix);
        cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(1e-4));
        cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(1e-1));
        cv::setIdentity(kf.errorCovPost, cv::Scalar::all(0.1));
    }
    
    cv::Point2f update(cv::Point2f measurement_point) {
        if (!initialized) {
            kf.statePre.at<float>(0) = measurement_point.x;
            kf.statePre.at<float>(1) = measurement_point.y;
            kf.statePre.at<float>(2) = 0; // velocity
            kf.statePre.at<float>(3) = 0;
            initialized = true;
            return measurement_point;
        }
        
        cv::Mat prediction = kf.predict();
        cv::Mat measurement = (cv::Mat_<float>(2,1) << measurement_point.x, measurement_point.y);
        cv::Mat estimated = kf.correct(measurement);
        
        return cv::Point2f(estimated.at<float>(0), estimated.at<float>(1));
    }
    
    void reset() {
        initialized = false;
    }
};

// Enhanced distance detection result
struct EnhancedDistance {
    float distance_mm;
    float center_x, center_y;
    int valid_points;
    float confidence;
    bool is_tracked;
    cv::Rect2d tracking_bbox;
    
    EnhancedDistance() : distance_mm(0), center_x(0), center_y(0), 
                        valid_points(0), confidence(0), is_tracked(false) {}
};

// Simple motion detector
class SimpleMotionDetector {
private:
    cv::Ptr<cv::BackgroundSubtractor> bg_subtractor;
    cv::Mat mask, kernel;
    bool initialized = false;
    
public:
    SimpleMotionDetector() {
        try {
            bg_subtractor = cv::createBackgroundSubtractorMOG2(500, 16, false);
            kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        } catch (const cv::Exception& e) {
            std::cout << "Warning: Motion detector initialization failed: " << e.what() << std::endl;
            enhanced_params.enable_motion_detection = false;
        }
    }
    
    std::vector<cv::Rect> detectMotion(cv::Mat& frame) {
        if (frame.empty() || !bg_subtractor) return {};
        
        try {
            bg_subtractor->apply(frame, mask);
            
            // Clean up mask
            cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
            cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
            
            // Find contours
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            
            std::vector<cv::Rect> motion_rects;
            for (const auto& contour : contours) {
                cv::Rect rect = cv::boundingRect(contour);
                if (rect.area() > 500) { // Filter small movements
                    motion_rects.push_back(rect);
                }
            }
            
            return motion_rects;
        } catch (const cv::Exception& e) {
            std::cout << "Motion detection failed: " << e.what() << std::endl;
            return {};
        }
    }
};

// Enhanced tracking and filtering system
class EnhancedTrackingSystem {
private:
    cv::Ptr<cv::Tracker> tracker;
    SimpleKalmanFilter position_filter;
    SimpleKalmanFilter distance_filter;
    SimpleMotionDetector motion_detector;
    
    std::deque<float> distance_history;
    std::deque<cv::Point2f> position_history;
    const size_t HISTORY_SIZE = 10;
    
    bool tracking_active = false;
    cv::Rect2d current_bbox;
    int tracking_lost_frames = 0;
    const int MAX_LOST_FRAMES = 10;
    
public:
    EnhancedTrackingSystem() {
#if HAS_TRACKING_MODULE
        if (enhanced_params.enable_tracking) {
            try {
                tracker = cv::TrackerKCF::create(); // Fast and reliable
            } catch (const cv::Exception& e) {
                std::cout << "Warning: OpenCV tracking not available: " << e.what() << std::endl;
                enhanced_params.enable_tracking = false;
            }
        }
#else
        std::cout << "Warning: OpenCV tracking module not available. Tracking disabled." << std::endl;
        enhanced_params.enable_tracking = false;
#endif
    }
    
    void initializeTracking(cv::Mat& frame, cv::Rect2d bbox) {
#if HAS_TRACKING_MODULE
        if (!enhanced_params.enable_tracking || frame.empty() || !tracker) return;
        
        try {
            if (tracker->init(frame, bbox)) {
                tracking_active = true;
                current_bbox = bbox;
                tracking_lost_frames = 0;
                position_filter.reset();
                distance_filter = SimpleKalmanFilter(); // Reset distance filter
            }
        } catch (const cv::Exception& e) {
            std::cout << "Tracking initialization failed: " << e.what() << std::endl;
            tracking_active = false;
        }
#endif
    }
    
    bool updateTracking(cv::Mat& frame, cv::Rect2d& updated_bbox) {
#if HAS_TRACKING_MODULE
        if (!tracking_active || !tracker || frame.empty()) return false;
        
        try {
            if (tracker->update(frame, updated_bbox)) {
                current_bbox = updated_bbox;
                tracking_lost_frames = 0;
                return true;
            } else {
                tracking_lost_frames++;
                if (tracking_lost_frames > MAX_LOST_FRAMES) {
                    tracking_active = false;
                    tracker.release();
                    if (enhanced_params.enable_tracking) {
                        tracker = cv::TrackerKCF::create();
                    }
                }
                return false;
            }
        } catch (const cv::Exception& e) {
            std::cout << "Tracking update failed: " << e.what() << std::endl;
            tracking_active = false;
            return false;
        }
#else
        return false;
#endif
    }
    
    cv::Point2f smoothPosition(cv::Point2f raw_position) {
        if (!enhanced_params.enable_kalman_filtering) return raw_position;
        
        cv::Point2f smoothed = position_filter.update(raw_position);
        
        // Maintain position history
        position_history.push_back(smoothed);
        if (position_history.size() > HISTORY_SIZE) {
            position_history.pop_front();
        }
        
        return smoothed;
    }
    
    float smoothDistance(float raw_distance) {
        if (!enhanced_params.enable_kalman_filtering) return raw_distance;
        
        cv::Point2f smoothed_point = distance_filter.update(cv::Point2f(raw_distance, 0));
        float smoothed_distance = smoothed_point.x;
        
        // Maintain distance history
        distance_history.push_back(smoothed_distance);
        if (distance_history.size() > HISTORY_SIZE) {
            distance_history.pop_front();
        }
        
        return smoothed_distance;
    }
    
    std::vector<cv::Rect> detectMotion(cv::Mat& frame) {
        if (!enhanced_params.enable_motion_detection) return {};
        return motion_detector.detectMotion(frame);
    }
    
    bool isTrackingActive() const { return tracking_active; }
    cv::Rect2d getCurrentBbox() const { return current_bbox; }
    
    float getDistanceStability() const {
        if (distance_history.size() < 3) return 0.0f;
        
        float mean = 0;
        for (float d : distance_history) mean += d;
        mean /= distance_history.size();
        
        float variance = 0;
        for (float d : distance_history) {
            variance += (d - mean) * (d - mean);
        }
        variance /= distance_history.size();
        
        return 1.0f / (1.0f + sqrt(variance) / mean); // Stability score [0,1]
    }
};

// Global tracking system
std::unique_ptr<EnhancedTrackingSystem> tracking_system;
bool is_running = true;
bool is_print_fps = false;
std::shared_ptr<ViewerHelper> viewer_helper;
CameraParam g_camera_param;

// Enhanced distance calculation with multi-point sampling
EnhancedDistance calculateEnhancedDistance(std::shared_ptr<StreamFrame> depth_frame, 
                                         cv::Rect2d* tracking_bbox = nullptr) {
    EnhancedDistance result;
    
    if (!depth_frame || depth_frame->frame_type != FrameType::kDepthFrame || !depth_frame->data) {
        return result;
    }
    
    uint16_t* depth_data = static_cast<uint16_t*>(depth_frame->data.get());
    int width = depth_frame->cols;
    int height = depth_frame->rows;
    
    // Determine ROI - use tracking bbox if available and active
    cv::Rect2d roi;
    if (tracking_bbox && tracking_system && tracking_system->isTrackingActive()) {
        roi = *tracking_bbox;
        result.is_tracked = true;
        result.tracking_bbox = roi;
    } else {
        // Use default ROI
        int roi_width = static_cast<int>(width * enhanced_params.roi_x_ratio);
        int roi_height = static_cast<int>(height * enhanced_params.roi_y_ratio);
        int roi_start_x = (width - roi_width) / 2;
        int roi_start_y = (height - roi_height) / 2;
        roi = cv::Rect2d(roi_start_x, roi_start_y, roi_width, roi_height);
        result.is_tracked = false;
    }
    
    // Ensure ROI is within bounds
    roi.x = std::max(0.0, std::min(roi.x, (double)(width - 1)));
    roi.y = std::max(0.0, std::min(roi.y, (double)(height - 1)));
    roi.width = std::min(roi.width, (double)(width - roi.x));
    roi.height = std::min(roi.height, (double)(height - roi.y));
    
    int roi_start_x = static_cast<int>(roi.x);
    int roi_start_y = static_cast<int>(roi.y);
    int roi_end_x = static_cast<int>(roi.x + roi.width);
    int roi_end_y = static_cast<int>(roi.y + roi.height);
    
    // Multi-level sampling for better accuracy
    std::vector<uint16_t> center_samples, edge_samples;
    float sum_x = 0, sum_y = 0, sum_center_x = 0, sum_center_y = 0;
    int center_count = 0;
    
    // Define center region (inner 60% of ROI)
    int center_margin_x = static_cast<int>(roi.width * 0.2);
    int center_margin_y = static_cast<int>(roi.height * 0.2);
    int center_x1 = roi_start_x + center_margin_x;
    int center_y1 = roi_start_y + center_margin_y;
    int center_x2 = roi_end_x - center_margin_x;
    int center_y2 = roi_end_y - center_margin_y;
    
    // Collect samples with spatial weighting
    for (int y = roi_start_y; y < roi_end_y; y += 2) { // Skip every other pixel for speed
        for (int x = roi_start_x; x < roi_end_x; x += 2) {
            int idx = y * width + x;
            if (idx < 0 || idx >= width * height) continue;
            
            uint16_t depth = depth_data[idx];
            if (depth >= enhanced_params.min_depth && depth <= enhanced_params.max_depth) {
                // Check if this pixel is in center region
                if (x >= center_x1 && x <= center_x2 && y >= center_y1 && y <= center_y2) {
                    center_samples.push_back(depth);
                    sum_center_x += x;
                    sum_center_y += y;
                    center_count++;
                } else {
                    edge_samples.push_back(depth);
                }
                sum_x += x;
                sum_y += y;
            }
        }
    }
    
    // Process samples with center preference
    std::vector<uint16_t> valid_depths;
    
    // Prefer center samples (3:1 ratio)
    for (size_t i = 0; i < center_samples.size(); i++) {
        valid_depths.push_back(center_samples[i]);
        valid_depths.push_back(center_samples[i]); // Double weight
        valid_depths.push_back(center_samples[i]); // Triple weight
    }
    
    // Add edge samples
    for (size_t i = 0; i < edge_samples.size(); i++) {
        valid_depths.push_back(edge_samples[i]);
    }
    
    if (valid_depths.size() >= enhanced_params.min_valid_points) {
        std::sort(valid_depths.begin(), valid_depths.end());
        
        // Use 25th percentile for better foreground detection
        size_t index = std::min(valid_depths.size() - 1, 
                               static_cast<size_t>(valid_depths.size() * 0.25));
        result.distance_mm = valid_depths[index];
        
        // Use weighted center if center samples exist
        if (center_count > 0) {
            result.center_x = sum_center_x / center_count;
            result.center_y = sum_center_y / center_count;
        } else {
            result.center_x = roi_start_x + roi.width / 2;
            result.center_y = roi_start_y + roi.height / 2;
        }
        
        result.valid_points = valid_depths.size();
        
        // Calculate confidence based on sample consistency
        float mean = result.distance_mm;
        float variance = 0;
        int sample_size = std::min(valid_depths.size(), static_cast<size_t>(100));
        for (size_t i = 0; i < sample_size; i++) {
            float diff = valid_depths[i] - mean;
            variance += diff * diff;
        }
        variance /= sample_size;
        result.confidence = 1.0f / (1.0f + sqrt(variance) / mean);
    }
    
    return result;
}

// Enhanced ShowFrame with advanced tracking overlay
void ShowFrameWithEnhancedTracking(const StreamFrames& frames, CameraParam camera_param = {}) {
    EnhancedDistance detection;
    cv::Mat rgb_display;
    cv::Rect2d tracking_bbox;
    bool tracking_updated = false;
    
    for (int index = 0; index < frames.count; index++) {
        auto frame = frames.frame_ptr[index];
        if (!frame || !frame->data) continue;
        
        FrameType frame_type = frame->frame_type;
        std::string title;
        
        if (frame->frame_attr != FrameAttr::kAttrInvalid) {
            title = frame_type_to_string_map[frame_type] + "_" +
                    frame_attr_to_string_map[frame->frame_attr];
        } else {
            title = frame_type_to_string_map[frame_type];
        }

        // Skip point cloud frames
        if (kPointCloudFrame == frame->frame_type || kRgbdPointCloudFrame == frame->frame_type ||
            kRgbdIrPointCloudFrame == frame->frame_type) {
            continue;
        }

        // Initialize viewer if needed
        auto iter = viewer_helper->viewer_set_.find(title);
        if (iter == viewer_helper->viewer_set_.end()) {
            cv::namedWindow(title, cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
            cv::resizeWindow(title, frame->cols, frame->rows);
            viewer_helper->viewer_set_.emplace(title);
        }

        switch (frame_type) {
            case kRgbFrame: {
                // Process RGB frame for tracking and motion detection
                if (viewer_helper->device_type_ == "Aurora912" || viewer_helper->device_type_ == "Aurora930" ||
                    viewer_helper->device_type_ == "Aurora932") {
                    int rgb_height = frame->rows;
                    int rgb_width = frame->cols;
                    cv::Mat yuv_mat(rgb_height * 1.5f, rgb_width, CV_8UC1, frame->data.get());
                    rgb_display = cv::Mat(rgb_height, rgb_width, CV_8UC3);
                    cv::cvtColor(yuv_mat, rgb_display, cv::COLOR_YUV2BGR_NV12);
                } else {
                    if (frame->data && frame->size > 0) {
                        cv::Mat rgb_mat(frame->rows, frame->cols, CV_8UC3, frame->data.get());
                        rgb_mat.copyTo(rgb_display);
                    }
                }
                
                if (!rgb_display.empty() && tracking_system) {
                    // Motion detection
                    std::vector<cv::Rect> motion_rects;
                    if (enhanced_params.enable_motion_detection) {
                        motion_rects = tracking_system->detectMotion(rgb_display);
                    }
                    
                    // Update tracking if active
                    if (tracking_system->isTrackingActive()) {
                        tracking_updated = tracking_system->updateTracking(rgb_display, tracking_bbox);
                    } else if (!motion_rects.empty() && enhanced_params.enable_tracking) {
                        // Initialize tracking with largest motion area
                        auto largest_motion = *std::max_element(motion_rects.begin(), motion_rects.end(),
                            [](const cv::Rect& a, const cv::Rect& b) { return a.area() < b.area(); });
                        
                        cv::Rect2d init_bbox(static_cast<double>(largest_motion.x), 
                                           static_cast<double>(largest_motion.y), 
                                           static_cast<double>(largest_motion.width), 
                                           static_cast<double>(largest_motion.height));
                        tracking_system->initializeTracking(rgb_display, init_bbox);
                        tracking_bbox = init_bbox;
                        tracking_updated = true;
                    }
                    
                    // Draw motion areas
                    for (const auto& rect : motion_rects) {
                        cv::rectangle(rgb_display, rect, cv::Scalar(255, 255, 0), 2);
                    }
                    
                    // Draw tracking box
                    if (tracking_updated || tracking_system->isTrackingActive()) {
                        cv::rectangle(rgb_display, tracking_bbox, cv::Scalar(0, 255, 0), 3);
                        cv::putText(rgb_display, "TRACKING", 
                                   cv::Point(static_cast<int>(tracking_bbox.x), 
                                           static_cast<int>(tracking_bbox.y) - 10),
                                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
                    }
                }
                break;
            }
            
            case kDepthFrame: {
                // Calculate enhanced distance
                cv::Rect2d* bbox_ptr = (tracking_updated || tracking_system->isTrackingActive()) ? &tracking_bbox : nullptr;
                detection = calculateEnhancedDistance(frame, bbox_ptr);
                
                // Apply filtering if enabled
                if (tracking_system && enhanced_params.enable_kalman_filtering && detection.valid_points > 0) {
                    cv::Point2f smoothed_pos = tracking_system->smoothPosition(
                        cv::Point2f(detection.center_x, detection.center_y));
                    detection.center_x = smoothed_pos.x;
                    detection.center_y = smoothed_pos.y;
                    detection.distance_mm = tracking_system->smoothDistance(detection.distance_mm);
                }
                
                // Enhanced console output
                if (detection.valid_points > 0) {
                    float stability = tracking_system ? tracking_system->getDistanceStability() : 0.0f;
                    std::cout << "\r[ENHANCED] " << std::setw(4) << static_cast<int>(detection.distance_mm) 
                              << "mm | Conf: " << std::setprecision(2) << std::fixed << detection.confidence
                              << " | Stab: " << stability
                              << " | Pts: " << std::setw(4) << detection.valid_points 
                              << " | " << (detection.is_tracked ? "TRACKED" : "STATIC")
                              << " | Pos: (" << std::setw(3) << static_cast<int>(detection.center_x) 
                              << "," << std::setw(3) << static_cast<int>(detection.center_y) << ")" << std::flush;
                }
                
                // Display colored depth with enhanced overlay
                int size = frame->rows * frame->cols * 3;
                std::shared_ptr<uint8_t> colored_depth(new uint8_t[size], [](uint8_t* p) { delete[] p; });
                viewer_helper->DealWithDepthFrame(*frame, camera_param, colored_depth.get());
                cv::Mat depth_mat(frame->rows, frame->cols, CV_8UC3, colored_depth.get());
                
                // Enhanced visualization
                if (detection.is_tracked && tracking_updated) {
                    // Draw tracking bbox
                    cv::rectangle(depth_mat, tracking_bbox, cv::Scalar(0, 255, 0), 2);
                } else {
                    // Draw static ROI
                    int width = depth_mat.cols;
                    int height = depth_mat.rows;
                    int roi_width = static_cast<int>(width * enhanced_params.roi_x_ratio);
                    int roi_height = static_cast<int>(height * enhanced_params.roi_y_ratio);
                    int roi_start_x = (width - roi_width) / 2;
                    int roi_start_y = (height - roi_height) / 2;
                    
                    cv::rectangle(depth_mat, 
                                 cv::Point(roi_start_x, roi_start_y),
                                 cv::Point(roi_start_x + roi_width, roi_start_y + roi_height),
                                 cv::Scalar(0, 255, 0), 2);
                }
                
                // Draw detection center with confidence-based size
                if (detection.valid_points > 0) {
                    int circle_radius = 4 + static_cast<int>(detection.confidence * 8);
                    cv::Scalar color = detection.is_tracked ? cv::Scalar(255, 255, 255) : cv::Scalar(0, 0, 255);
                    cv::circle(depth_mat, 
                              cv::Point(static_cast<int>(detection.center_x), 
                                       static_cast<int>(detection.center_y)),
                              circle_radius, color, -1);
                    
                    // Add distance text overlay
                    std::string distance_text = std::to_string(static_cast<int>(detection.distance_mm)) + "mm";
                    if (detection.is_tracked) distance_text += " [T]";
                    
                    cv::putText(depth_mat, distance_text, 
                               cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, 
                               cv::Scalar(255, 255, 255), 2);
                    
                    // Add confidence bar
                    int bar_width = static_cast<int>(200 * detection.confidence);
                    cv::rectangle(depth_mat, cv::Point(10, 50), cv::Point(10 + bar_width, 65), 
                                 cv::Scalar(0, 255, 0), -1);
                    cv::rectangle(depth_mat, cv::Point(10, 50), cv::Point(210, 65), 
                                 cv::Scalar(255, 255, 255), 1);
                }
                
                cv::imshow(title, depth_mat);
                break;
            }
            
            case kIrFrame: {
                if (frame->data && frame->size != 0) {
                    cv::Mat ir_mat(frame->rows, frame->cols, CV_8UC1, frame->data.get());
                    cv::imshow(title, ir_mat);
                }
                break;
            }
        }
        cv::waitKey(1);
    }
    
    // Also show RGB frame with overlays if available
    if (!rgb_display.empty()) {
        // Add distance info to RGB display if available
        if (detection.valid_points > 0) {
            cv::circle(rgb_display, 
                      cv::Point(static_cast<int>(detection.center_x), 
                               static_cast<int>(detection.center_y)),
                      8, cv::Scalar(0, 0, 255), -1);
            
            std::string distance_text = "Distance: " + std::to_string(static_cast<int>(detection.distance_mm)) + "mm";
            cv::rectangle(rgb_display, cv::Point(10, 10), cv::Point(350, 50), cv::Scalar(0, 0, 0), -1);
            cv::putText(rgb_display, distance_text, cv::Point(20, 35), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        }
        
        cv::imshow("RGB_Enhanced", rgb_display);
    }
}

// Main enhanced streaming process
int EnhancedRGBDStreamProcess(std::shared_ptr<Device> device,
                             const std::vector<StreamType>& stream_type,
                             long frame_loop_times,
                             std::string device_name) {
    Stream* stream = nullptr;

    int ret = device->CreateStream(stream, stream_type);
    if (ret != 0) {
        std::cout << "Failed to create stream: " << ret << std::endl;
        return ret;
    }

    ret = stream->Start();
    if (ret != 0) {
        std::cout << "Failed to start stream: " << ret << std::endl;
        device->DestroyStream(stream);
        return ret;
    }

    // Initialize tracking system
    tracking_system = std::make_unique<EnhancedTrackingSystem>();

    StreamFrames frames;
    FrameRateHelper frame_rate_helper;
    long cnt = 0;

    std::cout << "\n=== Enhanced Aurora 900 Distance Detection Started ===" << std::endl;
    std::cout << "Features: " 
              << (enhanced_params.enable_tracking ? "Tracking " : "")
              << (enhanced_params.enable_motion_detection ? "Motion " : "")
              << (enhanced_params.enable_kalman_filtering ? "Filtering " : "") << std::endl;
    std::cout << "Controls: 'q' quit, 'a' toggle FPS, 't' toggle tracking, 'm' toggle motion, 'f' toggle filtering" << std::endl;

    while (is_running) {
        if (frame_loop_times == -1) {
            // Run indefinitely
        } else {
            if (frame_loop_times == 0) break;
            frame_loop_times--;
        }
        
        ret = stream->GetFrames(frames, 2000);
        if (ret != 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        
        frame_rate_helper.RecordTimestamp();
        if (is_print_fps && 0 == cnt++ % 30) {
            std::cout << "\nFPS: " << frame_rate_helper.GetFrameRate() << std::endl;
        }

        // Show frames with enhanced tracking
        ShowFrameWithEnhancedTracking(frames, g_camera_param);
        
        // Handle keyboard input
        char key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 'Q') {
            is_running = false;
        } else if (key == 'a' || key == 'A') {
            is_print_fps = !is_print_fps;
            std::cout << "\nFPS display: " << (is_print_fps ? "ON" : "OFF") << std::endl;
        } else if (key == 't' || key == 'T') {
            enhanced_params.enable_tracking = !enhanced_params.enable_tracking;
            std::cout << "\nTracking: " << (enhanced_params.enable_tracking ? "ON" : "OFF") << std::endl;
        } else if (key == 'm' || key == 'M') {
            enhanced_params.enable_motion_detection = !enhanced_params.enable_motion_detection;
            std::cout << "\nMotion Detection: " << (enhanced_params.enable_motion_detection ? "ON" : "OFF") << std::endl;
        } else if (key == 'f' || key == 'F') {
            enhanced_params.enable_kalman_filtering = !enhanced_params.enable_kalman_filtering;
            std::cout << "\nKalman Filtering: " << (enhanced_params.enable_kalman_filtering ? "ON" : "OFF") << std::endl;
        }
    }

    stream->Stop();
    device->DestroyStream(stream);
    return 0;
}

// Main setup function (enhanced version of your PrepareSimpleRGBD)
int PrepareEnhancedRGBD() {
    is_running = true;
    
    // Register device hotplug callback
    DeviceManager::GetInstance()->RegisterDeviceConnectedCallback();

    // Query the list of connected devices
    std::vector<DeviceInformation> device_list;
    int ret = DeviceManager::GetInstance()->GetDeviceList(device_list);
    CHECK_SDK_RETURN_VALUE(ret);
    
    // Check device count
    CHECK_DEVICE_COUNT(device_list.size());
    {
        printf("devices info:\n");
        for (int index = 0; index < static_cast<int>(device_list.size()); index++) {
            printf("index(%d):\t device_addr:%d usb_port(%s)\n",
                   index,
                   device_list[index].ir_camera.device_addr,
                   device_list[index].ir_camera.port_path.c_str());
        }
    }

    // Create a device
    std::shared_ptr<Device> device = DeviceManager::GetInstance()->CreateDevice(device_list[0]);
    CHECK_DEVICE_VALID(device);

    // Print the sdk version
    auto sdk_version = device->GetSdkVersion();
    std::cout << "SDK version: " << sdk_version << std::endl;

    ret = device->Open();
    CHECK_SDK_RETURN_VALUE(ret);

    // Get device name
    std::string device_name = device->GetDeviceName();
    std::cout << "Device name: " << device_name << std::endl;
    viewer_helper.reset(new ViewerHelper(device_name));

    // Set camera support mode
    FrameMode ir_mode, rgb_mode, depth_mode;
    std::vector<std::tuple<FrameMode, FrameMode, FrameMode>> device_resolution_vec;
    ret = device->GetSupportedFrameMode(device_resolution_vec);
    if (ret != 0) {
        std::cout << "GetSupportedFrameMode error: " << ret << std::endl;
        return ret;
    }
    
    int index = ChooceFrameMode(device_resolution_vec);
    ir_mode = std::get<0>(device_resolution_vec[index]);
    rgb_mode = std::get<1>(device_resolution_vec[index]);
    depth_mode = std::get<2>(device_resolution_vec[index]);
    ret = device->SetMode(ir_mode, rgb_mode, depth_mode);
    CHECK_SDK_RETURN_VALUE(ret);

    // Aurora 900 specific settings
    if (device_name == "Aurora930" || device_name == "Aurora931" || device_name == "Aurora932") {
        std::shared_ptr<Aurora900> device_unique_ptr = std::dynamic_pointer_cast<Aurora900>(device);
        
        // Set IR FPS
        int ir_fps = 15;
        std::cout << "Setting IR FPS to " << ir_fps << std::endl;
        device_unique_ptr->SetIrFps(ir_fps);
        
        // Enable aligned mode for better RGB-Depth alignment
        std::cout << "Enabling aligned mode for enhanced tracking" << std::endl;
        device_unique_ptr->SwitchAlignedMode(true);
        device_unique_ptr->DepthCorrection(true);
        
        // Get camera parameters
        Intrinsic ir_intri, rgb_intri;
        Extrinsic extrinsic;
        device->GetCameraParameters(ir_intri, rgb_intri, extrinsic);
        g_camera_param.cx = ir_intri.principal_point[0];
        g_camera_param.cy = ir_intri.principal_point[1];
        g_camera_param.fx = ir_intri.focal_length[0];
        g_camera_param.fy = ir_intri.focal_length[1];
        
        // Set optimal parameters for tracking
        device_unique_ptr->SetRemoveFilterSize(110); // Noise removal
        device_unique_ptr->FilterOutRangeDepthMap(150, 4000); // Depth range filtering
        
        std::cout << "Enhanced Aurora 900 settings applied" << std::endl;
    }

    // Configure enhanced parameters
    std::cout << "\n=== Enhanced Detection Configuration ===" << std::endl;
    std::cout << "Current settings:" << std::endl;
    std::cout << "  ROI size: " << enhanced_params.roi_x_ratio << " x " << enhanced_params.roi_y_ratio << std::endl;
    std::cout << "  Depth range: " << enhanced_params.min_depth << " - " << enhanced_params.max_depth << " mm" << std::endl;
    std::cout << "  Min valid points: " << enhanced_params.min_valid_points << std::endl;
    std::cout << "  Tracking: " << (enhanced_params.enable_tracking ? "ON" : "OFF") << std::endl;
    std::cout << "  Motion detection: " << (enhanced_params.enable_motion_detection ? "ON" : "OFF") << std::endl;
    std::cout << "  Kalman filtering: " << (enhanced_params.enable_kalman_filtering ? "ON" : "OFF") << std::endl;

    // Option to adjust parameters
    char config_choice;
    std::cout << "\nAdjust parameters? (y/n): ";
    std::cin >> config_choice;
    
    if (config_choice == 'y' || config_choice == 'Y') {
        std::cout << "ROI width ratio (0.1-0.8, current " << enhanced_params.roi_x_ratio << "): ";
        std::cin >> enhanced_params.roi_x_ratio;
        enhanced_params.roi_x_ratio = std::max(0.1f, std::min(0.8f, enhanced_params.roi_x_ratio));
        
        std::cout << "ROI height ratio (0.1-0.8, current " << enhanced_params.roi_y_ratio << "): ";
        std::cin >> enhanced_params.roi_y_ratio;
        enhanced_params.roi_y_ratio = std::max(0.1f, std::min(0.8f, enhanced_params.roi_y_ratio));
        
        std::cout << "Enable tracking? (1/0): ";
        int tracking_enable;
        std::cin >> tracking_enable;
        enhanced_params.enable_tracking = (tracking_enable == 1);
        
        std::cout << "Enable motion detection? (1/0): ";
        int motion_enable;
        std::cin >> motion_enable;
        enhanced_params.enable_motion_detection = (motion_enable == 1);
        
        std::cout << "Enable Kalman filtering? (1/0): ";
        int kalman_enable;
        std::cin >> kalman_enable;
        enhanced_params.enable_kalman_filtering = (kalman_enable == 1);
    }

    // Create stream - prioritize RGBD for best tracking performance
    std::vector<StreamType> stream_types_vector;
    std::thread stream_thread;
    
    // Get supported stream types
    std::vector<StreamType> supported_streams;
    device->GetSupportedStreamType(supported_streams);
    
    std::cout << "\nChoose stream mode for enhanced detection:" << std::endl;
    std::cout << "1: RGBD (Recommended - RGB + Depth combined)" << std::endl;
    std::cout << "2: RGB + Depth (separate streams)" << std::endl;
    std::cout << "3: Depth only (no tracking)" << std::endl;
    std::cout << "Enter choice (1-3): ";
    
    int choice;
    std::cin >> choice;
    
    switch (choice) {
        case 1:
            stream_types_vector.push_back(StreamType::kRgbd);
            std::cout << "Using RGBD stream (optimal for tracking)" << std::endl;
            break;
        case 2:
            stream_types_vector.push_back(StreamType::kRgb);
            stream_types_vector.push_back(StreamType::kDepth);
            std::cout << "Using separate RGB and Depth streams" << std::endl;
            break;
        case 3:
            stream_types_vector.push_back(StreamType::kDepth);
            enhanced_params.enable_tracking = false;
            enhanced_params.enable_motion_detection = false;
            std::cout << "Using Depth stream only (tracking disabled)" << std::endl;
            break;
        default:
            stream_types_vector.push_back(StreamType::kRgbd);
            std::cout << "Default: Using RGBD stream" << std::endl;
            break;
    }
    
    if (stream_types_vector.empty()) {
        std::cout << "No stream type selected!" << std::endl;
        return -1;
    }
    
    long frame_loop_times = -1;
    std::cout << "\nStarting enhanced stream..." << std::endl;
    stream_thread = std::thread(EnhancedRGBDStreamProcess,
                                device,
                                stream_types_vector,
                                frame_loop_times,
                                device_name);

    // Wait for stream thread to complete
    if (stream_thread.joinable()) {
        stream_thread.join();
    }
    
    viewer_helper.reset();
    tracking_system.reset();
    device->Close();
    std::cout << "\nEnhanced Aurora 900 detection completed." << std::endl;
    return 0;
}

// Additional utility functions for parameter tuning
void printUsageInstructions() {
    std::cout << "\n=== Enhanced Aurora 900 Distance Detection ===" << std::endl;
    std::cout << "Real-time Controls:" << std::endl;
    std::cout << "  'q' - Quit application" << std::endl;
    std::cout << "  'a' - Toggle FPS display" << std::endl;
    std::cout << "  't' - Toggle object tracking" << std::endl;
    std::cout << "  'm' - Toggle motion detection" << std::endl;
    std::cout << "  'f' - Toggle Kalman filtering" << std::endl;
    std::cout << "\nFeatures:" << std::endl;
    std::cout << "  • Advanced multi-point depth sampling" << std::endl;
    std::cout << "  • OpenCV object tracking (KCF)" << std::endl;
    std::cout << "  • Motion-based region detection" << std::endl;
    std::cout << "  • Kalman filtering for smooth measurements" << std::endl;
    std::cout << "  • Confidence-based measurement weighting" << std::endl;
    std::cout << "  • Real-time stability analysis" << std::endl;
    std::cout << "=========================================" << std::endl;
}

int main() {
    printUsageInstructions();
    
    try {
        return PrepareEnhancedRGBD();
    } catch (const std::exception& e) {
        std::cout << "Exception: " << e.what() << std::endl;
        return -1;
    }
}