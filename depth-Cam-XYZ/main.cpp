#pragma warning(push)
#pragma warning(disable: 4244)  // Disable conversion warnings for OpenCV calls

#include <iostream>
#include <iomanip>
#include <thread>
#include <vector>
#include <memory>
#include <string>
#include <chrono>
#include <fstream>
#include <sstream>
#include <numeric>      
#include <algorithm>    
#include <cmath>        

// Deptrum SDK headers (exactly as in your samples)
#include "deptrum/device.h"
#include "deptrum/stream.h"
#include "functional/base.h"
#include "functional/frame_rate_helper.h"
#include "sample_helper.h"
#include "viewer_helper.hpp"

#ifdef DEVICE_TYPE_AURORA900
#include "deptrum/aurora900_series.h"
#endif

// OpenCV headers
#include "opencv2/opencv.hpp"
#include "opencv2/dnn.hpp"

using namespace std;
using namespace deptrum;
using namespace deptrum::stream;

// Enhanced YOLO detector class with XYZ coordinate support
class SimpleYOLODetector {
private:
    cv::dnn::Net net;
    std::vector<std::string> classNames;
    float confThreshold = 0.5f;
    float nmsThreshold = 0.4f;
    
public:
    struct Detection {
        std::string className;
        float confidence;
        cv::Rect bbox;
        float depth = -1.0f;
        bool hasDepth = false;
        
        // XYZ coordinate members
        float worldX = 0.0f;         // X coordinate in world space (meters)
        float worldY = 0.0f;         // Y coordinate in world space (meters)  
        float worldZ = 0.0f;         // Z coordinate in world space (meters)
        bool hasWorldPos = false;    // Whether XYZ calculation succeeded
        float distance3D = 0.0f;     // 3D distance from camera origin
        
        // Enhanced depth information
        float depthVariance = 0.0f;  // Variance in depth measurements for reliability
        int depthSampleCount = 0;    // Number of depth samples used
        
        // Utility methods for XYZ data
        cv::Point3f getWorldPosition() const {
            return cv::Point3f(worldX, worldY, worldZ);
        }
        
        bool isValid3D() const {
            return hasWorldPos && worldZ > 0.1f && worldZ < 10.0f;  // Reasonable depth range
        }
        
        std::string getXYZString() const {
            if (!hasWorldPos) return "No XYZ";
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(2) 
                << "(" << worldX << ", " << worldY << ", " << worldZ << ")m";
            return oss.str();
        }
        
        std::string getDistanceString() const {
            if (!hasWorldPos) return "No distance";
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(2) << distance3D << "m";
            return oss.str();
        }
    };

    SimpleYOLODetector(const std::string& modelPath) {
        // Initialize class names
        classNames = {
            "person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
            "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
            "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
            "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
            "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
            "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
            "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake",
            "chair", "sofa", "pottedplant", "bed", "diningtable", "toilet", "tvmonitor",
            "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven",
            "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
            "teddy bear", "hair drier", "toothbrush"
        };
        
        confThreshold = 0.5f;
        nmsThreshold = 0.4f;
        
        try {
            net = cv::dnn::readNetFromONNX(modelPath);
            
            // Try CUDA, fall back to CPU
            if (cv::cuda::getCudaEnabledDeviceCount() > 0) {
                std::cout << "Using CUDA for YOLO" << std::endl;
                net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
                net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
            } else {
                std::cout << "Using CPU for YOLO" << std::endl;
                net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
                net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
            }
            
            std::cout << "YOLO model loaded successfully!" << std::endl;
        } catch (const std::exception& e) {
            throw std::runtime_error("Failed to load YOLO model: " + std::string(e.what()));
        }
    }

    std::vector<Detection> detect(const cv::Mat& image, const cv::Mat& depthMap = cv::Mat()) {
        std::vector<Detection> detections;
        
        if (image.empty()) return detections;
        
        static int detection_count = 0;
        detection_count++;
        
        // Preprocess image
        cv::Mat blob;
        cv::dnn::blobFromImage(image, blob, 1.0/255.0, cv::Size(640, 640), cv::Scalar(0,0,0), true, false);
        net.setInput(blob);
        
        // Run inference
        std::vector<cv::Mat> outputs;
        net.forward(outputs, net.getUnconnectedOutLayersNames());
        
        if (outputs.empty()) {
            std::cout << "No YOLO outputs received!" << std::endl;
            return detections;
        }
        
        // Debug output format only for first few detections
        if (detection_count <= 3) {
            std::cout << "=== YOLO Debug Info ===" << std::endl;
            std::cout << "Input image size: " << image.cols << "x" << image.rows << std::endl;
            std::cout << "Number of output tensors: " << outputs.size() << std::endl;
            for (size_t i = 0; i < outputs.size(); ++i) {
                cv::Mat& output = outputs[i];
                std::cout << "Output " << i << " shape: [";
                for (int j = 0; j < output.dims; ++j) {
                    std::cout << output.size[j];
                    if (j < output.dims - 1) std::cout << ", ";
                }
                std::cout << "]" << std::endl;
            }
        }
        
        // Simple postprocessing
        std::vector<int> classIds;
        std::vector<float> confidences;
        std::vector<cv::Rect> boxes;
        
        float x_factor = static_cast<float>(image.cols) / 640.0f;
        float y_factor = static_cast<float>(image.rows) / 640.0f;
        
        // Process YOLO output
        cv::Mat output = outputs[0];
        
        if (output.dims != 3 || output.size[0] != 1) {
            std::cout << "Unexpected YOLO output format!" << std::endl;
            return detections;
        }
        
        const int num_classes = 80;
        const int num_boxes = output.size[2];
        const int output_size = output.size[1];
        
        // YOLOv8 format: [x_center, y_center, width, height, class0_score, class1_score, ..., class79_score]
        cv::Mat output_transposed;
        cv::transpose(output.reshape(1, output.size[1]), output_transposed);
        
        const float* data = output_transposed.ptr<float>();
        
        for (int i = 0; i < num_boxes; ++i) {
            const float* row = data + i * output_size;
            
            // Extract bbox coordinates (first 4 values)
            float cx = row[0];
            float cy = row[1];
            float w = row[2];
            float h = row[3];
            
            // Find the class with highest score (values 4-83)
            float max_class_score = 0.0f;
            int best_class_id = -1;
            
            for (int j = 0; j < num_classes; ++j) {
                float class_score = row[4 + j];
                if (class_score > max_class_score) {
                    max_class_score = class_score;
                    best_class_id = j;
                }
            }
            
            float confidence = max_class_score;
            
            if (confidence >= confThreshold && best_class_id >= 0) {
                // Convert normalized coordinates to pixel coordinates
                float x1 = (cx - w * 0.5f) * x_factor;
                float y1 = (cy - h * 0.5f) * y_factor;
                float x2 = (cx + w * 0.5f) * x_factor;
                float y2 = (cy + h * 0.5f) * y_factor;
                
                // Clamp to image bounds
                x1 = std::max(0.0f, std::min(x1, static_cast<float>(image.cols - 1)));
                y1 = std::max(0.0f, std::min(y1, static_cast<float>(image.rows - 1)));
                x2 = std::max(x1 + 1.0f, std::min(x2, static_cast<float>(image.cols)));
                y2 = std::max(y1 + 1.0f, std::min(y2, static_cast<float>(image.rows)));
                
                int bbox_x = static_cast<int>(x1);
                int bbox_y = static_cast<int>(y1);
                int bbox_w = static_cast<int>(x2 - x1);
                int bbox_h = static_cast<int>(y2 - y1);
                
                if (bbox_w > 0 && bbox_h > 0) {
                    boxes.push_back(cv::Rect(bbox_x, bbox_y, bbox_w, bbox_h));
                    classIds.push_back(best_class_id);
                    confidences.push_back(confidence);
                }
            }
        }
        
        // Apply NMS
        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
        
        // Limit to reasonable number of detections
        const size_t max_detections = 5;
        if (indices.size() > max_detections) {
            std::vector<std::pair<float, int>> conf_idx_pairs;
            for (size_t i = 0; i < indices.size(); ++i) {
                conf_idx_pairs.push_back({confidences[indices[i]], indices[i]});
            }
            std::sort(conf_idx_pairs.rbegin(), conf_idx_pairs.rend());
            
            indices.clear();
            for (size_t i = 0; i < max_detections; ++i) {
                indices.push_back(conf_idx_pairs[i].second);
            }
        }
        
        // Create final detections
        for (size_t i = 0; i < indices.size(); ++i) {
            int idx = indices[i];
            Detection detection;
            
            detection.bbox = boxes[idx];
            detection.confidence = confidences[idx];
            
            if (classIds[idx] >= 0 && classIds[idx] < static_cast<int>(classNames.size())) {
                detection.className = classNames[classIds[idx]];
            } else {
                detection.className = "unknown";
            }
            
            // Calculate depth if available
            if (!depthMap.empty()) {
                cv::Rect safeRect = detection.bbox & cv::Rect(0, 0, depthMap.cols, depthMap.rows);
                if (safeRect.area() > 0) {
                    cv::Mat roi = depthMap(safeRect);
                    cv::Mat mask = roi > 0;
                    if (cv::countNonZero(mask) > 0) {
                        cv::Scalar meanDepth = cv::mean(roi, mask);
                        detection.depth = static_cast<float>(meanDepth[0]);
                        detection.hasDepth = true;
                    }
                }
            }
            
            detections.push_back(detection);
        }
        
        return detections;
    }

    void drawDetections(cv::Mat& image, const std::vector<Detection>& detections) {
        for (const auto& detection : detections) {
            // Color based on depth availability
            cv::Scalar color = detection.hasDepth ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
            
            // Draw bounding box
            cv::rectangle(image, detection.bbox, color, 2);
            
            // Create label with proper percentage
            std::string label = detection.className + " " + 
                               std::to_string(static_cast<int>(detection.confidence * 100)) + "%";
            if (detection.hasDepth) {
                label += " " + std::to_string(static_cast<int>(detection.depth)) + "mm";
            }
            
            // Draw label
            int baseline;
            cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
            cv::Point labelPos(detection.bbox.x, detection.bbox.y - 10);
            
            // Label background
            cv::rectangle(image, 
                         cv::Point(labelPos.x, labelPos.y - labelSize.height), 
                         cv::Point(labelPos.x + labelSize.width, labelPos.y + baseline),
                         color, cv::FILLED);
            
            // Label text
            cv::putText(image, label, labelPos, cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                       cv::Scalar(255, 255, 255), 1);
        }
    }
};

// Global variables
CameraParam g_camera_parm;
std::shared_ptr<ViewerHelper> viewer_helper;
bool is_running = true;
std::unique_ptr<SimpleYOLODetector> yolo_detector;

// XYZ feature control variables
bool g_enableXYZMode = true;        // Enable XYZ coordinate mode
bool g_showXYZGuide = false;         // Show coordinate system guide
bool g_saveXYZLog = false;          // Flag to save XYZ data

// Function declarations
void calculateXYZForDetections(std::vector<SimpleYOLODetector::Detection>& detections, const cv::Mat& depthFrame);
void drawDetectionsWithXYZ(cv::Mat& image, const std::vector<SimpleYOLODetector::Detection>& detections, const cv::Mat& depthFrame);
void drawXYZAxisIndicator(cv::Mat& image, const cv::Point2f& center);
void drawCoordinateSystemGuide(cv::Mat& image);
void printXYZDetectionInfo(const std::vector<SimpleYOLODetector::Detection>& detections);
void saveXYZCoordinatesToCSV(const std::vector<SimpleYOLODetector::Detection>& detections);

// Enhanced frame callback with XYZ coordinates
void FrameCallbackWithXYZYOLO(const StreamFrames& frames) {
    if (!yolo_detector) return;
    
    cv::Mat rgbFrame, depthFrame;
    static int frame_count = 0;
    frame_count++;
    
    // Extract frames
    for (int i = 0; i < frames.count; i++) {
        auto frame = frames.frame_ptr[i];
        
        if (frame->frame_type == kRgbFrame && frame->cols > 0 && frame->rows > 0) {
            cv::Mat yuvMat(static_cast<int>(frame->rows * 1.5f), frame->cols, CV_8UC1, frame->data.get());
            rgbFrame = cv::Mat(frame->rows, frame->cols, CV_8UC3);
            cv::cvtColor(yuvMat, rgbFrame, cv::COLOR_YUV2BGR_NV12);
        }
        else if (frame->frame_type == kDepthFrame && frame->cols > 0 && frame->rows > 0) {
            depthFrame = cv::Mat(frame->rows, frame->cols, CV_16UC1, frame->data.get());
        }
    }
    
    // Debug info every 30 frames
    if (frame_count % 30 == 0) {
        std::cout << "\n=== Frame " << frame_count << " Debug (XYZ Mode) ===" << std::endl;
        std::cout << "RGB Frame: " << (rgbFrame.empty() ? "EMPTY" : std::to_string(rgbFrame.cols) + "x" + std::to_string(rgbFrame.rows)) << std::endl;
        std::cout << "Depth Frame: " << (depthFrame.empty() ? "EMPTY" : std::to_string(depthFrame.cols) + "x" + std::to_string(depthFrame.rows)) << std::endl;
    }
    
    // Run YOLO detection with XYZ calculation if we have both RGB and depth frames
    if (!rgbFrame.empty()) {
        auto detections = yolo_detector->detect(rgbFrame, depthFrame);
        
        // Calculate XYZ coordinates for each detection
        if (g_enableXYZMode && !depthFrame.empty()) {
            calculateXYZForDetections(detections, depthFrame);
        }
        
        // Draw detections with XYZ info
        if (!detections.empty() && detections.size() < 20) {
            if (g_enableXYZMode) {
                drawDetectionsWithXYZ(rgbFrame, detections, depthFrame);
            } else {
                yolo_detector->drawDetections(rgbFrame, detections);
            }
            
            // Print XYZ detection info (reduced frequency to avoid spam)
            if (frame_count % 15 == 0 && g_enableXYZMode) {
                printXYZDetectionInfo(detections);
            }
            
            // Save XYZ data if requested
            if (g_saveXYZLog && g_enableXYZMode) {
                saveXYZCoordinatesToCSV(detections);
                g_saveXYZLog = false;
            }
        }
        
        // Draw coordinate system guide
        if (g_showXYZGuide && g_enableXYZMode) {
            drawCoordinateSystemGuide(rgbFrame);
        }
        
        // Display RGB with XYZ annotations
        cv::imshow("YOLO + Depth Camera + XYZ Coordinates", rgbFrame);
    }
    
    // Show depth visualization using existing viewer
    viewer_helper->ShowFrame(frames, g_camera_parm);
}

// Function to calculate XYZ coordinates for detections
void calculateXYZForDetections(std::vector<SimpleYOLODetector::Detection>& detections, const cv::Mat& depthFrame) {
    for (auto& detection : detections) {
        if (!detection.hasDepth || depthFrame.empty()) continue;
        
        // Calculate center point of detection
        cv::Point2f center(static_cast<float>(detection.bbox.x + detection.bbox.width / 2),
                          static_cast<float>(detection.bbox.y + detection.bbox.height / 2));
        
        // Sample depth values in the bounding box for robustness
        std::vector<float> depthSamples;
        cv::Rect safeRect = detection.bbox & cv::Rect(0, 0, depthFrame.cols, depthFrame.rows);
        
        if (safeRect.area() > 0) {
            // Sample multiple points in the bounding box
            int stepX = std::max(1, safeRect.width / 3);
            int stepY = std::max(1, safeRect.height / 3);
            
            for (int y = safeRect.y; y < safeRect.y + safeRect.height; y += stepY) {
                for (int x = safeRect.x; x < safeRect.x + safeRect.width; x += stepX) {
                    uint16_t depthValue = depthFrame.at<uint16_t>(y, x);
                    if (depthValue > 0 && depthValue < 8000) { // Valid depth range
                        depthSamples.push_back(static_cast<float>(depthValue));
                    }
                }
            }
            
            if (!depthSamples.empty()) {
                // Use median depth for robustness
                std::sort(depthSamples.begin(), depthSamples.end());
                float medianDepth = depthSamples[depthSamples.size() / 2];
                
                // Convert depth from mm to meters
                float depthMeters = medianDepth / 1000.0f;
                
                // Calculate XYZ coordinates using camera intrinsics
                detection.worldX = (center.x - g_camera_parm.cx) * depthMeters / g_camera_parm.fx;
                detection.worldY = (center.y - g_camera_parm.cy) * depthMeters / g_camera_parm.fy;
                detection.worldZ = depthMeters;
                detection.hasWorldPos = true;
                
                // Calculate 3D distance from camera
                detection.distance3D = std::sqrt(detection.worldX * detection.worldX + 
                                               detection.worldY * detection.worldY + 
                                               detection.worldZ * detection.worldZ);
                
                // Calculate depth variance for quality assessment
                if (depthSamples.size() > 1) {
                    float mean = std::accumulate(depthSamples.begin(), depthSamples.end(), 0.0f) / 
                                static_cast<float>(depthSamples.size());
                    float variance = 0.0f;
                    for (float depth : depthSamples) {
                        variance += (depth - mean) * (depth - mean);
                    }
                    detection.depthVariance = variance / static_cast<float>(depthSamples.size() - 1);
                }
                detection.depthSampleCount = static_cast<int>(depthSamples.size());
            }
        }
    }
}

// Enhanced detection drawing with XYZ coordinates
void drawDetectionsWithXYZ(cv::Mat& image, const std::vector<SimpleYOLODetector::Detection>& detections, const cv::Mat& depthFrame) {
    for (const auto& detection : detections) {
        // Color based on whether we have 3D position
        cv::Scalar color = detection.hasWorldPos ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
        
        // Draw bounding box
        cv::rectangle(image, detection.bbox, color, 2);
        
        // Create enhanced label with XYZ info
        std::ostringstream labelStream;
        labelStream << detection.className << " " 
                   << std::to_string(static_cast<int>(detection.confidence * 100)) << "%";
        
        if (detection.hasWorldPos) {
            labelStream << std::fixed << std::setprecision(2)
                       << "\nXYZ: (" << detection.worldX << ", " 
                       << detection.worldY << ", " << detection.worldZ << ")m"
                       << "\nDist: " << detection.distance3D << "m";
        } else if (detection.hasDepth) {
            labelStream << "\nDepth: " << static_cast<int>(detection.depth) << "mm";
        }
        
        std::string label = labelStream.str();
        
        // Draw multi-line label
        std::istringstream iss(label);
        std::string line;
        int lineNum = 0;
        
        while (std::getline(iss, line)) {
            int baseline;
            cv::Size labelSize = cv::getTextSize(line, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseline);
            
            cv::Point labelPos(detection.bbox.x, detection.bbox.y - 10 - (lineNum * 15));
            if (labelPos.y < 0) labelPos.y = detection.bbox.y + detection.bbox.height + 15 + (lineNum * 15);
            
            // Label background
            cv::rectangle(image, 
                         cv::Point(labelPos.x, labelPos.y - labelSize.height), 
                         cv::Point(labelPos.x + labelSize.width, labelPos.y + baseline),
                         cv::Scalar(0, 0, 0), cv::FILLED);
            
            // Label text
            cv::putText(image, line, labelPos, cv::FONT_HERSHEY_SIMPLEX, 0.4, 
                       cv::Scalar(255, 255, 255), 1);
            lineNum++;
        }
        
        // Draw XYZ axis indicator at object center if we have 3D position
        if (detection.hasWorldPos) {
            cv::Point2f center(static_cast<float>(detection.bbox.x + detection.bbox.width / 2),
                              static_cast<float>(detection.bbox.y + detection.bbox.height / 2));
            drawXYZAxisIndicator(image, center);
        }
    }
}


// Draw XYZ axis indicator at object center
void drawXYZAxisIndicator(cv::Mat& image, const cv::Point2f& center) {
    int axisLength = 20;
    
    // X-axis (Red) - horizontal right
    cv::Point2f xEnd(center.x + static_cast<float>(axisLength), center.y);
    cv::arrowedLine(image, 
                   cv::Point(static_cast<int>(center.x), static_cast<int>(center.y)), 
                   cv::Point(static_cast<int>(xEnd.x), static_cast<int>(xEnd.y)),
                   cv::Scalar(0, 0, 255), 2, 8, 0, 0.3);
    cv::putText(image, "X", 
               cv::Point(static_cast<int>(xEnd.x + 3), static_cast<int>(xEnd.y + 5)),
               cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 255), 1);
    
    // Y-axis (Green) - vertical down
    cv::Point2f yEnd(center.x, center.y + static_cast<float>(axisLength));
    cv::arrowedLine(image,
                   cv::Point(static_cast<int>(center.x), static_cast<int>(center.y)),
                   cv::Point(static_cast<int>(yEnd.x), static_cast<int>(yEnd.y)),
                   cv::Scalar(0, 255, 0), 2, 8, 0, 0.3);
    cv::putText(image, "Y", 
               cv::Point(static_cast<int>(yEnd.x + 3), static_cast<int>(yEnd.y + 12)),
               cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 255, 0), 1);
    
    // Z-axis (Blue) - diagonal up-left (representing depth away from camera)
    cv::Point2f zEnd(center.x - static_cast<float>(axisLength) * 0.7f, 
                     center.y - static_cast<float>(axisLength) * 0.7f);
    cv::arrowedLine(image,
                   cv::Point(static_cast<int>(center.x), static_cast<int>(center.y)),
                   cv::Point(static_cast<int>(zEnd.x), static_cast<int>(zEnd.y)),
                   cv::Scalar(255, 0, 0), 2, 8, 0, 0.3);
    cv::putText(image, "Z", 
               cv::Point(static_cast<int>(zEnd.x - 10), static_cast<int>(zEnd.y - 5)),
               cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 0, 0), 1);
    
    // Center point
    cv::circle(image, cv::Point(static_cast<int>(center.x), static_cast<int>(center.y)), 
              2, cv::Scalar(255, 255, 255), -1);
}

// Draw coordinate system guide
void drawCoordinateSystemGuide(cv::Mat& image) {
    int guideX = 20;
    int guideY = 60;
    int axisLength = 30;
    
    // Background rectangle
    cv::rectangle(image, 
                  cv::Point(guideX - 10, guideY - 45),
                  cv::Point(guideX + 130, guideY + 45),
                  cv::Scalar(0, 0, 0), -1);
    cv::rectangle(image, 
                  cv::Point(guideX - 10, guideY - 45),
                  cv::Point(guideX + 130, guideY + 45),
                  cv::Scalar(255, 255, 255), 1);
    
    // Title
    cv::putText(image, "3D Coordinate System", cv::Point(guideX, guideY - 30),
               cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
    
    cv::Point origin(guideX, guideY);
    
    // X-axis (Red)
    cv::Point xEnd(guideX + axisLength, guideY);
    cv::arrowedLine(image, origin, xEnd, cv::Scalar(0, 0, 255), 2, 8, 0, 0.3);
    cv::putText(image, "X+ Right", cv::Point(guideX + 35, guideY + 5),
               cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 255), 1);
    
    // Y-axis (Green)
    cv::Point yEnd(guideX, guideY + axisLength);
    cv::arrowedLine(image, origin, yEnd, cv::Scalar(0, 255, 0), 2, 8, 0, 0.3);
    cv::putText(image, "Y+ Down", cv::Point(guideX + 35, guideY + 15),
               cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 255, 0), 1);
    
    // Z-axis (Blue)
    cv::Point zEnd(guideX - static_cast<int>(static_cast<float>(axisLength) * 0.7f), 
                   guideY - static_cast<int>(static_cast<float>(axisLength) * 0.7f));
    cv::arrowedLine(image, origin, zEnd, cv::Scalar(255, 0, 0), 2, 8, 0, 0.3);
    cv::putText(image, "Z+ Forward", cv::Point(guideX + 35, guideY + 25),
               cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 0, 0), 1);
    
    // Origin point
    cv::circle(image, origin, 2, cv::Scalar(255, 255, 255), -1);
}

// Print XYZ detection information
void printXYZDetectionInfo(const std::vector<SimpleYOLODetector::Detection>& detections) {
    std::cout << "\n=== XYZ Detection Info ===" << std::endl;
    
    for (size_t i = 0; i < detections.size(); ++i) {
        const auto& detection = detections[i];
        
        std::cout << "[" << (i+1) << "] " << detection.className 
                  << " (conf: " << std::fixed << std::setprecision(1) 
                  << (detection.confidence * 100) << "%)";
        
        if (detection.hasWorldPos) {
            std::cout << "\n    World Position: X=" << std::setprecision(3) << detection.worldX
                      << "m, Y=" << detection.worldY << "m, Z=" << detection.worldZ << "m";
            std::cout << "\n    Distance from camera: " << std::setprecision(2) 
                      << detection.distance3D << "m";
            
            if (detection.depthSampleCount > 1) {
                std::cout << "\n    Depth quality: " << detection.depthSampleCount 
                          << " samples, variance=" << std::setprecision(1) << detection.depthVariance;
            }
        } else if (detection.hasDepth) {
            std::cout << "\n    Depth: " << static_cast<int>(detection.depth) << "mm (no XYZ calc)";
        } else {
            std::cout << "\n    No depth information available";
        }
        
        std::cout << std::endl;
    }
}

// Save XYZ coordinates to CSV file
void saveXYZCoordinatesToCSV(const std::vector<SimpleYOLODetector::Detection>& detections) {
    // Generate filename with timestamp
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    
    std::ostringstream filename;
    filename << "xyz_detections_" << time_t << ".csv";
    
    std::ofstream file(filename.str());
    if (!file.is_open()) {
        std::cerr << "Failed to create CSV file: " << filename.str() << std::endl;
        return;
    }
    
    // Write CSV header
    file << "Timestamp,Class,Confidence,X_meters,Y_meters,Z_meters,Distance_meters,ImageX,ImageY,BboxWidth,BboxHeight,DepthVariance,SampleCount\n";
    
    // Get current timestamp
    auto currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    
    // Write detection data
    for (const auto& detection : detections) {
        file << currentTime << ","
             << detection.className << ","
             << std::fixed << std::setprecision(3) << detection.confidence;
        
        if (detection.hasWorldPos) {
            file << "," << detection.worldX
                 << "," << detection.worldY
                 << "," << detection.worldZ
                 << "," << detection.distance3D;
        } else {
            file << ",,,,";  // Empty XYZ fields
        }
        
        file << "," << (detection.bbox.x + detection.bbox.width/2)
             << "," << (detection.bbox.y + detection.bbox.height/2)
             << "," << detection.bbox.width
             << "," << detection.bbox.height
             << "," << detection.depthVariance
             << "," << detection.depthSampleCount << "\n";
    }
    
    file.close();
    std::cout << "XYZ coordinates saved to: " << filename.str() << std::endl;
}

// Main function with enhanced XYZ features
int main() {
    std::cout << "=== Enhanced YOLO + Depth Camera + XYZ Coordinates Integration ===" << std::endl;
    std::cout << "Features:" << std::endl;
    std::cout << "  - Real-time object detection with YOLO" << std::endl;
    std::cout << "  - 3D position calculation (XYZ coordinates)" << std::endl;
    std::cout << "  - Distance measurement from camera" << std::endl;
    std::cout << "  - Coordinate system visualization" << std::endl;
    std::cout << "  - CSV export of XYZ data" << std::endl;
    std::cout << "  - Robust depth sampling and filtering" << std::endl;
    std::cout << std::endl;
    
    // Initialize YOLO detector
    try {
        yolo_detector = std::make_unique<SimpleYOLODetector>("yolov8n.onnx");
    } catch (const std::exception& e) {
        std::cerr << "Failed to initialize YOLO: " << e.what() << std::endl;
        std::cerr << "Make sure yolov8n.onnx is in the current directory" << std::endl;
        return -1;
    }
    
    is_running = true;
    
    // Register device hotplug callback
    DeviceManager::GetInstance()->RegisterDeviceConnectedCallback();

    // Query connected devices
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

    // Create device
    std::shared_ptr<Device> device = DeviceManager::GetInstance()->CreateDevice(device_list[0]);
    CHECK_DEVICE_VALID(device);

    // Print SDK version
    auto sdk_version = device->GetSdkVersion();
    std::cout << "SDK version: " << sdk_version << std::endl;

    ret = device->Open();
    CHECK_SDK_RETURN_VALUE(ret);

    // Get device name and setup viewer
    std::string device_name = device->GetDeviceName();
    std::cout << "Device name: " << device_name << std::endl;
    viewer_helper = std::make_shared<ViewerHelper>(device_name);

    // Set camera support mode with interactive selection
    FrameMode ir_mode, rgb_mode, depth_mode;
    std::vector<std::tuple<FrameMode, FrameMode, FrameMode>> device_resolution_vec;
    ret = device->GetSupportedFrameMode(device_resolution_vec);
    if (ret != 0) {
        std::cout << "GetSupportedFrameMode error: " << ret << std::endl;
        return -1;
    }

    if (device_resolution_vec.empty()) {
        std::cout << "No supported frame modes found!" << std::endl;
        return -1;
    }

    // Use the existing ChooceFrameMode function for interactive selection
    std::cout << "\nPlease select a frame mode for higher resolution (try index 1 or 2 for 640x400):" << std::endl;
    int selected_index = ChooceFrameMode(device_resolution_vec);
    
    ir_mode = std::get<0>(device_resolution_vec[selected_index]);
    rgb_mode = std::get<1>(device_resolution_vec[selected_index]);
    depth_mode = std::get<2>(device_resolution_vec[selected_index]);
    
    std::cout << "Selected mode index " << selected_index << std::endl;
    ret = device->SetMode(ir_mode, rgb_mode, depth_mode);
    CHECK_SDK_RETURN_VALUE(ret);

    // Device-specific setup
    if (device_name == "Aurora930" || device_name == "Aurora931" || device_name == "Aurora932") {
#ifdef DEVICE_TYPE_AURORA900
        std::shared_ptr<Aurora900> device_unique_ptr = std::dynamic_pointer_cast<Aurora900>(device);
        if (device_unique_ptr) {
            device_unique_ptr->SwitchAlignedMode(true);
        }
#endif
    }

    // Get camera parameters
    Intrinsic ir_intri, rgb_intri;
    Extrinsic extrinsic;
    device->GetCameraParameters(ir_intri, rgb_intri, extrinsic);
    
    g_camera_parm.cx = ir_intri.principal_point[0];
    g_camera_parm.cy = ir_intri.principal_point[1];
    g_camera_parm.fx = ir_intri.focal_length[0];
    g_camera_parm.fy = ir_intri.focal_length[1];
    
    std::cout << "Camera intrinsics for XYZ calculation:" << std::endl;
    std::cout << "  fx: " << g_camera_parm.fx << ", fy: " << g_camera_parm.fy << std::endl;
    std::cout << "  cx: " << g_camera_parm.cx << ", cy: " << g_camera_parm.cy << std::endl;

    // Create stream
    Stream* stream = nullptr;
    std::vector<StreamType> stream_types = {StreamType::kRgbd};
    ret = device->CreateStream(stream, stream_types);
    CHECK_SDK_RETURN_VALUE(ret);
    
    ret = stream->Start();
    CHECK_SDK_RETURN_VALUE(ret);

    std::cout << "\nStarting YOLO + Depth + XYZ detection..." << std::endl;
    std::cout << "Enhanced Controls:" << std::endl;
    std::cout << "  'q' or ESC - Quit" << std::endl;
    std::cout << "  'x' - Toggle XYZ coordinate mode (currently: " << (g_enableXYZMode ? "ON" : "OFF") << ")" << std::endl;
    std::cout << "  'g' - Toggle coordinate system guide (currently: " << (g_showXYZGuide ? "ON" : "OFF") << ")" << std::endl;
    std::cout << "  's' - Save current XYZ data to CSV" << std::endl;
    std::cout << std::endl;

    // Main processing loop with enhanced controls
    while (is_running) {
        StreamFrames frames;
        ret = stream->GetFrames(frames, 2000);
        
        if (ret == 0 && frames.count > 0) {
            FrameCallbackWithXYZYOLO(frames);  // Use enhanced callback
        }
        
        // Enhanced keyboard controls
        char key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 27) {
            is_running = false;
            break;
        }
        else if (key == 'x') {
            g_enableXYZMode = !g_enableXYZMode;
            std::cout << "XYZ coordinate mode: " << (g_enableXYZMode ? "ON" : "OFF") << std::endl;
        }
        else if (key == 'g') {
            g_showXYZGuide = !g_showXYZGuide;
            std::cout << "Coordinate guide: " << (g_showXYZGuide ? "ON" : "OFF") << std::endl;
        }
        else if (key == 's') {
            g_saveXYZLog = true;
            std::cout << "XYZ data will be saved on next detection..." << std::endl;
        }
    }

    // Cleanup
    std::cout << "Stopping stream..." << std::endl;
    stream->Stop();
    device->DestroyStream(stream);
    cv::destroyAllWindows();
    
    std::cout << "Enhanced program finished successfully!" << std::endl;
    std::cout << "XYZ coordinate extraction completed." << std::endl;
    return 0;
}

#pragma warning(pop)