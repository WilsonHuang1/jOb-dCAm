// Minimal YOLO + Depth integration - based exactly on your working samples
// This version follows the exact pattern from your sample_viewer.cc

#include <iostream>
#include <iomanip>
#include <thread>
#include <vector>
#include <memory>
#include <string>
#include <chrono>

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

// Simple YOLO detector class (minimal version to avoid compilation issues)
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
    };

    SimpleYOLODetector(const std::string& modelPath) {
        // Initialize class names (just a few key ones to keep it simple)
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
        
        // Set more reasonable thresholds
        confThreshold = 0.5f;  // 50% confidence minimum
        nmsThreshold = 0.4f;   // NMS threshold
        
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
        
        // Process YOLO output - COMPLETELY FIXED VERSION
        cv::Mat output = outputs[0];
        
        // Debug output format
        std::cout << "YOLO output shape: [" << output.size[0] << ", " << output.size[1] << ", " << output.size[2] << "]" << std::endl;
        
        // For YOLOv8, output shape is typically [1, 84, 8400] where:
        // - 84 = 4 (bbox) + 80 (classes), no separate objectness score
        // - 8400 = number of anchor boxes
        
        if (output.dims != 3 || output.size[0] != 1) {
            std::cout << "Unexpected YOLO output format!" << std::endl;
            return detections;
        }
        
        const int num_classes = 80;
        const int num_boxes = output.size[2];  // Usually 8400
        const int output_size = output.size[1]; // Usually 84
        
        if (output_size != 84) {
            std::cout << "Warning: Expected 84 outputs but got " << output_size << std::endl;
        }
        
        // YOLOv8 format: [x_center, y_center, width, height, class0_score, class1_score, ..., class79_score]
        cv::Mat output_transposed;
        cv::transpose(output.reshape(1, output.size[1]), output_transposed); // Shape: [8400, 84]
        
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
            
            // The confidence IS the class score (no separate objectness in YOLOv8)
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
                
                // Only add valid boxes
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
        
        // Limit to reasonable number of detections to avoid spam
        const size_t max_detections = 5;
        if (indices.size() > max_detections) {
            // Sort by confidence and keep only top detections
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
        
        // Debug first few detection runs
        if (detection_count <= 3) {
            std::cout << "Before NMS: " << boxes.size() << " boxes" << std::endl;
            std::cout << "After NMS: " << indices.size() << " detections" << std::endl;
            if (!confidences.empty()) {
                float min_conf = *std::min_element(confidences.begin(), confidences.end());
                float max_conf = *std::max_element(confidences.begin(), confidences.end());
                std::cout << "Confidence range: " << min_conf << " to " << max_conf << std::endl;
            }
        }
        
        // Create final detections
        for (size_t i = 0; i < indices.size(); ++i) {
            int idx = indices[i];
            Detection detection;
            
            detection.bbox = boxes[idx];
            detection.confidence = confidences[idx];
            
            // Safe class name access
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

// Global variables (exactly as in your existing samples)
CameraParam g_camera_parm;
std::shared_ptr<ViewerHelper> viewer_helper;
bool is_running = true;
std::unique_ptr<SimpleYOLODetector> yolo_detector;

// Enhanced frame callback with YOLO detection and better debugging
void FrameCallbackWithYOLO(const StreamFrames& frames) {
    if (!yolo_detector) return;
    
    cv::Mat rgbFrame, depthFrame;
    static int frame_count = 0;
    frame_count++;
    
    // Extract frames (same pattern as your existing code)
    for (int i = 0; i < frames.count; i++) {
        auto frame = frames.frame_ptr[i];
        
        if (frame->frame_type == kRgbFrame && frame->cols > 0 && frame->rows > 0) {
            // Convert YUV to BGR (exactly as in your samples)
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
        std::cout << "\n=== Frame " << frame_count << " Debug ===" << std::endl;
        std::cout << "RGB Frame: " << (rgbFrame.empty() ? "EMPTY" : std::to_string(rgbFrame.cols) + "x" + std::to_string(rgbFrame.rows)) << std::endl;
        std::cout << "Depth Frame: " << (depthFrame.empty() ? "EMPTY" : std::to_string(depthFrame.cols) + "x" + std::to_string(depthFrame.rows)) << std::endl;
    }
    
    // Run YOLO detection if we have RGB frame
    if (!rgbFrame.empty()) {
        auto detections = yolo_detector->detect(rgbFrame, depthFrame);
        
        // Only process if we have reasonable detections
        if (!detections.empty() && detections.size() < 20) { // Limit to reasonable number
            // Draw detections
            yolo_detector->drawDetections(rgbFrame, detections);
            
            // Print detection info with better formatting (only every few frames to reduce spam)
            if (frame_count % 10 == 0) {
                std::cout << "\n=== Frame " << frame_count << " Detections ===" << std::endl;
                for (const auto& detection : detections) {
                    std::cout << "Detected: " << detection.className 
                             << " (conf: " << std::fixed << std::setprecision(1) << (detection.confidence * 100) << "%)";
                    if (detection.hasDepth) {
                        std::cout << " at " << static_cast<int>(detection.depth) << "mm";
                    } else {
                        std::cout << " [no depth]";
                    }
                    std::cout << " bbox: [" << detection.bbox.x << "," << detection.bbox.y 
                             << "," << detection.bbox.width << "x" << detection.bbox.height << "]" << std::endl;
                }
            }
        }
        
        // Display RGB with detections
        cv::imshow("YOLO + Depth Camera", rgbFrame);
    }
    
    // Also show depth using your existing viewer
    viewer_helper->ShowFrame(frames, g_camera_parm);
}

// Main function - based exactly on your sample_viewer.cc PrepareDevice()
int main() {
    std::cout << "YOLO + Depth Camera Integration" << std::endl;
    
    // Initialize YOLO detector
    try {
        yolo_detector = std::make_unique<SimpleYOLODetector>("yolov8n.onnx");
    } catch (const std::exception& e) {
        std::cerr << "Failed to initialize YOLO: " << e.what() << std::endl;
        std::cerr << "Make sure yolov8n.onnx is in the current directory" << std::endl;
        return -1;
    }
    
    is_running = true;
    
    // Register device hotplug callback (exactly as in your samples)
    DeviceManager::GetInstance()->RegisterDeviceConnectedCallback();

    // Query connected devices (exactly as in your samples)
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

    // Create device (exactly as in your samples)
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

    // Set camera support mode with interactive selection for higher resolution
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

    // Use the existing ChooceFrameMode function from your samples for interactive selection
    std::cout << "\nPlease select a frame mode for higher resolution (try index 1 or 2 for 640x400):" << std::endl;
    int selected_index = ChooceFrameMode(device_resolution_vec);
    
    ir_mode = std::get<0>(device_resolution_vec[selected_index]);
    rgb_mode = std::get<1>(device_resolution_vec[selected_index]);
    depth_mode = std::get<2>(device_resolution_vec[selected_index]);
    
    std::cout << "Selected mode index " << selected_index << std::endl;
    ret = device->SetMode(ir_mode, rgb_mode, depth_mode);
    CHECK_SDK_RETURN_VALUE(ret);

    // Device-specific setup (exactly as in your samples)
    if (device_name == "Aurora930" || device_name == "Aurora931" || device_name == "Aurora932") {
#ifdef DEVICE_TYPE_AURORA900
        std::shared_ptr<Aurora900> device_unique_ptr = std::dynamic_pointer_cast<Aurora900>(device);
        if (device_unique_ptr) {
            device_unique_ptr->SwitchAlignedMode(true);
            // Note: Some SDK versions may not have SwitchDepthCorrect
            // device_unique_ptr->SwitchDepthCorrect(true);
        }
#endif
    }

    // Get camera parameters (exactly as in your samples)
    Intrinsic ir_intri, rgb_intri;
    Extrinsic extrinsic;
    device->GetCameraParameters(ir_intri, rgb_intri, extrinsic);
    
    g_camera_parm.cx = ir_intri.principal_point[0];
    g_camera_parm.cy = ir_intri.principal_point[1];
    g_camera_parm.fx = ir_intri.focal_length[0];
    g_camera_parm.fy = ir_intri.focal_length[1];

    // Create stream (corrected for proper API usage)
    Stream* stream = nullptr;
    std::vector<StreamType> stream_types = {StreamType::kRgbd}; // RGBD stream for both RGB and depth
    ret = device->CreateStream(stream, stream_types);
    CHECK_SDK_RETURN_VALUE(ret);
    
    ret = stream->Start();
    CHECK_SDK_RETURN_VALUE(ret);

    std::cout << "Starting YOLO + Depth detection..." << std::endl;
    std::cout << "Press 'q' to quit" << std::endl;

    // Main processing loop
    while (is_running) {
        StreamFrames frames;
        ret = stream->GetFrames(frames, 2000); // 2 second timeout
        
        if (ret == 0 && frames.count > 0) {
            FrameCallbackWithYOLO(frames);
        }
        
        // Check for quit key
        char key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 27) { // 'q' or ESC key
            is_running = false;
            break;
        }
    }

    // Cleanup (exactly as in your samples)
    std::cout << "Stopping stream..." << std::endl;
    stream->Stop();
    device->DestroyStream(stream);
    cv::destroyAllWindows();
    
    std::cout << "Program finished successfully!" << std::endl;
    return 0;
}