#pragma warning(push)
#pragma warning(disable: 4244)

#define NOMINMAX
#define WIN32_LEAN_AND_MEAN

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
#include <mutex>

// Deptrum SDK headers
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

// Open3D headers - include after Windows headers to avoid conflicts
#include <Open3D/Open3D.h>

using namespace std;
using namespace deptrum;
using namespace deptrum::stream;

// Improved YOLO detector based on your depth-XYZ implementation
class ImprovedYOLODetector {
private:
    cv::dnn::Net net;
    std::vector<std::string> classNames;
    float confThreshold = 0.7f;  // Higher threshold for better quality
    float nmsThreshold = 0.4f;
    bool isInitialized = false;

public:
    struct Detection {
        std::string className;
        float confidence;
        cv::Rect bbox;
        float depth = -1.0f;
        bool hasDepth = false;

        // XYZ coordinate members
        float worldX = 0.0f;
        float worldY = 0.0f;
        float worldZ = 0.0f;
        bool hasWorldPos = false;
        float distance3D = -1.0f;
        float depthVariance = 0.0f;
        int depthSampleCount = 0;

        Detection() : confidence(0.0f), hasDepth(false), hasWorldPos(false),
            distance3D(-1.0f), depthVariance(0.0f), depthSampleCount(0) {}
    };

    ImprovedYOLODetector(const std::string& modelPath) {
        initializeClassNames();

        try {
            if (!modelPath.empty()) {
                net = cv::dnn::readNetFromONNX(modelPath);
                if (net.empty()) {
                    throw std::runtime_error("Failed to load ONNX model");
                }

                // Try GPU first, fallback to CPU
                if (cv::cuda::getCudaEnabledDeviceCount() > 0) {
                    net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
                    net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
                    std::cout << "YOLO using GPU acceleration" << std::endl;
                }
                else {
                    net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
                    net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
                    std::cout << "YOLO using CPU" << std::endl;
                }
                isInitialized = true;
            }
        }
        catch (const std::exception& e) {
            std::cerr << "YOLO initialization failed: " << e.what() << std::endl;
            isInitialized = false;
        }
    }

    bool isReady() const { return isInitialized; }

    std::vector<Detection> detect(const cv::Mat& image, const cv::Mat& depthFrame = cv::Mat()) {
        std::vector<Detection> detections;

        if (!isInitialized || image.empty()) {
            return detections;
        }

        // Prepare input blob
        cv::Mat blob;
        cv::dnn::blobFromImage(image, blob, 1.0 / 255.0, cv::Size(640, 640), cv::Scalar(0, 0, 0), true, false);

        // Run inference
        net.setInput(blob);
        std::vector<cv::Mat> outputs;
        net.forward(outputs, net.getUnconnectedOutLayersNames());

        // Parse outputs - improved parsing from your depth-XYZ
        float x_factor = static_cast<float>(image.cols) / 640.0f;
        float y_factor = static_cast<float>(image.rows) / 640.0f;

        std::vector<int> class_ids;
        std::vector<float> confidences;
        std::vector<cv::Rect> boxes;

        for (auto& output : outputs) {
            auto output_shape = output.size;
            int num_detections = output_shape[1];
            int num_classes = output_shape[2] - 5;

            for (int i = 0; i < num_detections; i++) {
                float* data = (float*)output.data + i * (num_classes + 5);
                float confidence = data[4];

                if (confidence >= confThreshold) {
                    float* classes_scores = data + 5;

                    // Find max class score
                    double max_class_score = 0;
                    int max_class_id = 0;
                    for (int j = 0; j < num_classes; j++) {
                        if (classes_scores[j] > max_class_score) {
                            max_class_score = classes_scores[j];
                            max_class_id = j;
                        }
                    }

                    if (max_class_score > confThreshold) {
                        float x = data[0];
                        float y = data[1];
                        float w = data[2];
                        float h = data[3];

                        int left = static_cast<int>((x - 0.5 * w) * x_factor);
                        int top = static_cast<int>((y - 0.5 * h) * y_factor);
                        int width = static_cast<int>(w * x_factor);
                        int height = static_cast<int>(h * y_factor);

                        // Ensure bbox is within image bounds
                        left = std::max(0, std::min(left, image.cols - 1));
                        top = std::max(0, std::min(top, image.rows - 1));
                        width = std::min(width, image.cols - left);
                        height = std::min(height, image.rows - top);

                        if (width > 0 && height > 0) {
                            boxes.push_back(cv::Rect(left, top, width, height));
                            confidences.push_back(confidence);
                            class_ids.push_back(max_class_id);
                        }
                    }
                }
            }
        }

        // Apply NMS
        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);

        // Create final detections with improved depth calculation
        for (int idx : indices) {
            Detection detection;
            detection.className = class_ids[idx] < classNames.size() ?
                classNames[class_ids[idx]] : "unknown";
            detection.confidence = confidences[idx];
            detection.bbox = boxes[idx];

            // Add depth information using improved method from depth-XYZ
            if (!depthFrame.empty()) {
                calculateRobustDepth(detection, depthFrame);
            }

            detections.push_back(detection);
        }

        return detections;
    }

    void drawDetections(cv::Mat& image, const std::vector<Detection>& detections) {
        for (const auto& detection : detections) {
            // Color based on detection quality
            cv::Scalar color = detection.hasWorldPos ?
                cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255); // Green for good, Orange for ok

            cv::rectangle(image, detection.bbox, color, 2);

            // Create label with confidence and XYZ if available
            std::ostringstream label;
            label << detection.className << " "
                << std::fixed << std::setprecision(1) << (detection.confidence * 100) << "%";

            if (detection.hasWorldPos) {
                label << std::setprecision(2) << " D:" << detection.distance3D << "m";
            }

            // Draw label background
            cv::Size textSize = cv::getTextSize(label.str(), cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, nullptr);
            cv::Point labelPos(detection.bbox.x, detection.bbox.y - 5);
            cv::rectangle(image,
                cv::Point(labelPos.x, labelPos.y - textSize.height - 5),
                cv::Point(labelPos.x + textSize.width, labelPos.y),
                color, -1);

            cv::putText(image, label.str(), labelPos, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(255, 255, 255), 1);
        }
    }

private:
    void initializeClassNames() {
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
    }

    // Improved depth calculation based on your depth-XYZ implementation
    void calculateRobustDepth(Detection& detection, const cv::Mat& depthFrame) {
        cv::Rect safeRect = detection.bbox & cv::Rect(0, 0, depthFrame.cols, depthFrame.rows);
        if (safeRect.area() <= 0) return;

        std::vector<float> depthSamples;

        // Sample multiple points in a grid pattern (from your implementation)
        int stepX = std::max(1, safeRect.width / 3);
        int stepY = std::max(1, safeRect.height / 3);

        for (int y = safeRect.y; y < safeRect.y + safeRect.height; y += stepY) {
            for (int x = safeRect.x; x < safeRect.x + safeRect.width; x += stepX) {
                uint16_t depthValue = depthFrame.at<uint16_t>(y, x);
                if (depthValue > 0 && depthValue < 8000) { // Valid depth range in mm
                    depthSamples.push_back(static_cast<float>(depthValue));
                }
            }
        }

        if (!depthSamples.empty()) {
            // Use median depth for robustness (from your depth-XYZ approach)
            std::sort(depthSamples.begin(), depthSamples.end());
            float medianDepth = depthSamples[depthSamples.size() / 2];

            detection.depth = medianDepth;
            detection.hasDepth = true;
            detection.depthSampleCount = static_cast<int>(depthSamples.size());

            // Calculate variance for quality assessment
            if (depthSamples.size() > 1) {
                float mean = std::accumulate(depthSamples.begin(), depthSamples.end(), 0.0f) /
                    static_cast<float>(depthSamples.size());
                float variance = 0.0f;
                for (float depth : depthSamples) {
                    variance += (depth - mean) * (depth - mean);
                }
                detection.depthVariance = variance / static_cast<float>(depthSamples.size() - 1);
            }
        }
    }
};

// Global variables
CameraParam g_camera_parm;
std::shared_ptr<ViewerHelper> viewer_helper;
bool is_running = true;
std::unique_ptr<ImprovedYOLODetector> yolo_detector;

// Feature flags
bool g_enableYOLO = false;
bool g_enableXYZ = false;
bool g_enable3D = false;
bool g_show3DWindow = false;

// Open3D 3D visualization variables
std::shared_ptr<open3d::geometry::PointCloud> g_point_cloud;
std::shared_ptr<open3d::visualization::Visualizer> g_visualizer;
open3d::camera::PinholeCameraIntrinsic g_intrinsic;
std::thread g_3d_thread;
std::mutex g_point_cloud_mutex;

// Function declarations
void calculateXYZForDetections(std::vector<ImprovedYOLODetector::Detection>& detections, const cv::Mat& depthFrame);
void printDetectionInfo(const std::vector<ImprovedYOLODetector::Detection>& detections);
void saveDetectionsToCSV(const std::vector<ImprovedYOLODetector::Detection>& detections);
void initialize3DVisualization();
void generate3DPointCloud(const cv::Mat& rgbFrame, const cv::Mat& depthFrame,
    const std::vector<ImprovedYOLODetector::Detection>& detections);

// Enhanced frame callback with optional features
void EnhancedFrameCallback(const StreamFrames& frames) {
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

    // Process with optional YOLO detection
    std::vector<ImprovedYOLODetector::Detection> detections;
    if (g_enableYOLO && yolo_detector && yolo_detector->isReady() && !rgbFrame.empty()) {
        detections = yolo_detector->detect(rgbFrame, depthFrame);

        // Calculate XYZ coordinates if enabled
        if (g_enableXYZ && !depthFrame.empty()) {
            calculateXYZForDetections(detections, depthFrame);
        }

        // Draw detections
        if (!detections.empty()) {
            yolo_detector->drawDetections(rgbFrame, detections);

            // Print info occasionally
            if (frame_count % 30 == 0) {
                printDetectionInfo(detections);
            }
        }
    }

    // Generate 3D point cloud if enabled
    if (g_enable3D && !rgbFrame.empty() && !depthFrame.empty()) {
        generate3DPointCloud(rgbFrame, depthFrame, detections);
    }

    // Display the frame
    if (!rgbFrame.empty()) {
        std::string windowTitle = "Depth Scanner";
        if (g_enableYOLO) windowTitle += " + YOLO";
        if (g_enableXYZ) windowTitle += " + XYZ";
        if (g_enable3D) windowTitle += " + 3D";

        cv::imshow(windowTitle, rgbFrame);
    }

    // Show depth visualization
    viewer_helper->ShowFrame(frames, g_camera_parm);
}

void calculateXYZForDetections(std::vector<ImprovedYOLODetector::Detection>& detections, const cv::Mat& depthFrame) {
    for (auto& detection : detections) {
        if (!detection.hasDepth) continue;

        cv::Point2f center(detection.bbox.x + detection.bbox.width / 2.0f,
            detection.bbox.y + detection.bbox.height / 2.0f);

        float depthMeters = detection.depth / 1000.0f; // Convert mm to meters

        // Calculate XYZ world coordinates using camera intrinsics
        detection.worldX = (center.x - g_camera_parm.cx) * depthMeters / g_camera_parm.fx;
        detection.worldY = (center.y - g_camera_parm.cy) * depthMeters / g_camera_parm.fy;
        detection.worldZ = depthMeters;
        detection.hasWorldPos = true;

        detection.distance3D = std::sqrt(detection.worldX * detection.worldX +
            detection.worldY * detection.worldY +
            detection.worldZ * detection.worldZ);
    }
}

void printDetectionInfo(const std::vector<ImprovedYOLODetector::Detection>& detections) {
    if (detections.empty()) return;

    std::cout << "\n=== Detections ===" << std::endl;
    for (const auto& detection : detections) {
        std::cout << detection.className
            << " (" << std::fixed << std::setprecision(1) << (detection.confidence * 100) << "%)";

        if (detection.hasWorldPos) {
            std::cout << " XYZ:("
                << std::setprecision(2) << detection.worldX << ","
                << detection.worldY << "," << detection.worldZ << ")m"
                << " dist:" << detection.distance3D << "m";
        }
        std::cout << std::endl;
    }
}

void saveDetectionsToCSV(const std::vector<ImprovedYOLODetector::Detection>& detections) {
    auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    std::ostringstream filename;
    filename << "detections_" << timestamp << ".csv";

    std::ofstream file(filename.str());
    if (!file.is_open()) return;

    file << "Timestamp,Class,Confidence,X_meters,Y_meters,Z_meters,Distance_meters\n";

    for (const auto& detection : detections) {
        file << timestamp << "," << detection.className << ","
            << std::fixed << std::setprecision(3) << detection.confidence;

        if (detection.hasWorldPos) {
            file << "," << detection.worldX << "," << detection.worldY
                << "," << detection.worldZ << "," << detection.distance3D;
        }
        else {
            file << ",,,,";
        }
        file << "\n";
    }

    file.close();
    std::cout << "Saved detections to: " << filename.str() << std::endl;
}

void initialize3DVisualization() {
    if (!g_enable3D) return;

    std::cout << "Initializing 3D visualization..." << std::endl;

    g_point_cloud = std::make_shared<open3d::geometry::PointCloud>();

    // Use the actual camera resolution from the selected mode
    int width = 640, height = 480; // Default, will be updated from actual frame
    g_intrinsic.SetIntrinsics(width, height,
        g_camera_parm.fx, g_camera_parm.fy,
        g_camera_parm.cx, g_camera_parm.cy);

    // Only create visualizer window if requested and avoid thread issues
    if (g_show3DWindow) {
        try {
            std::cout << "Creating 3D window (this may take a moment)..." << std::endl;
            // Create visualizer in main thread first, then move to separate thread
            g_visualizer = std::make_shared<open3d::visualization::Visualizer>();

            // Small delay to ensure proper initialization
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

        }
        catch (const std::exception& ex) {
            std::cerr << "Failed to create 3D visualizer: " << ex.what() << std::endl;
            std::cerr << "Continuing without 3D window..." << std::endl;
            g_show3DWindow = false;
            g_visualizer.reset();
        }
    }

    std::cout << "3D visualization ready" << std::endl;
}

void generate3DPointCloud(const cv::Mat& rgbFrame, const cv::Mat& depthFrame,
    const std::vector<ImprovedYOLODetector::Detection>& detections) {
    if (!g_enable3D || rgbFrame.empty() || depthFrame.empty()) return;

    // Skip if processing too frequently to avoid crashes
    static auto last_process_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_process_time);

    if (elapsed.count() < 100) { // Process max every 100ms (10 FPS) to avoid overload
        return;
    }
    last_process_time = now;

    try {
        // Update intrinsics with actual frame size
        if (g_intrinsic.width_ != rgbFrame.cols || g_intrinsic.height_ != rgbFrame.rows) {
            g_intrinsic.SetIntrinsics(rgbFrame.cols, rgbFrame.rows,
                g_camera_parm.fx, g_camera_parm.fy,
                g_camera_parm.cx, g_camera_parm.cy);
            std::cout << "Updated 3D intrinsics: " << rgbFrame.cols << "x" << rgbFrame.rows << std::endl;
        }

        open3d::geometry::Image color_image, depth_image;

        // Convert BGR to RGB for Open3D
        cv::Mat rgbImage;
        cv::cvtColor(rgbFrame, rgbImage, cv::COLOR_BGR2RGB);

        // Prepare color image
        color_image.Prepare(rgbFrame.cols, rgbFrame.rows, 3, sizeof(uint8_t));
        std::memcpy(color_image.data_.data(), rgbImage.data, rgbImage.total() * rgbImage.elemSize());

        // Convert depth to float and prepare
        cv::Mat depthFloat;
        depthFrame.convertTo(depthFloat, CV_32F, 1.0 / 1000.0); // mm to meters
        depth_image.Prepare(depthFrame.cols, depthFrame.rows, 1, sizeof(float));
        std::memcpy(depth_image.data_.data(), depthFloat.data, depthFloat.total() * depthFloat.elemSize());

        // Create RGBD image
        auto rgbd = open3d::geometry::RGBDImage::CreateFromColorAndDepth(
            color_image, depth_image, 1.0, 4.0, false);

        // Generate point cloud
        auto new_pcd = open3d::geometry::PointCloud::CreateFromRGBDImage(*rgbd, g_intrinsic);

        // Apply transform for correct orientation
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        transform(1, 1) = -1; // Flip Y
        transform(2, 2) = -1; // Flip Z
        new_pcd->Transform(transform);

        // Downsample to reduce memory usage and improve performance
        if (new_pcd->points_.size() > 50000) { // If too many points
            new_pcd = new_pcd->VoxelDownSample(0.01); // 1cm voxels
        }

        // Update global point cloud safely
        {
            std::lock_guard<std::mutex> lock(g_point_cloud_mutex);
            if (g_point_cloud) {
                g_point_cloud->points_ = new_pcd->points_;
                g_point_cloud->colors_ = new_pcd->colors_;
            }
        }

        // Update visualizer if it exists (non-blocking)
        if (g_show3DWindow && g_visualizer) {
            static bool visualizer_initialized = false;
            if (!visualizer_initialized) {
                try {
                    g_visualizer->CreateVisualizerWindow("3D Point Cloud", 1024, 768);
                    g_visualizer->AddGeometry(g_point_cloud);
                    visualizer_initialized = true;
                    std::cout << "3D window created successfully" << std::endl;
                }
                catch (const std::exception& ex) {
                    std::cerr << "Failed to create 3D window: " << ex.what() << std::endl;
                    g_show3DWindow = false;
                    g_visualizer.reset();
                    return;
                }
            }

            // Non-blocking update
            try {
                if (g_visualizer->PollEvents()) {
                    std::lock_guard<std::mutex> lock(g_point_cloud_mutex);
                    g_visualizer->UpdateGeometry(g_point_cloud);
                    // UpdateRenderer() doesn't exist in this Open3D version
                    // The visualization updates automatically with PollEvents()
                }
                else {
                    // Window was closed
                    g_show3DWindow = false;
                    g_visualizer.reset();
                }
            }
            catch (const std::exception& ex) {
                std::cerr << "3D window update error: " << ex.what() << std::endl;
                g_show3DWindow = false;
                g_visualizer.reset();
            }
        }

    }
    catch (const std::exception& ex) {
        std::cerr << "3D generation error: " << ex.what() << std::endl;
        // Don't crash, just disable 3D
        g_enable3D = false;
        std::cout << "3D visualization disabled due to errors" << std::endl;
    }
}

// Input handling function - fixes the input reading issue
void getUserConfiguration() {
    std::cout << "\n========== Scanner Configuration ==========" << std::endl;
    std::cout << "Choose features to enable:" << std::endl;
    std::cout << "1. Enable YOLO object detection? (y/n): ";

    std::string input;
    std::getline(std::cin, input);
    g_enableYOLO = (!input.empty() && (input[0] == 'y' || input[0] == 'Y'));

    if (g_enableYOLO) {
        std::cout << "2. Enable XYZ coordinate calculation? (y/n): ";
        std::getline(std::cin, input);
        g_enableXYZ = (!input.empty() && (input[0] == 'y' || input[0] == 'Y'));
    }

    std::cout << "3. Enable 3D point cloud generation? (y/n): ";
    std::getline(std::cin, input);
    g_enable3D = (!input.empty() && (input[0] == 'y' || input[0] == 'Y'));

    if (g_enable3D) {
        std::cout << "4. Show 3D visualization window? (y/n): ";
        std::getline(std::cin, input);
        g_show3DWindow = (!input.empty() && (input[0] == 'y' || input[0] == 'Y'));
    }

    std::cout << "\nConfiguration:" << std::endl;
    std::cout << "  YOLO Detection: " << (g_enableYOLO ? "ON" : "OFF") << std::endl;
    std::cout << "  XYZ Coordinates: " << (g_enableXYZ ? "ON" : "OFF") << std::endl;
    std::cout << "  3D Point Cloud: " << (g_enable3D ? "ON" : "OFF") << std::endl;
    std::cout << "  3D Window: " << (g_show3DWindow ? "ON" : "OFF") << std::endl;
    std::cout << "===========================================" << std::endl;
}

// Main function
int main() {
    std::cout << "========== Depth Scanner ==========" << std::endl;
    std::cout << "Advanced depth camera scanner with optional features" << std::endl;
    std::cout << "====================================" << std::endl;

    // Get user configuration
    getUserConfiguration();

    // Initialize YOLO only if requested
    if (g_enableYOLO) {
        try {
            yolo_detector = std::make_unique<ImprovedYOLODetector>("yolov8n.onnx");
            if (!yolo_detector->isReady()) {
                std::cout << "YOLO model not found, running without object detection" << std::endl;
                g_enableYOLO = false;
                g_enableXYZ = false;
            }
            else {
                std::cout << "YOLO detector initialized successfully" << std::endl;
            }
        }
        catch (const std::exception& e) {
            std::cout << "YOLO initialization failed, running without object detection" << std::endl;
            g_enableYOLO = false;
            g_enableXYZ = false;
        }
    }

    is_running = true;

    // Initialize depth camera (your existing code pattern)
    DeviceManager::GetInstance()->RegisterDeviceConnectedCallback();

    std::vector<DeviceInformation> device_list;
    int ret = DeviceManager::GetInstance()->GetDeviceList(device_list);
    CHECK_SDK_RETURN_VALUE(ret);
    CHECK_DEVICE_COUNT(device_list.size());

    {
        printf("Found %zu depth camera(s):\n", device_list.size());
        for (size_t index = 0; index < device_list.size(); index++) {
            printf("  [%zu] device_addr: %d, usb_port: %s\n", index,
                device_list[index].ir_camera.device_addr,
                device_list[index].ir_camera.port_path.c_str());
        }
    }

    auto device = DeviceManager::GetInstance()->CreateDevice(device_list[0]);
    CHECK_DEVICE_VALID(device);

    ret = device->Open();
    CHECK_SDK_RETURN_VALUE(ret);

    std::string device_name = device->GetDeviceName();
    viewer_helper = std::make_shared<ViewerHelper>(device_name);

    // Get supported frame modes
    std::vector<std::tuple<FrameMode, FrameMode, FrameMode>> device_resolution_vec;
    ret = device->GetSupportedFrameMode(device_resolution_vec);
    CHECK_SDK_RETURN_VALUE(ret);

    if (device_resolution_vec.empty()) {
        std::cout << "No supported frame modes found!" << std::endl;
        return -1;
    }

    std::cout << "\nSelect frame mode:" << std::endl;
    int selected_index = ChooceFrameMode(device_resolution_vec);

    FrameMode ir_mode = std::get<0>(device_resolution_vec[selected_index]);
    FrameMode rgb_mode = std::get<1>(device_resolution_vec[selected_index]);
    FrameMode depth_mode = std::get<2>(device_resolution_vec[selected_index]);

    ret = device->SetMode(ir_mode, rgb_mode, depth_mode);
    CHECK_SDK_RETURN_VALUE(ret);

    // Aurora-specific setup
    if (device_name.find("Aurora") != std::string::npos) {
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

    std::cout << "\nCamera parameters:" << std::endl;
    std::cout << "  fx: " << g_camera_parm.fx << ", fy: " << g_camera_parm.fy << std::endl;
    std::cout << "  cx: " << g_camera_parm.cx << ", cy: " << g_camera_parm.cy << std::endl;

    // Initialize 3D visualization if enabled
    if (g_enable3D) {
        initialize3DVisualization();
        std::cout << "3D visualization initialized" << std::endl;
    }

    // Create stream
    Stream* stream = nullptr;
    std::vector<StreamType> stream_types = { StreamType::kRgbd };
    ret = device->CreateStream(stream, stream_types);
    CHECK_SDK_RETURN_VALUE(ret);

    ret = stream->Start();
    CHECK_SDK_RETURN_VALUE(ret);

    std::cout << "\n========== Scanner Started ==========" << std::endl;
    std::cout << "Runtime Controls:" << std::endl;
    std::cout << "  'q' or ESC  - Quit" << std::endl;
    std::cout << "  'h'         - Show this help" << std::endl;
    if (g_enableYOLO) {
        std::cout << "  't'         - Toggle YOLO detection" << std::endl;
        if (g_enableXYZ) {
            std::cout << "  'x'         - Toggle XYZ coordinates" << std::endl;
            std::cout << "  'c'         - Save detections to CSV" << std::endl;
        }
    }
    std::cout << "  '3'         - Toggle 3D point cloud" << std::endl;
    std::cout << "  'w'         - Toggle 3D window" << std::endl;
    std::cout << "  's'         - Save 3D point cloud" << std::endl;
    std::cout << "=====================================" << std::endl;

    // Main processing loop with proper keyboard handling
    while (is_running) {
        StreamFrames frames;
        ret = stream->GetFrames(frames, 2000);

        if (ret == 0 && frames.count > 0) {
            EnhancedFrameCallback(frames);
        }

        // Non-blocking keyboard input handling - FIXED for runtime input
        int key = cv::waitKey(1) & 0xFF;
        if (key != 255) { // Key was pressed
            switch (key) {
            case 'q':
            case 27: // ESC
                std::cout << "\nShutting down scanner..." << std::endl;
                is_running = false;
                break;

            case 'h':
                std::cout << "\n========== Help ==========" << std::endl;
                std::cout << "Current status:" << std::endl;
                std::cout << "  YOLO Detection: " << (g_enableYOLO ? "ON" : "OFF") << std::endl;
                std::cout << "  XYZ Coordinates: " << (g_enableXYZ ? "ON" : "OFF") << std::endl;
                std::cout << "  3D Point Cloud: " << (g_enable3D ? "ON" : "OFF") << std::endl;
                std::cout << "==========================" << std::endl;
                break;

            case 't':
                if (yolo_detector && yolo_detector->isReady()) {
                    g_enableYOLO = !g_enableYOLO;
                    std::cout << "YOLO detection: " << (g_enableYOLO ? "ON" : "OFF") << std::endl;
                    if (!g_enableYOLO) {
                        g_enableXYZ = false; // Turn off XYZ if YOLO is off
                        std::cout << "XYZ coordinates: OFF (requires YOLO)" << std::endl;
                    }
                }
                else {
                    std::cout << "YOLO detector not available" << std::endl;
                }
                break;

            case 'x':
                if (g_enableYOLO) {
                    g_enableXYZ = !g_enableXYZ;
                    std::cout << "XYZ coordinates: " << (g_enableXYZ ? "ON" : "OFF") << std::endl;
                }
                else {
                    std::cout << "XYZ requires YOLO detection to be enabled" << std::endl;
                }
                break;

            case 'c':
                if (g_enableYOLO && g_enableXYZ) {
                    std::cout << "Saving detections to CSV on next frame..." << std::endl;
                    // Set a flag to save on next detection
                    static bool save_next = true;
                    // You could implement this with a global flag
                }
                else {
                    std::cout << "CSV save requires YOLO and XYZ to be enabled" << std::endl;
                }
                break;

            case 's':
                if (g_enable3D) {
                    // Save current point cloud
                    auto timestamp = std::chrono::duration_cast<std::chrono::seconds>(
                        std::chrono::system_clock::now().time_since_epoch()).count();
                    std::ostringstream filename;
                    filename << "pointcloud_" << timestamp << ".ply";

                    if (g_point_cloud && !g_point_cloud->points_.empty()) {
                        std::lock_guard<std::mutex> lock(g_point_cloud_mutex);
                        if (open3d::io::WritePointCloud(filename.str(), *g_point_cloud)) {
                            std::cout << "Saved point cloud: " << filename.str() << std::endl;
                        }
                        else {
                            std::cout << "Failed to save point cloud" << std::endl;
                        }
                    }
                    else {
                        std::cout << "No point cloud data to save" << std::endl;
                    }
                }
                else {
                    std::cout << "3D point cloud not enabled" << std::endl;
                }
                break;

            case '3':
                // Toggle 3D visualization
                g_enable3D = !g_enable3D;
                std::cout << "3D point cloud: " << (g_enable3D ? "ON" : "OFF") << std::endl;

                if (g_enable3D) {
                    // Re-initialize 3D
                    initialize3DVisualization();
                }
                else {
                    // Clean up 3D resources
                    if (g_visualizer) {
                        try {
                            g_visualizer->DestroyVisualizerWindow();
                            g_visualizer.reset();
                        }
                        catch (...) {
                            // Ignore cleanup errors
                        }
                    }
                    g_show3DWindow = false;
                }
                break;

            case 'w':
                // Toggle 3D window only
                if (g_enable3D) {
                    g_show3DWindow = !g_show3DWindow;
                    std::cout << "3D window: " << (g_show3DWindow ? "ON" : "OFF") << std::endl;

                    if (!g_show3DWindow && g_visualizer) {
                        try {
                            g_visualizer->DestroyVisualizerWindow();
                            g_visualizer.reset();
                        }
                        catch (...) {
                            // Ignore errors
                        }
                    }
                }
                else {
                    std::cout << "Enable 3D point cloud first (press '3')" << std::endl;
                }
                break;

            default:
                // Ignore other keys
                break;
            }
        }
    }

    // Cleanup
    std::cout << "Cleaning up..." << std::endl;

    stream->Stop();
    device->DestroyStream(stream);
    device->Close();

    // Cleanup 3D visualization thread
    if (g_enable3D) {
        std::cout << "Cleaning up 3D visualization..." << std::endl;

        // Close 3D window first
        if (g_visualizer) {
            try {
                g_visualizer->DestroyVisualizerWindow();
                g_visualizer.reset();
            }
            catch (const std::exception& ex) {
                std::cerr << "Error closing 3D window: " << ex.what() << std::endl;
            }
        }

        // Clean up thread if it exists
        if (g_3d_thread.joinable()) {
            g_3d_thread.join();
        }

        // Clean up point cloud
        {
            std::lock_guard<std::mutex> lock(g_point_cloud_mutex);
            if (g_point_cloud) {
                g_point_cloud.reset();
            }
        }
    }

    cv::destroyAllWindows();

    std::cout << "========== Scanner Finished ==========" << std::endl;
    std::cout << "Thank you for using Depth Scanner!" << std::endl;
    std::cout << "======================================" << std::endl;

    return 0;
}

#pragma warning(pop)