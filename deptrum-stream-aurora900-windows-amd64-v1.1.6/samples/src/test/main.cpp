#include <iostream>
#include <thread>
#include <vector>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <cstdio>
#include "deptrum/device.h"
#include "deptrum/stream.h"
#include "deptrum/aurora900_series.h"
#include "functional/base.h"
#include "functional/frame_rate_helper.h"
#include "sample_helper.h"

using namespace std;
using namespace deptrum;
using namespace deptrum::stream;

// Global variables
bool is_running = true;
bool object_detected = false;

// Enhanced object detection parameters
struct ObjectDetectionParams {
    // Color thresholds (BGR format like OpenCV)
    int target_b_min = 100, target_b_max = 255;  // Blue channel
    int target_g_min = 0, target_g_max = 100;   // Green channel  
    int target_r_min = 0, target_r_max = 100;   // Red channel

    // Depth range (in mm)
    int depth_min = 300;  // 30cm
    int depth_max = 1500; // 150cm

    // Detection area (to avoid noise at edges)
    float roi_x_min = 0.1f; // 10% from left
    float roi_x_max = 0.9f; // 90% from left
    float roi_y_min = 0.1f; // 10% from top
    float roi_y_max = 0.9f; // 90% from top

    // Clustering parameters
    int min_cluster_size = 50;     // Minimum points for valid object
    float cluster_tolerance = 30.0f; // mm - points within this distance are clustered

    // Stability parameters
    int detection_confidence_threshold = 5; // Consecutive detections needed
    float position_stability_threshold = 50.0f; // mm - max movement for stable detection
};

ObjectDetectionParams detect_params;

// Object tracking structure
struct DetectedObject {
    float x, y, z;           // Position in mm
    float distance;          // Distance from camera in mm
    int confidence;          // Detection confidence counter
    bool is_stable;          // Whether object position is stable
    int point_count;         // Number of points in cluster
    uint64_t last_seen;      // Timestamp of last detection

    DetectedObject() : x(0), y(0), z(0), distance(0), confidence(0),
        is_stable(false), point_count(0), last_seen(0) {}
};

DetectedObject tracked_object;

// Enhanced function to check if pixel represents target object
bool isTargetObject(uint8_t b, uint8_t g, uint8_t r, uint16_t depth) {
    // Check color range
    bool color_match = (r >= detect_params.target_r_min && r <= detect_params.target_r_max) &&
        (g >= detect_params.target_g_min && g <= detect_params.target_g_max) &&
        (b >= detect_params.target_b_min && b <= detect_params.target_b_max);

    // Check depth range
    bool depth_match = (depth >= detect_params.depth_min && depth <= detect_params.depth_max);

    return color_match && depth_match;
}

// Calculate 3D distance from camera origin
float calculate3DDistance(float x, float y, float z) {
    return static_cast<float>(sqrt(x * x + y * y + z * z));
}

// Enhanced function to send coordinates to servo arm
void sendToServoArm(const DetectedObject& obj) {
    printf("\n=== STABLE OBJECT DETECTED ===\n");
    printf("Position (mm):\n");
    printf("  X: %.2f mm\n", obj.x);
    printf("  Y: %.2f mm\n", obj.y);
    printf("  Z: %.2f mm\n", obj.z);
    printf("Distance from camera: %.2f mm\n", obj.distance);
    printf("Point count: %d\n", obj.point_count);
    printf("Confidence: %d/%d\n", obj.confidence, detect_params.detection_confidence_threshold);
    printf("===============================\n");

    object_detected = true;

    // TODO: Add your servo control code here
    // Example: ServoController::moveTo(obj.x, obj.y, obj.z);
}

// Calculate centroid of detected points
DetectedObject calculateCentroid(const vector<PointXyzRgbIr<float>>& points, uint64_t timestamp) {
    DetectedObject obj;

    if (points.empty()) return obj;

    float sum_x = 0, sum_y = 0, sum_z = 0;
    for (const auto& point : points) {
        sum_x += point.x;
        sum_y += point.y;
        sum_z += point.z;
    }

    obj.x = sum_x / static_cast<float>(points.size());
    obj.y = sum_y / static_cast<float>(points.size());
    obj.z = sum_z / static_cast<float>(points.size());
    obj.distance = calculate3DDistance(obj.x, obj.y, obj.z);
    obj.point_count = static_cast<int>(points.size());
    obj.last_seen = timestamp;

    return obj;
}

// Process point cloud data for enhanced object detection
void processPointCloudFrame(std::shared_ptr<StreamFrame> frame) {
    if (frame->frame_type != FrameType::kPointCloudFrame) return;

    // Cast data to point cloud format
    PointXyzRgbIr<float>* point_cloud = static_cast<PointXyzRgbIr<float>*>(frame->data.get());
    int total_points = frame->rows * frame->cols;

    // Calculate ROI bounds
    int roi_start_x = static_cast<int>(frame->cols * detect_params.roi_x_min);
    int roi_end_x = static_cast<int>(frame->cols * detect_params.roi_x_max);
    int roi_start_y = static_cast<int>(frame->rows * detect_params.roi_y_min);
    int roi_end_y = static_cast<int>(frame->rows * detect_params.roi_y_max);

    std::vector<PointXyzRgbIr<float>> detected_points;

    // Process points in ROI
    for (int row = roi_start_y; row < roi_end_y; row++) {
        for (int col = roi_start_x; col < roi_end_x; col++) {
            int idx = row * frame->cols + col;
            if (idx >= total_points) continue;

            float x = point_cloud[idx].x;
            float y = point_cloud[idx].y;
            float z = point_cloud[idx].z;
            uint8_t r = point_cloud[idx].r;
            uint8_t g = point_cloud[idx].g;
            uint8_t b = point_cloud[idx].b;

            // Check if point has valid depth
            if (z <= 0 || z > 5000) continue; // Skip invalid or too far points

            // Check if this point represents target object
            if (isTargetObject(b, g, r, static_cast<uint16_t>(z))) {
                detected_points.push_back(point_cloud[idx]);
            }
        }
    }

    // If we found target points, calculate centroid
    if (!detected_points.empty() && static_cast<int>(detected_points.size()) >= detect_params.min_cluster_size) {
        DetectedObject current_obj = calculateCentroid(detected_points, frame->timestamp);

        // Check if this is the same object as before (position stability)
        if (tracked_object.confidence > 0) {
            float dx = current_obj.x - tracked_object.x;
            float dy = current_obj.y - tracked_object.y;
            float dz = current_obj.z - tracked_object.z;
            float position_diff = static_cast<float>(sqrt(dx * dx + dy * dy + dz * dz));

            if (position_diff <= detect_params.position_stability_threshold) {
                // Same object, update tracking
                tracked_object.x = current_obj.x;
                tracked_object.y = current_obj.y;
                tracked_object.z = current_obj.z;
                tracked_object.distance = current_obj.distance;
                tracked_object.point_count = current_obj.point_count;
                tracked_object.last_seen = current_obj.last_seen;
                tracked_object.confidence++;

                if (tracked_object.confidence >= detect_params.detection_confidence_threshold) {
                    tracked_object.is_stable = true;
                    sendToServoArm(tracked_object);
                }
            }
            else {
                // Different object, reset tracking
                tracked_object = current_obj;
                tracked_object.confidence = 1;
            }
        }
        else {
            // First detection
            tracked_object = current_obj;
            tracked_object.confidence = 1;
        }

        printf("Tracking: (%.1f, %.1f, %.1f) mm, Dist: %.1f mm, Conf: %d/%d, Pts: %d\n",
            tracked_object.x, tracked_object.y, tracked_object.z, tracked_object.distance,
            tracked_object.confidence, detect_params.detection_confidence_threshold, tracked_object.point_count);
    }
    else {
        // No points detected, decrease confidence
        if (tracked_object.confidence > 0) {
            tracked_object.confidence--;
            if (tracked_object.confidence <= 0) {
                tracked_object.is_stable = false;
                printf("Object lost\n");
            }
        }
    }
}

// Enhanced setup function
void setupDetectionParams() {
    printf("\n=== Enhanced Object Detection Configuration ===\n");

    char choice;
    printf("Use default parameters? (y/n): ");
    scanf_s("%c", &choice, 1);

    // Clear input buffer
    int c;
    while ((c = getchar()) != '\n' && c != EOF) {}

    if (choice == 'n' || choice == 'N') {
        printf("\n--- Color Detection Range (0-255) ---\n");
        printf("Blue min: "); scanf_s("%d", &detect_params.target_b_min);
        printf("Blue max: "); scanf_s("%d", &detect_params.target_b_max);
        printf("Green min: "); scanf_s("%d", &detect_params.target_g_min);
        printf("Green max: "); scanf_s("%d", &detect_params.target_g_max);
        printf("Red min: "); scanf_s("%d", &detect_params.target_r_min);
        printf("Red max: "); scanf_s("%d", &detect_params.target_r_max);

        printf("\n--- Depth Range (mm) ---\n");
        printf("Depth min (mm): "); scanf_s("%d", &detect_params.depth_min);
        printf("Depth max (mm): "); scanf_s("%d", &detect_params.depth_max);

        printf("\n--- Advanced Parameters ---\n");
        printf("Minimum cluster size (points): "); scanf_s("%d", &detect_params.min_cluster_size);
        printf("Cluster tolerance (mm): "); scanf_s("%f", &detect_params.cluster_tolerance);
        printf("Detection confidence threshold: "); scanf_s("%d", &detect_params.detection_confidence_threshold);
    }

    printf("\nDetection Parameters:\n");
    printf("Color (BGR): B[%d-%d] G[%d-%d] R[%d-%d]\n",
        detect_params.target_b_min, detect_params.target_b_max,
        detect_params.target_g_min, detect_params.target_g_max,
        detect_params.target_r_min, detect_params.target_r_max);
    printf("Depth: %d-%d mm\n", detect_params.depth_min, detect_params.depth_max);
    printf("Min cluster size: %d points\n", detect_params.min_cluster_size);
    printf("Cluster tolerance: %.1f mm\n", detect_params.cluster_tolerance);
    printf("Confidence threshold: %d\n", detect_params.detection_confidence_threshold);
    printf("Configuration complete!\n");
}

// Object detection stream process - based on sample.cc structure
int ObjectDetectionStreamProcess(std::shared_ptr<Device> device,
    const std::vector<StreamType>& stream_type,
    long frame_loop_times,
    std::string device_name) {
    Stream* stream = nullptr;

    int ret = device->CreateStream(stream, stream_type);
    CHECK_SDK_RETURN_VALUE(ret);

    ret = stream->Start();
    CHECK_SDK_RETURN_VALUE(ret);

    StreamFrames frames;
    FrameRateHelper frame_rate_helper;
    long cnt = 0;

    printf("\nStarting enhanced object detection...\n");
    printf("Press any key in the console to stop...\n");

    while (is_running) {
        if (frame_loop_times == -1) {
            // Run indefinitely
        }
        else {
            if (frame_loop_times == 0) break;
            frame_loop_times--;
        }

        ret = stream->GetFrames(frames, 2000);
        if (ret != 0) {
            printf("Failed to get frame, error: %d\n", ret);
            continue;
        }

        frame_rate_helper.RecordTimestamp();
        if (0 == cnt++ % 30) { // Print FPS every 30 frames
            printf("FPS: %.1f\n", frame_rate_helper.GetFrameRate());
        }

        // Process each frame for object detection
        for (int i = 0; i < frames.count; i++) {
            auto frame = frames.frame_ptr[i];
            if (frame->frame_type == FrameType::kPointCloudFrame) {
                //processPointCloudFrame(frame);
                printf("Got frame type: %d\n", frame->frame_type);
            }
        }
    }

    stream->Stop();
    device->DestroyStream(stream);
    return 0;
}

// Main preparation function - closely follows sample.cc structure
int PrepareObjectDetection() {
    is_running = true;

    // Setup detection parameters first
    setupDetectionParams();

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

    // Create a device, 0 means the index of the first device
    std::shared_ptr<Device> device = DeviceManager::GetInstance()->CreateDevice(device_list[0]);
    CHECK_DEVICE_VALID(device);

    // Print the sdk version number
    auto sdk_version = device->GetSdkVersion();
    printf("SDK version: %s\n", sdk_version.c_str());

    ret = device->Open();
    CHECK_SDK_RETURN_VALUE(ret);

    // Get device name
    std::string device_name = device->GetDeviceName();
    printf("Device name: %s\n", device_name.c_str());

    // Set camera support mode
    FrameMode ir_mode, rgb_mode, depth_mode;
    std::vector<std::tuple<FrameMode, FrameMode, FrameMode>> device_resolution_vec;
    ret = device->GetSupportedFrameMode(device_resolution_vec);
    if (ret != 0) {
        printf("GetSupportedFrameMode error: %d\n", ret);
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

        int ir_fps = 15;
        printf("Setting IR FPS to %d\n", ir_fps);
        device_unique_ptr->SetIrFps(ir_fps);

        int align_mode_i = 1;
        bool align_mode = true;
        printf("Enabling aligned mode\n");
        device_unique_ptr->SwitchAlignedMode(align_mode);

        if (align_mode == true) {
            bool depth_correct = true;
            printf("Enabling depth correction\n");
            device_unique_ptr->DepthCorrection(depth_correct);
        }
    }

    // Create stream - prefer Point Cloud for object detection
    std::vector<StreamType> stream_types_vector;

    // Check supported stream types
    std::vector<StreamType> supported_streams;
    device->GetSupportedStreamType(supported_streams);

    bool point_cloud_supported = false;
    for (auto stream_type : supported_streams) {
        if (stream_type == StreamType::kPointCloud) {
            point_cloud_supported = true;
            break;
        }
    }

    if (point_cloud_supported) {
        printf("Using Point Cloud stream for object detection\n");
        stream_types_vector.push_back(StreamType::kPointCloud);
    }
    else {
        printf("Point Cloud not supported, using RGBD stream\n");
        stream_types_vector.push_back(StreamType::kRgbd);
    }

    if (stream_types_vector.empty()) {
        printf("No suitable stream type found\n");
        return -1;
    }

    long frame_loop_times = -1; // Run indefinitely
    std::thread stream_thread = std::thread(ObjectDetectionStreamProcess,
        device,
        stream_types_vector,
        frame_loop_times,
        device_name);

    // Main control loop
    printf("\nControl commands:\n");
    printf("q: Quit\n");
    printf("r: Reset object tracking\n");
    printf("Press Enter after each command\n");

    while (is_running) {
        char key = getchar();
        switch (key) {
        case 'q':
        case 'Q':
            is_running = false;
            break;
        case 'r':
        case 'R':
            tracked_object = DetectedObject();
            object_detected = false;
            printf("Object tracking reset.\n");
            break;
        default:
            break;
        }
    }

    if (stream_thread.joinable()) {
        stream_thread.join();
    }

    device->Close();
    printf("Enhanced object detection completed.\n");
    return 0;
}

int main() {
    printf("Enhanced Aurora 900 Object Detection System\n");
    printf("===========================================\n");

    try {
        return PrepareObjectDetection();
    }
    catch (const std::exception& e) {
        printf("Exception: %s\n", e.what());
        return -1;
    }
}