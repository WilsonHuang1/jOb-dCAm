#include <iostream>
#include <thread>
#include <vector>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <cstdio>
#include <fstream>
#include "deptrum/device.h"
#include "deptrum/stream.h"
#include "deptrum/aurora900_series.h"
#include "functional/base.h"
#include "functional/frame_rate_helper.h"
#include "sample_helper.h"
#include "opencv2/opencv.hpp"

using namespace std;
using namespace deptrum;
using namespace deptrum::stream;

// Global variables
bool is_running = true;
bool is_print_fps = true;
bool enable_distance_detection = true;

// Distance detection parameters
struct DistanceDetectionParams {
    // Detection area (center region of the image)
    float roi_x_ratio = 0.3f;  // 30% width around center
    float roi_y_ratio = 0.3f;  // 30% height around center

    // Depth filtering
    uint16_t min_depth = 100;   // 10cm minimum
    uint16_t max_depth = 3000;  // 300cm maximum

    // Object detection
    int min_valid_points = 50;  // Minimum points to consider as object

    // Output settings
    bool save_to_file = false;
    std::string output_file = "distance_log.txt";
} detect_params;

// Structure to hold detection results
struct ObjectDistance {
    float distance_mm;
    float center_x, center_y;
    int valid_points;
    uint64_t timestamp;

    ObjectDistance() : distance_mm(0), center_x(0), center_y(0),
        valid_points(0), timestamp(0) {}
};

// Function to calculate distance from depth data
ObjectDistance calculateObjectDistance(std::shared_ptr<StreamFrame> depth_frame) {
    ObjectDistance result;

    if (!depth_frame || depth_frame->frame_type != FrameType::kDepthFrame) {
        return result;
    }

    uint16_t* depth_data = static_cast<uint16_t*>(depth_frame->data.get());
    int width = depth_frame->cols;
    int height = depth_frame->rows;

    // Calculate ROI bounds (center region)
    int roi_width = static_cast<int>(width * detect_params.roi_x_ratio);
    int roi_height = static_cast<int>(height * detect_params.roi_y_ratio);
    int roi_start_x = (width - roi_width) / 2;
    int roi_start_y = (height - roi_height) / 2;
    int roi_end_x = roi_start_x + roi_width;
    int roi_end_y = roi_start_y + roi_height;

    // Process ROI and collect valid depth points
    std::vector<uint16_t> valid_depths;
    float sum_x = 0, sum_y = 0;

    for (int y = roi_start_y; y < roi_end_y; y++) {
        for (int x = roi_start_x; x < roi_end_x; x++) {
            int idx = y * width + x;
            uint16_t depth = depth_data[idx];

            // Filter valid depth values
            if (depth >= detect_params.min_depth && depth <= detect_params.max_depth) {
                valid_depths.push_back(depth);
                sum_x += x;
                sum_y += y;
            }
        }
    }

    // Calculate results if we have enough valid points
    if (valid_depths.size() >= detect_params.min_valid_points) {
        // Calculate median distance (more robust than mean)
        std::sort(valid_depths.begin(), valid_depths.end());
        result.distance_mm = valid_depths[valid_depths.size() / 2];

        // Calculate center of detected object
        result.center_x = sum_x / valid_depths.size();
        result.center_y = sum_y / valid_depths.size();
        result.valid_points = valid_depths.size();
        result.timestamp = depth_frame->timestamp;
    }

    return result;
}

// Function to display depth frame with ROI visualization
void displayDepthWithROI(std::shared_ptr<StreamFrame> depth_frame, const ObjectDistance& detection) {
    if (!depth_frame || depth_frame->frame_type != FrameType::kDepthFrame) {
        return;
    }

    uint16_t* depth_data = static_cast<uint16_t*>(depth_frame->data.get());
    int width = depth_frame->cols;
    int height = depth_frame->rows;

    // Create colored depth image
    cv::Mat colored_depth = cv::Mat::zeros(height, width, CV_8UC3);

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int idx = y * width + x;
            uint16_t depth = depth_data[idx];

            if (depth > 0 && depth < 6000) {
                // Color mapping: closer = red, farther = blue
                float normalized = static_cast<float>(depth) / 6000.0f;
                colored_depth.at<cv::Vec3b>(y, x) = cv::Vec3b(
                    static_cast<uint8_t>(255 * (1 - normalized)), // Blue
                    static_cast<uint8_t>(128 * (1 - std::abs(normalized - 0.5f) * 2)), // Green
                    static_cast<uint8_t>(255 * normalized) // Red
                );
            }
        }
    }

    // Draw ROI rectangle
    int roi_width = static_cast<int>(width * detect_params.roi_x_ratio);
    int roi_height = static_cast<int>(height * detect_params.roi_y_ratio);
    int roi_start_x = (width - roi_width) / 2;
    int roi_start_y = (height - roi_height) / 2;

    cv::rectangle(colored_depth,
        cv::Point(roi_start_x, roi_start_y),
        cv::Point(roi_start_x + roi_width, roi_start_y + roi_height),
        cv::Scalar(0, 255, 0), 2);

    // Draw detection center if valid
    if (detection.valid_points > 0) {
        cv::circle(colored_depth,
            cv::Point(static_cast<int>(detection.center_x), static_cast<int>(detection.center_y)),
            5, cv::Scalar(0, 0, 255), -1);

        // Add distance text
        std::string distance_text = "Distance: " + std::to_string(static_cast<int>(detection.distance_mm)) + "mm";
        cv::putText(colored_depth, distance_text, cv::Point(10, 30),
            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);

        std::string points_text = "Points: " + std::to_string(detection.valid_points);
        cv::putText(colored_depth, points_text, cv::Point(10, 60),
            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    }

    cv::imshow("Depth with Distance Detection", colored_depth);
}

// Function to log distance data to file
void logDistanceData(const ObjectDistance& detection) {
    if (!detect_params.save_to_file || detection.valid_points == 0) {
        return;
    }

    static bool first_write = true;
    std::ofstream file(detect_params.output_file, std::ios::app);

    if (first_write) {
        file << "Timestamp,Distance_mm,Center_X,Center_Y,Valid_Points\n";
        first_write = false;
    }

    file << detection.timestamp << ","
        << detection.distance_mm << ","
        << detection.center_x << ","
        << detection.center_y << ","
        << detection.valid_points << "\n";

    file.close();
}

// Stream processing function with distance detection
int DistanceDetectionStreamProcess(std::shared_ptr<Device> device,
    const std::vector<StreamType>& stream_type,
    long frame_loop_times) {
    Stream* stream = nullptr;

    int ret = device->CreateStream(stream, stream_type);
    CHECK_SDK_RETURN_VALUE(ret);

    ret = stream->Start();
    CHECK_SDK_RETURN_VALUE(ret);

    StreamFrames frames;
    FrameRateHelper frame_rate_helper;
    long cnt = 0;

    std::cout << "\n=== Starting Distance Detection ===" << std::endl;
    std::cout << "ROI Size: " << (detect_params.roi_x_ratio * 100) << "% x " << (detect_params.roi_y_ratio * 100) << "%" << std::endl;
    std::cout << "Depth Range: " << detect_params.min_depth << "mm - " << detect_params.max_depth << "mm" << std::endl;
    std::cout << "Min Points: " << detect_params.min_valid_points << std::endl;
    std::cout << "Press 'q' to quit, 's' to toggle file saving" << std::endl;

    while (is_running) {
        if (frame_loop_times == -1) {
            // Run indefinitely
        }
        else {
            if (frame_loop_times == 0) break;
            frame_loop_times--;
        }

        ret = stream->GetFrames(frames, 2000);
        CHECK_GET_FRAMES(ret);

        frame_rate_helper.RecordTimestamp();
        if (is_print_fps && 0 == cnt++ % 30) {
            std::cout << "FPS: " << std::fixed << std::setprecision(1)
                << frame_rate_helper.GetFrameRate() << std::endl;
        }

        // Process frames for distance detection
        ObjectDistance detection;
        std::shared_ptr<StreamFrame> depth_frame = nullptr;

        for (int i = 0; i < frames.count; i++) {
            auto frame = frames.frame_ptr[i];

            if (frame->frame_type == FrameType::kDepthFrame) {
                depth_frame = frame;
                detection = calculateObjectDistance(frame);

                // Print detection results
                if (detection.valid_points > 0) {
                    std::cout << "\r[DETECTED] Distance: " << std::setw(4) << static_cast<int>(detection.distance_mm)
                        << "mm, Points: " << std::setw(4) << detection.valid_points
                        << ", Center: (" << std::setw(3) << static_cast<int>(detection.center_x)
                        << ", " << std::setw(3) << static_cast<int>(detection.center_y) << ")" << std::flush;

                    // Log to file if enabled
                    logDistanceData(detection);
                }
                else {
                    std::cout << "\r[NO OBJECT] Searching for objects in ROI..." << std::flush;
                }
            }
        }

        // Display depth image with ROI and detection
        if (depth_frame) {
            displayDepthWithROI(depth_frame, detection);
        }

        // Handle keyboard input
        char key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 'Q') {
            is_running = false;
        }
        else if (key == 's' || key == 'S') {
            detect_params.save_to_file = !detect_params.save_to_file;
            std::cout << "\nFile saving: " << (detect_params.save_to_file ? "ENABLED" : "DISABLED") << std::endl;
        }
    }

    stream->Stop();
    device->DestroyStream(stream);
    cv::destroyAllWindows();

    std::cout << "\n=== Distance Detection Completed ===" << std::endl;
    return 0;
}

// Main device preparation function
int PrepareDistanceDetection() {
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

        // Enable aligned mode for better depth accuracy
        std::cout << "Enabling aligned mode for better depth accuracy" << std::endl;
        device_unique_ptr->SwitchAlignedMode(true);
        device_unique_ptr->DepthCorrection(true);
    }

    // Configure detection parameters
    std::cout << "\n=== Distance Detection Configuration ===" << std::endl;
    char choice;
    std::cout << "Use default detection parameters? (y/n): ";
    std::cin >> choice;

    if (choice == 'n' || choice == 'N') {
        std::cout << "ROI width ratio (0.1-0.9, default 0.3): ";
        std::cin >> detect_params.roi_x_ratio;

        std::cout << "ROI height ratio (0.1-0.9, default 0.3): ";
        std::cin >> detect_params.roi_y_ratio;

        std::cout << "Minimum depth (mm, default 100): ";
        std::cin >> detect_params.min_depth;

        std::cout << "Maximum depth (mm, default 3000): ";
        std::cin >> detect_params.max_depth;

        std::cout << "Minimum valid points (default 50): ";
        std::cin >> detect_params.min_valid_points;

        std::cout << "Save to file? (y/n): ";
        std::cin >> choice;
        detect_params.save_to_file = (choice == 'y' || choice == 'Y');
    }

    // Create stream for depth data
    std::vector<StreamType> stream_types_vector;
    stream_types_vector.push_back(StreamType::kDepth);

    long frame_loop_times = -1; // Run indefinitely
    std::thread stream_thread = std::thread(DistanceDetectionStreamProcess,
        device,
        stream_types_vector,
        frame_loop_times);

    // Wait for stream thread to complete
    if (stream_thread.joinable()) {
        stream_thread.join();
    }

    device->Close();
    std::cout << "Distance detection completed." << std::endl;
    return 0;
}

int main() {
    std::cout << "Aurora 900 Distance Detection System" << std::endl;
    std::cout << "====================================" << std::endl;

    try {
        return PrepareDistanceDetection();
    }
    catch (const std::exception& e) {
        std::cout << "Exception: " << e.what() << std::endl;
        return -1;
    }
}