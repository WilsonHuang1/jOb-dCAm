// Simple, crash-safe version based on working sample_viewer.cc
#include <iostream>
#include <thread>
#include <vector>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include "deptrum/device.h"
#include "deptrum/stream.h"
#include "deptrum/aurora900_series.h"
#include "functional/base.h"
#include "functional/frame_rate_helper.h"
#include "sample_helper.h"
#include "viewer_helper.hpp"

using namespace std;
using namespace deptrum;
using namespace deptrum::stream;

// Global variables
bool is_running = true;
bool is_print_fps = false;
std::shared_ptr<ViewerHelper> viewer_helper;
CameraParam g_camera_param;

// Simple distance detection parameters
struct SimpleDistanceParams {
    float roi_x_ratio = 0.3f;
    float roi_y_ratio = 0.3f;
    uint16_t min_depth = 100;
    uint16_t max_depth = 3000;
    int min_valid_points = 50;
} simple_params;

// Simple distance calculation
struct SimpleDistance {
    float distance_mm;
    float center_x, center_y;
    int valid_points;
    
    SimpleDistance() : distance_mm(0), center_x(0), center_y(0), valid_points(0) {}
};

SimpleDistance calculateSimpleDistance(std::shared_ptr<StreamFrame> depth_frame) {
    SimpleDistance result;
    
    if (!depth_frame || depth_frame->frame_type != FrameType::kDepthFrame || !depth_frame->data) {
        return result;
    }
    
    uint16_t* depth_data = static_cast<uint16_t*>(depth_frame->data.get());
    int width = depth_frame->cols;
    int height = depth_frame->rows;
    
    // ROI bounds
    int roi_width = static_cast<int>(width * simple_params.roi_x_ratio);
    int roi_height = static_cast<int>(height * simple_params.roi_y_ratio);
    int roi_start_x = (width - roi_width) / 2;
    int roi_start_y = (height - roi_height) / 2;
    int roi_end_x = roi_start_x + roi_width;
    int roi_end_y = roi_start_y + roi_height;
    
    // Collect valid depth points
    std::vector<uint16_t> valid_depths;
    float sum_x = 0, sum_y = 0;
    
    for (int y = roi_start_y; y < roi_end_y; y++) {
        for (int x = roi_start_x; x < roi_end_x; x++) {
            int idx = y * width + x;
            if (idx < 0 || idx >= width * height) continue;
            
            uint16_t depth = depth_data[idx];
            if (depth >= simple_params.min_depth && depth <= simple_params.max_depth) {
                valid_depths.push_back(depth);
                sum_x += x;
                sum_y += y;
            }
        }
    }
    
    if (valid_depths.size() >= simple_params.min_valid_points) {
        std::sort(valid_depths.begin(), valid_depths.end());
        result.distance_mm = valid_depths[valid_depths.size() / 2];
        result.center_x = sum_x / valid_depths.size();
        result.center_y = sum_y / valid_depths.size();
        result.valid_points = valid_depths.size();
    }
    
    return result;
}

// Enhanced ShowFrame with distance overlay - based on viewer_helper.hpp
void ShowFrameWithDistance(const StreamFrames& frames, CameraParam camera_param = {}) {
    SimpleDistance detection;
    
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
                // Handle RGB frame with distance overlay
                cv::Mat rgb_display;
                
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
                
                if (!rgb_display.empty()) {
                    // Add ROI rectangle
                    int width = rgb_display.cols;
                    int height = rgb_display.rows;
                    int roi_width = static_cast<int>(width * simple_params.roi_x_ratio);
                    int roi_height = static_cast<int>(height * simple_params.roi_y_ratio);
                    int roi_start_x = (width - roi_width) / 2;
                    int roi_start_y = (height - roi_height) / 2;
                    
                    cv::rectangle(rgb_display, 
                                 cv::Point(roi_start_x, roi_start_y),
                                 cv::Point(roi_start_x + roi_width, roi_start_y + roi_height),
                                 cv::Scalar(0, 255, 0), 2);
                    
                    // Add distance info if available
                    if (detection.valid_points > 0) {
                        cv::circle(rgb_display, 
                                  cv::Point(static_cast<int>(detection.center_x), 
                                           static_cast<int>(detection.center_y)),
                                  8, cv::Scalar(0, 0, 255), -1);
                        
                        std::string distance_text = "Distance: " + std::to_string(static_cast<int>(detection.distance_mm)) + "mm";
                        cv::rectangle(rgb_display, cv::Point(10, 10), cv::Point(300, 50), cv::Scalar(0, 0, 0), -1);
                        cv::putText(rgb_display, distance_text, cv::Point(20, 35), 
                                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
                    }
                    
                    cv::imshow(title, rgb_display);
                }
                break;
            }
            
            case kIrFrame: {
                if (frame->data && frame->size != 0) {
                    cv::Mat ir_mat(frame->rows, frame->cols, CV_8UC1, frame->data.get());
                    cv::imshow(title, ir_mat);
                }
                break;
            }
            
            case kDepthFrame: {
                // Calculate distance for this depth frame
                detection = calculateSimpleDistance(frame);
                
                // Print distance info
                if (detection.valid_points > 0) {
                    std::cout << "\r[DISTANCE] " << std::setw(4) << static_cast<int>(detection.distance_mm) 
                              << "mm | Points: " << std::setw(4) << detection.valid_points 
                              << " | Center: (" << std::setw(3) << static_cast<int>(detection.center_x) 
                              << "," << std::setw(3) << static_cast<int>(detection.center_y) << ")" << std::flush;
                }
                
                // Display colored depth
                int size = frame->rows * frame->cols * 3;
                std::shared_ptr<uint8_t> colored_depth(new uint8_t[size], [](uint8_t* p) { delete[] p; });
                viewer_helper->DealWithDepthFrame(*frame, camera_param, colored_depth.get());
                cv::Mat depth_mat(frame->rows, frame->cols, CV_8UC3, colored_depth.get());
                
                // Add ROI and detection overlay to depth image
                int width = depth_mat.cols;
                int height = depth_mat.rows;
                int roi_width = static_cast<int>(width * simple_params.roi_x_ratio);
                int roi_height = static_cast<int>(height * simple_params.roi_y_ratio);
                int roi_start_x = (width - roi_width) / 2;
                int roi_start_y = (height - roi_height) / 2;
                
                cv::rectangle(depth_mat, 
                             cv::Point(roi_start_x, roi_start_y),
                             cv::Point(roi_start_x + roi_width, roi_start_y + roi_height),
                             cv::Scalar(0, 255, 0), 2);
                
                if (detection.valid_points > 0) {
                    cv::circle(depth_mat, 
                              cv::Point(static_cast<int>(detection.center_x), 
                                       static_cast<int>(detection.center_y)),
                              8, cv::Scalar(255, 255, 255), -1);
                }
                
                cv::imshow(title, depth_mat);
                break;
            }
        }
        cv::waitKey(1);
    }
}

// Simple stream process based on sample_viewer.cc
int SimpleRGBDStreamProcess(std::shared_ptr<Device> device,
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

    StreamFrames frames;
    FrameRateHelper frame_rate_helper;
    long cnt = 0;

    std::cout << "\n=== Simple RGBD Distance Detection Started ===" << std::endl;
    std::cout << "Press 'q' to quit, 'a' to toggle FPS display" << std::endl;

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
            std::cout << "FPS: " << frame_rate_helper.GetFrameRate() << std::endl;
        }

        // Show frames with distance detection
        ShowFrameWithDistance(frames, g_camera_param);
        
        // Handle keyboard input
        char key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 'Q') {
            is_running = false;
        } else if (key == 'a' || key == 'A') {
            is_print_fps = !is_print_fps;
            std::cout << "\nFPS display: " << (is_print_fps ? "ON" : "OFF") << std::endl;
        }
    }

    stream->Stop();
    device->DestroyStream(stream);
    return 0;
}

// Main function based on sample_viewer.cc
int PrepareSimpleRGBD() {
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
        std::cout << "Enabling aligned mode" << std::endl;
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
    }

    // Create stream - try different options
    std::vector<StreamType> stream_types_vector;
    std::thread stream_thread;
    
    // Get supported stream types
    std::vector<StreamType> supported_streams;
    device->GetSupportedStreamType(supported_streams);
    
    std::cout << "\nSupported stream types:" << std::endl;
    for (auto stream_type : supported_streams) {
        std::cout << "  Stream type: " << static_cast<int>(stream_type) << std::endl;
    }
    
    // Choose stream type (prioritize RGBD, fallback to separate streams)
    std::cout << "\nChoose stream mode:" << std::endl;
    std::cout << "1: RGBD (RGB + Depth combined)" << std::endl;
    std::cout << "2: RGB + Depth (separate streams)" << std::endl;
    std::cout << "3: Depth only" << std::endl;
    std::cout << "Enter choice (1-3): ";
    
    int choice;
    std::cin >> choice;
    
    switch (choice) {
        case 1:
            stream_types_vector.push_back(StreamType::kRgbd);
            std::cout << "Using RGBD stream" << std::endl;
            break;
        case 2:
            stream_types_vector.push_back(StreamType::kRgb);
            stream_types_vector.push_back(StreamType::kDepth);
            std::cout << "Using separate RGB and Depth streams" << std::endl;
            break;
        case 3:
            stream_types_vector.push_back(StreamType::kDepth);
            std::cout << "Using Depth stream only" << std::endl;
            break;
        default:
            stream_types_vector.push_back(StreamType::kDepth);
            std::cout << "Default: Using Depth stream only" << std::endl;
            break;
    }
    
    if (stream_types_vector.empty()) {
        std::cout << "No stream type selected!" << std::endl;
        return -1;
    }
    
    long frame_loop_times = -1;
    std::cout << "\nStarting stream..." << std::endl;
    stream_thread = std::thread(SimpleRGBDStreamProcess,
                                device,
                                stream_types_vector,
                                frame_loop_times,
                                device_name);

    // Wait for stream thread to complete
    if (stream_thread.joinable()) {
        stream_thread.join();
    }
    
    viewer_helper.reset();
    device->Close();
    std::cout << "Simple RGBD detection completed." << std::endl;
    return 0;
}

int main() {
    std::cout << "Simple Aurora 900 RGBD Distance Detection" << std::endl;
    std::cout << "========================================" << std::endl;
    
    try {
        return PrepareSimpleRGBD();
    } catch (const std::exception& e) {
        std::cout << "Exception: " << e.what() << std::endl;
        return -1;
    }
}