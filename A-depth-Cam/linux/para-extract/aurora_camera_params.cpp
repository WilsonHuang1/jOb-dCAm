#include <iostream>
#include <iomanip>
#include "deptrum/device.h"
#include "deptrum/aurora900_series.h"
#include "functional/base.h"

using namespace deptrum;
using namespace deptrum::stream;

int main(int argc, char** argv) {
    std::cout << "=== Aurora900 Camera Parameters Extractor ===" << std::endl;
    
    // Register device hotplug callback
    DeviceManager::GetInstance()->RegisterDeviceConnectedCallback();

    // Query the list of connected devices
    std::vector<DeviceInformation> device_list;
    int ret = DeviceManager::GetInstance()->GetDeviceList(device_list);
    CHECK_SDK_RETURN_VALUE(ret);
    
    // Check device count
    CHECK_DEVICE_COUNT(device_list.size());
    
    // Display available devices
    std::cout << "\nAvailable devices:" << std::endl;
    for (int index = 0; index < device_list.size(); index++) {
        std::cout << "Device " << index << ": model=" << device_list[index].model 
                  << " device_addr=" << device_list[index].ir_camera.device_addr
                  << " usb_port=" << device_list[index].ir_camera.port_path << std::endl;
    }

    // Select device (use first Aurora900 device or first device if only one)
    int dev_index = 0;
    if (device_list.size() > 1) {
        std::cout << "\nEnter device index (0-" << device_list.size() - 1 << "): ";
        std::cin >> dev_index;
    }
    std::cout << "Using device index: " << dev_index << std::endl;

    // Create device
    std::shared_ptr<Device> device = DeviceManager::GetInstance()->CreateDevice(device_list[dev_index]);
    CHECK_DEVICE_VALID(device);

    // Print SDK version
    auto sdk_version = device->GetSdkVersion();
    std::cout << "SDK version: " << sdk_version << std::endl;

    // Open device
    ret = device->Open();
    CHECK_SDK_RETURN_VALUE(ret);
    std::cout << "Device opened successfully" << std::endl;

    // Get device name
    std::string device_name = device->GetDeviceName();
    std::cout << "Device name: " << device_name << std::endl;

    // Check if it's Aurora900 series device
    if (device_name == "Aurora930" || device_name == "Aurora931" || 
        device_name == "Aurora932" || device_name == "Aurora936") {
        
        std::cout << "\n=== Aurora900 Device Detected ===" << std::endl;
        
        // Cast to Aurora900 device
        std::shared_ptr<Aurora900> aurora_device = std::dynamic_pointer_cast<Aurora900>(device);
        if (!aurora_device) {
            std::cerr << "Failed to cast to Aurora900 device!" << std::endl;
            return -1;
        }

        // Set supported frame modes
        FrameMode ir_mode, rgb_mode, depth_mode;
        std::vector<std::tuple<FrameMode, FrameMode, FrameMode>> device_resolution_vec;
        ret = device->GetSupportedFrameMode(device_resolution_vec);
        CHECK_SDK_RETURN_VALUE(ret);
        
        // Use first supported mode
        ir_mode = std::get<0>(device_resolution_vec[0]);
        rgb_mode = std::get<1>(device_resolution_vec[0]);
        depth_mode = std::get<2>(device_resolution_vec[0]);
        ret = device->SetMode(ir_mode, rgb_mode, depth_mode);
        CHECK_SDK_RETURN_VALUE(ret);
        std::cout << "Frame modes set successfully" << std::endl;

        // Get device firmware information
        DeviceDescription device_info;
        ret = aurora_device->GetDeviceInfo(device_info);
        if (ret == 0) {
            std::cout << "\n=== Device Information ===" << std::endl;
            std::cout << "Device name: " << device_info.device_name << std::endl;
            std::cout << "Serial number: " << device_info.serial_num << std::endl;
            std::cout << "SDK version: " << device_info.stream_sdk_version << std::endl;
            std::cout << "RGB firmware version: " << device_info.rgb_firmware_version << std::endl;
            std::cout << "IR firmware version: " << device_info.ir_firmware_version << std::endl;
            std::cout << "PID: 0x" << std::hex << device_info.pid << std::dec << std::endl;
            std::cout << "VID: 0x" << std::hex << device_info.vid << std::dec << std::endl;
        }

        // *** EXTRACT CAMERA PARAMETERS ***
        Intrinsic ir_intri, rgb_intri;
        Extrinsic extrinsic;
        
        std::cout << "\n=== Extracting Camera Parameters ===" << std::endl;
        ret = device->GetCameraParameters(ir_intri, rgb_intri, extrinsic);
        CHECK_SDK_RETURN_VALUE(ret);
        std::cout << "Camera parameters extracted successfully!" << std::endl;

        // Display IR Camera Intrinsic Parameters
        std::cout << "\n=== IR Camera Intrinsic Parameters ===" << std::endl;
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "Resolution: " << ir_intri.cols << " x " << ir_intri.rows << std::endl;
        std::cout << "Focal Length (fx, fy): (" << ir_intri.focal_length[0] << ", " << ir_intri.focal_length[1] << ")" << std::endl;
        std::cout << "Principal Point (cx, cy): (" << ir_intri.principal_point[0] << ", " << ir_intri.principal_point[1] << ")" << std::endl;
        std::cout << "Distortion Coefficients: [";
        for (int i = 0; i < 5; i++) {
            std::cout << ir_intri.distortion_coeffs[i];
            if (i < 4) std::cout << ", ";
        }
        std::cout << "]" << std::endl;

        // Display RGB Camera Intrinsic Parameters
        std::cout << "\n=== RGB Camera Intrinsic Parameters ===" << std::endl;
        std::cout << "Resolution: " << rgb_intri.cols << " x " << rgb_intri.rows << std::endl;
        std::cout << "Focal Length (fx, fy): (" << rgb_intri.focal_length[0] << ", " << rgb_intri.focal_length[1] << ")" << std::endl;
        std::cout << "Principal Point (cx, cy): (" << rgb_intri.principal_point[0] << ", " << rgb_intri.principal_point[1] << ")" << std::endl;
        std::cout << "Distortion Coefficients: [";
        for (int i = 0; i < 5; i++) {
            std::cout << rgb_intri.distortion_coeffs[i];
            if (i < 4) std::cout << ", ";
        }
        std::cout << "]" << std::endl;

        // Display Extrinsic Parameters (between IR and RGB cameras)
        std::cout << "\n=== Extrinsic Parameters (IR to RGB transformation) ===" << std::endl;
        std::cout << "Rotation Matrix:" << std::endl;
        for (int i = 0; i < 3; i++) {
            std::cout << "[";
            for (int j = 0; j < 3; j++) {
                std::cout << std::setw(8) << std::fixed << std::setprecision(4) << extrinsic.rotation_matrix[i*3 + j];
                if (j < 2) std::cout << ", ";
            }
            std::cout << "]" << std::endl;
        }
        std::cout << "Translation Vector: [" 
                  << std::setprecision(4) << extrinsic.translation_vector[0] << ", "
                  << extrinsic.translation_vector[1] << ", "
                  << extrinsic.translation_vector[2] << "]" << std::endl;

        // Store individual parameters (as in the sample)
        float ir_fx = ir_intri.focal_length[0];
        float ir_fy = ir_intri.focal_length[1];
        float ir_cx = ir_intri.principal_point[0];
        float ir_cy = ir_intri.principal_point[1];
        
        float rgb_fx = rgb_intri.focal_length[0];
        float rgb_fy = rgb_intri.focal_length[1];
        float rgb_cx = rgb_intri.principal_point[0];
        float rgb_cy = rgb_intri.principal_point[1];

        std::cout << "\n=== Summary ===" << std::endl;
        std::cout << "IR Camera: fx=" << ir_fx << ", fy=" << ir_fy << ", cx=" << ir_cx << ", cy=" << ir_cy << std::endl;
        std::cout << "RGB Camera: fx=" << rgb_fx << ", fy=" << rgb_fy << ", cx=" << rgb_cx << ", cy=" << rgb_cy << std::endl;

    } else {
        std::cout << "Warning: Device '" << device_name << "' is not an Aurora900 series device!" << std::endl;
        std::cout << "This program is specifically designed for Aurora900 series cameras." << std::endl;
        
        // Still try to extract parameters for other devices
        std::cout << "\nAttempting to extract camera parameters anyway..." << std::endl;
        Intrinsic ir_intri, rgb_intri;
        Extrinsic extrinsic;
        
        ret = device->GetCameraParameters(ir_intri, rgb_intri, extrinsic);
        if (ret == 0) {
            std::cout << "Camera parameters extracted successfully!" << std::endl;
            std::cout << "IR Camera: fx=" << ir_intri.focal_length[0] << ", fy=" << ir_intri.focal_length[1] 
                      << ", cx=" << ir_intri.principal_point[0] << ", cy=" << ir_intri.principal_point[1] << std::endl;
            std::cout << "RGB Camera: fx=" << rgb_intri.focal_length[0] << ", fy=" << rgb_intri.focal_length[1] 
                      << ", cx=" << rgb_intri.principal_point[0] << ", cy=" << rgb_intri.principal_point[1] << std::endl;
        } else {
            std::cout << "Failed to extract camera parameters. Error code: 0x" << std::hex << ret << std::dec << std::endl;
        }
    }

    // Close device
    std::cout << "\nClosing device..." << std::endl;
    ret = device->Close();
    CHECK_SDK_RETURN_VALUE(ret);
    std::cout << "Device closed successfully" << std::endl;

    std::cout << "\n=== Program completed ===" << std::endl;
    return 0;
}