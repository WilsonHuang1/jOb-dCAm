// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

#include <libobsensor/ObSensor.hpp>
#include "libobsensor/hpp/Utils.hpp"
#include "libobsensor/hpp/Frame.hpp"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>

// Function declarations
void printCameraIntrinsics(const OBCameraIntrinsic& intrinsic, const std::string& streamName);
void printCameraDistortion(const OBCameraDistortion& distortion, const std::string& streamName);
void printExtrinsic(const OBExtrinsic& extrinsic, const std::string& fromStream, const std::string& toStream);
void printIMUIntrinsics(const OBAccelIntrinsic& accelIntrinsic, const OBGyroIntrinsic& gyroIntrinsic);
void saveParametersToFile(const std::string& filename, 
                         const OBCameraIntrinsic& depthIntrinsic, const OBCameraDistortion& depthDistortion,
                         const OBCameraIntrinsic& colorIntrinsic, const OBCameraDistortion& colorDistortion,
                         const OBExtrinsic& depthToColor, const OBExtrinsic& colorToDepth,
                         const OBAccelIntrinsic& accelIntrinsic, const OBGyroIntrinsic& gyroIntrinsic,
                         const OBExtrinsic& depthToAccel, const OBExtrinsic& depthToGyro,
                         bool hasIMU);

int main(void) try {
    std::cout << "=== Orbbec Camera and IMU Parameter Extractor ===" << std::endl;
    std::cout << "Based on OrbbecSDK v2.4.8" << std::endl;
    std::cout << "=================================================" << std::endl;

    // Create context and get device list
    auto context = std::make_shared<ob::Context>();
    auto deviceList = context->queryDeviceList();
    
    if (deviceList->getCount() == 0) {
        std::cerr << "No Orbbec device connected!" << std::endl;
        return -1;
    }

    auto device = deviceList->getDevice(0);
    std::cout << "Device found: " << device->getDeviceInfo()->getName() << std::endl;
    std::cout << "Serial Number: " << device->getDeviceInfo()->getSerialNumber() << std::endl;
    std::cout << "Firmware Version: " << device->getDeviceInfo()->getFirmwareVersion() << std::endl;
    std::cout << "=================================================" << std::endl;

    // Configure pipeline for depth and color streams
    auto config = std::make_shared<ob::Config>();
    
    // Enable depth stream
    config->enableVideoStream(OB_STREAM_DEPTH, 848, 480, 60, OB_FORMAT_Y16);
    
    // Enable color stream
    config->enableVideoStream(OB_STREAM_COLOR, 848, 480, 60, OB_FORMAT_RGB);

    // Try to enable IMU streams (accelerometer and gyroscope)
    bool hasIMU = false;
    try {
        auto sensorList = device->getSensorList();
        for(uint32_t i = 0; i < sensorList->getCount(); i++) {
            auto sensorType = sensorList->getSensorType(i);
            if(sensorType == OB_SENSOR_ACCEL || sensorType == OB_SENSOR_GYRO) {
                config->enableStream(sensorType);
                hasIMU = true;
            }
        }
        hasIMU = true;
        std::cout << "IMU streams enabled successfully" << std::endl;
    } catch (ob::Error &e) {
        std::cout << "IMU not supported or failed to enable: " << e.what() << std::endl;
        hasIMU = false;
    }

    // Set frame aggregate output mode
    config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);

    // Create and start pipeline
    auto pipe = std::make_shared<ob::Pipeline>();
    pipe->start(config);

    std::cout << "Pipeline started, waiting for frames..." << std::endl;

    // Wait for a valid frameset
    std::shared_ptr<ob::FrameSet> frameSet = nullptr;
    int attempts = 0;
    while (frameSet == nullptr && attempts < 50) {
        frameSet = pipe->waitForFrameset(1000);
        attempts++;
        if (frameSet == nullptr) {
            std::cout << "Waiting for frames... (attempt " << attempts << "/50)" << std::endl;
        }
    }

    if (frameSet == nullptr) {
        std::cerr << "Failed to get frames from device" << std::endl;
        pipe->stop();
        return -1;
    }

    std::cout << "Frames received, extracting parameters..." << std::endl;
    std::cout << "=================================================" << std::endl;

    // Get depth and color frames
    auto depthFrame = frameSet->getFrame(OB_FRAME_DEPTH);
    auto colorFrame = frameSet->getFrame(OB_FRAME_COLOR);

    if (!depthFrame || !colorFrame) {
        std::cerr << "Failed to get depth or color frame" << std::endl;
        pipe->stop();
        return -1;
    }

    // Get stream profiles
    auto depthProfile = depthFrame->getStreamProfile()->as<ob::VideoStreamProfile>();
    auto colorProfile = colorFrame->getStreamProfile()->as<ob::VideoStreamProfile>();

    // Extract camera intrinsic parameters
    std::cout << "\n=== CAMERA INTRINSIC PARAMETERS ===" << std::endl;
    auto depthIntrinsic = depthProfile->getIntrinsic();
    auto colorIntrinsic = colorProfile->getIntrinsic();
    
    printCameraIntrinsics(depthIntrinsic, "Depth");
    printCameraIntrinsics(colorIntrinsic, "Color");

    // Verify we got the requested resolution
    std::cout << "Actual Depth Resolution: " << depthIntrinsic.width << "x" << depthIntrinsic.height << std::endl;
    std::cout << "Actual Color Resolution: " << colorIntrinsic.width << "x" << colorIntrinsic.height << std::endl;

    if (depthIntrinsic.width != 848 || depthIntrinsic.height != 480) {
        std::cout << "Warning: Depth stream resolution differs from requested 848x480" << std::endl;
    }
    if (colorIntrinsic.width != 848 || colorIntrinsic.height != 480) {
        std::cout << "Warning: Color stream resolution differs from requested 848x480" << std::endl;
    }

    // Extract camera distortion parameters
    std::cout << "\n=== CAMERA DISTORTION PARAMETERS ===" << std::endl;
    auto depthDistortion = depthProfile->getDistortion();
    auto colorDistortion = colorProfile->getDistortion();
    
    printCameraDistortion(depthDistortion, "Depth");
    printCameraDistortion(colorDistortion, "Color");

    // Extract extrinsic parameters between cameras
    std::cout << "\n=== CAMERA EXTRINSIC PARAMETERS ===" << std::endl;
    auto depthToColorExtrinsic = depthProfile->getExtrinsicTo(colorProfile);
    auto colorToDepthExtrinsic = colorProfile->getExtrinsicTo(depthProfile);
    
    printExtrinsic(depthToColorExtrinsic, "Depth", "Color");
    printExtrinsic(colorToDepthExtrinsic, "Color", "Depth");

    // Initialize IMU variables
    OBAccelIntrinsic accelIntrinsic = {};
    OBGyroIntrinsic gyroIntrinsic = {};
    OBExtrinsic depthToAccelExtrinsic = {};
    OBExtrinsic depthToGyroExtrinsic = {};

    // Extract IMU parameters if available
    if (hasIMU) {
        std::cout << "\n=== IMU PARAMETERS ===" << std::endl;
        
        // Try to get IMU frames
        auto accelFrame = frameSet->getFrame(OB_FRAME_ACCEL);
        auto gyroFrame = frameSet->getFrame(OB_FRAME_GYRO);

        if (accelFrame && gyroFrame) {
            // Get IMU stream profiles
            auto accelStreamProfile = accelFrame->getStreamProfile()->as<ob::AccelStreamProfile>();
            auto gyroStreamProfile = gyroFrame->getStreamProfile()->as<ob::GyroStreamProfile>();

            // Get IMU intrinsic parameters
            accelIntrinsic = accelStreamProfile->getIntrinsic();
            gyroIntrinsic = gyroStreamProfile->getIntrinsic();
            
            printIMUIntrinsics(accelIntrinsic, gyroIntrinsic);

            // Get extrinsic parameters between depth camera and IMU
            std::cout << "\n=== CAMERA-IMU EXTRINSIC PARAMETERS ===" << std::endl;
            try {
                depthToAccelExtrinsic = depthProfile->getExtrinsicTo(accelStreamProfile);
                printExtrinsic(depthToAccelExtrinsic, "Depth", "Accelerometer");
            } catch (ob::Error &e) {
                std::cout << "Failed to get depth to accelerometer extrinsic: " << e.what() << std::endl;
            }

            try {
                depthToGyroExtrinsic = depthProfile->getExtrinsicTo(gyroStreamProfile);
                printExtrinsic(depthToGyroExtrinsic, "Depth", "Gyroscope");
            } catch (ob::Error &e) {
                std::cout << "Failed to get depth to gyroscope extrinsic: " << e.what() << std::endl;
            }
        } else {
            std::cout << "IMU frames not available in this frameset" << std::endl;
            hasIMU = false;
        }
    }

    // Save parameters to file
    std::cout << "\n=== SAVING PARAMETERS ===" << std::endl;
   std::string filename = "orbbec_camera_imu_parameters_848x480_60hz.txt";
    saveParametersToFile(filename, depthIntrinsic, depthDistortion, 
                        colorIntrinsic, colorDistortion,
                        depthToColorExtrinsic, colorToDepthExtrinsic,
                        accelIntrinsic, gyroIntrinsic,
                        depthToAccelExtrinsic, depthToGyroExtrinsic,
                        hasIMU);
    
    std::cout << "Parameters saved to: " << filename << std::endl;

    // Stop pipeline
    pipe->stop();
    std::cout << "\nParameter extraction completed successfully!" << std::endl;
    
    return 0;
}
catch(ob::Error &e) {
    std::cerr << "OrbbecSDK Error:" << std::endl;
    std::cerr << "Function: " << e.getFunction() << std::endl;
    std::cerr << "Args: " << e.getArgs() << std::endl;
    std::cerr << "Message: " << e.what() << std::endl;
    std::cerr << "Type: " << e.getExceptionType() << std::endl;
    return EXIT_FAILURE;
}
catch(std::exception &e) {
    std::cerr << "Standard Error: " << e.what() << std::endl;
    return EXIT_FAILURE;
}

void printCameraIntrinsics(const OBCameraIntrinsic& intrinsic, const std::string& streamName) {
    std::cout << "\n" << streamName << " Camera Intrinsics:" << std::endl;
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "  Focal Length (fx, fy): (" << intrinsic.fx << ", " << intrinsic.fy << ")" << std::endl;
    std::cout << "  Principal Point (cx, cy): (" << intrinsic.cx << ", " << intrinsic.cy << ")" << std::endl;
    std::cout << "  Image Resolution (width, height): (" << intrinsic.width << ", " << intrinsic.height << ")" << std::endl;
}

void printCameraDistortion(const OBCameraDistortion& distortion, const std::string& streamName) {
    std::cout << "\n" << streamName << " Camera Distortion:" << std::endl;
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "  Model: ";
    switch(distortion.model) {
        case OB_DISTORTION_BROWN_CONRADY:
            std::cout << "Brown-Conrady";
            break;
        case OB_DISTORTION_INVERSE_BROWN_CONRADY:
            std::cout << "Inverse Brown-Conrady";
            break;
        case OB_DISTORTION_KANNALA_BRANDT4:
            std::cout << "Kannala-Brandt4";
            break;
        case OB_DISTORTION_MODIFIED_BROWN_CONRADY:
            std::cout << "Modified Brown-Conrady";
            break;
        case OB_DISTORTION_BROWN_CONRADY_K6:
            std::cout << "Brown-Conrady K6";
            break;
        default:
            std::cout << "Unknown (" << distortion.model << ")";
            break;
    }
    std::cout << std::endl;
    std::cout << "  Coefficients:" << std::endl;
    std::cout << "    k1=" << distortion.k1 << ", k2=" << distortion.k2 << ", k3=" << distortion.k3 << std::endl;
    std::cout << "    k4=" << distortion.k4 << ", k5=" << distortion.k5 << ", k6=" << distortion.k6 << std::endl;
    std::cout << "    p1=" << distortion.p1 << ", p2=" << distortion.p2 << std::endl;
}

void printExtrinsic(const OBExtrinsic& extrinsic, const std::string& fromStream, const std::string& toStream) {
    std::cout << "\n" << fromStream << " to " << toStream << " Extrinsics:" << std::endl;
    std::cout << std::fixed << std::setprecision(6);
    
    std::cout << "  Rotation Matrix:" << std::endl;
    for(int i = 0; i < 3; i++) {
        std::cout << "    [";
        for(int j = 0; j < 3; j++) {
            std::cout << std::setw(10) << extrinsic.rot[i*3 + j];
            if(j < 2) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }
    
    std::cout << "  Translation Vector (x, y, z): [";
    std::cout << extrinsic.trans[0] << ", " << extrinsic.trans[1] << ", " << extrinsic.trans[2];
    std::cout << "] (mm)" << std::endl;
}

void printIMUIntrinsics(const OBAccelIntrinsic& accelIntrinsic, const OBGyroIntrinsic& gyroIntrinsic) {
    std::cout << "\nAccelerometer Intrinsics:" << std::endl;
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "  Noise Density: " << accelIntrinsic.noiseDensity << std::endl;
    std::cout << "  Random Walk: " << accelIntrinsic.randomWalk << std::endl;
    std::cout << "  Reference Temperature: " << accelIntrinsic.referenceTemp << std::endl;
    std::cout << "  Bias [x, y, z]: [";
    std::cout << accelIntrinsic.bias[0] << ", ";
    std::cout << accelIntrinsic.bias[1] << ", ";
    std::cout << accelIntrinsic.bias[2] << "]" << std::endl;
    
    std::cout << "  Gravity [x, y, z]: [";
    std::cout << accelIntrinsic.gravity[0] << ", ";
    std::cout << accelIntrinsic.gravity[1] << ", ";
    std::cout << accelIntrinsic.gravity[2] << "]" << std::endl;

    std::cout << "\nGyroscope Intrinsics:" << std::endl;
    std::cout << "  Noise Density: " << gyroIntrinsic.noiseDensity << std::endl;
    std::cout << "  Random Walk: " << gyroIntrinsic.randomWalk << std::endl;
    std::cout << "  Reference Temperature: " << gyroIntrinsic.referenceTemp << std::endl;
    std::cout << "  Bias [x, y, z]: [";
    std::cout << gyroIntrinsic.bias[0] << ", ";
    std::cout << gyroIntrinsic.bias[1] << ", ";
    std::cout << gyroIntrinsic.bias[2] << "]" << std::endl;
}

void saveParametersToFile(const std::string& filename, 
                         const OBCameraIntrinsic& depthIntrinsic, const OBCameraDistortion& depthDistortion,
                         const OBCameraIntrinsic& colorIntrinsic, const OBCameraDistortion& colorDistortion,
                         const OBExtrinsic& depthToColor, const OBExtrinsic& colorToDepth,
                         const OBAccelIntrinsic& accelIntrinsic, const OBGyroIntrinsic& gyroIntrinsic,
                         const OBExtrinsic& depthToAccel, const OBExtrinsic& depthToGyro,
                         bool hasIMU) {
    
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
        return;
    }

    file << "# Orbbec Camera and IMU Parameters - 848x480 @ 60Hz" << std::endl;
    file << "# Generated by OrbbecSDK Parameter Extractor" << std::endl;
    file << "# Based on OrbbecSDK v2.4.8" << std::endl;
    file << "# Target Resolution: 848x480 pixels at 60 FPS" << std::endl;
    file << std::endl;

    // Depth camera parameters
    file << "[DEPTH_CAMERA_INTRINSICS]" << std::endl;
    file << "fx=" << depthIntrinsic.fx << std::endl;
    file << "fy=" << depthIntrinsic.fy << std::endl;
    file << "cx=" << depthIntrinsic.cx << std::endl;
    file << "cy=" << depthIntrinsic.cy << std::endl;
    file << "width=" << depthIntrinsic.width << std::endl;
    file << "height=" << depthIntrinsic.height << std::endl;
    file << std::endl;

    file << "[DEPTH_CAMERA_DISTORTION]" << std::endl;
    file << "model=" << depthDistortion.model << std::endl;
    file << "k1=" << depthDistortion.k1 << std::endl;
    file << "k2=" << depthDistortion.k2 << std::endl;
    file << "k3=" << depthDistortion.k3 << std::endl;
    file << "k4=" << depthDistortion.k4 << std::endl;
    file << "k5=" << depthDistortion.k5 << std::endl;
    file << "k6=" << depthDistortion.k6 << std::endl;
    file << "p1=" << depthDistortion.p1 << std::endl;
    file << "p2=" << depthDistortion.p2 << std::endl;
    file << std::endl;

    // Color camera parameters
    file << "[COLOR_CAMERA_INTRINSICS]" << std::endl;
    file << "fx=" << colorIntrinsic.fx << std::endl;
    file << "fy=" << colorIntrinsic.fy << std::endl;
    file << "cx=" << colorIntrinsic.cx << std::endl;
    file << "cy=" << colorIntrinsic.cy << std::endl;
    file << "width=" << colorIntrinsic.width << std::endl;
    file << "height=" << colorIntrinsic.height << std::endl;
    file << std::endl;

    file << "[COLOR_CAMERA_DISTORTION]" << std::endl;
    file << "model=" << colorDistortion.model << std::endl;
    file << "k1=" << colorDistortion.k1 << std::endl;
    file << "k2=" << colorDistortion.k2 << std::endl;
    file << "k3=" << colorDistortion.k3 << std::endl;
    file << "k4=" << colorDistortion.k4 << std::endl;
    file << "k5=" << colorDistortion.k5 << std::endl;
    file << "k6=" << colorDistortion.k6 << std::endl;
    file << "p1=" << colorDistortion.p1 << std::endl;
    file << "p2=" << colorDistortion.p2 << std::endl;
    file << std::endl;

    // Extrinsic parameters
    file << "[DEPTH_TO_COLOR_EXTRINSICS]" << std::endl;
    file << "# Rotation matrix (row-major)" << std::endl;
    for(int i = 0; i < 9; i++) {
        file << "r" << i << "=" << depthToColor.rot[i] << std::endl;
    }
    file << "# Translation vector (mm)" << std::endl;
    file << "tx=" << depthToColor.trans[0] << std::endl;
    file << "ty=" << depthToColor.trans[1] << std::endl;
    file << "tz=" << depthToColor.trans[2] << std::endl;
    file << std::endl;

    // IMU parameters if available
    if (hasIMU) {
        file << "[ACCELEROMETER_INTRINSICS]" << std::endl;
        file << "noise_density=" << accelIntrinsic.noiseDensity << std::endl;
        file << "random_walk=" << accelIntrinsic.randomWalk << std::endl;
        file << "reference_temp=" << accelIntrinsic.referenceTemp << std::endl;
        file << "bias_x=" << accelIntrinsic.bias[0] << std::endl;
        file << "bias_y=" << accelIntrinsic.bias[1] << std::endl;
        file << "bias_z=" << accelIntrinsic.bias[2] << std::endl;
        file << "gravity_x=" << accelIntrinsic.gravity[0] << std::endl;
        file << "gravity_y=" << accelIntrinsic.gravity[1] << std::endl;
        file << "gravity_z=" << accelIntrinsic.gravity[2] << std::endl;
        file << std::endl;

        file << "[GYROSCOPE_INTRINSICS]" << std::endl;
        file << "noise_density=" << gyroIntrinsic.noiseDensity << std::endl;
        file << "random_walk=" << gyroIntrinsic.randomWalk << std::endl;
        file << "reference_temp=" << gyroIntrinsic.referenceTemp << std::endl;
        file << "bias_x=" << gyroIntrinsic.bias[0] << std::endl;
        file << "bias_y=" << gyroIntrinsic.bias[1] << std::endl;
        file << "bias_z=" << gyroIntrinsic.bias[2] << std::endl;
        file << std::endl;

        file << "[DEPTH_TO_ACCELEROMETER_EXTRINSICS]" << std::endl;
        file << "# Rotation matrix (row-major)" << std::endl;
        for(int i = 0; i < 9; i++) {
            file << "r" << i << "=" << depthToAccel.rot[i] << std::endl;
        }
        file << "# Translation vector (mm)" << std::endl;
        file << "tx=" << depthToAccel.trans[0] << std::endl;
        file << "ty=" << depthToAccel.trans[1] << std::endl;
        file << "tz=" << depthToAccel.trans[2] << std::endl;
        file << std::endl;

        file << "[DEPTH_TO_GYROSCOPE_EXTRINSICS]" << std::endl;
        file << "# Rotation matrix (row-major)" << std::endl;
        for(int i = 0; i < 9; i++) {
            file << "r" << i << "=" << depthToGyro.rot[i] << std::endl;
        }
        file << "# Translation vector (mm)" << std::endl;
        file << "tx=" << depthToGyro.trans[0] << std::endl;
        file << "ty=" << depthToGyro.trans[1] << std::endl;
        file << "tz=" << depthToGyro.trans[2] << std::endl;
    }

    file.close();
}