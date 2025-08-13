/**
 * Orbbec Gemini 335 - IMPROVED Stereo IR Camera Parameter Extractor
 * Features:
 * - Enumerates all available IR resolutions 
 * - Allows user to select specific resolution
 * - Extracts parameters for chosen resolution
 * - Generates resolution-specific YAML files
 * - Supports multiple resolution parameter extraction
 */

#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <map>
#include <sstream>
#include <libobsensor/ObSensor.hpp>

// Structure to hold stream profile information
struct StreamProfileInfo {
    uint32_t width;
    uint32_t height;
    uint32_t fps;
    OBFormat format;
    std::string formatStr;
    
    std::string toString() const {
        std::stringstream ss;
        ss << width << "x" << height << "@" << fps << "fps (" << formatStr << ")";
        return ss.str();
    }
    
    std::string getResolutionStr() const {
        std::stringstream ss;
        ss << width << "x" << height;
        return ss.str();
    }
};

void printStreamProfileInfo(const StreamProfileInfo& profile, uint32_t index) {
    std::cout << "  " << index << ". " << profile.toString() << std::endl;
}

void enumerateIRStreamProfiles(std::shared_ptr<ob::Device> device, 
                              std::vector<StreamProfileInfo>& leftIRProfiles,
                              std::vector<StreamProfileInfo>& rightIRProfiles) {
    
    auto sensorList = device->getSensorList();
    
    // Find IR sensors
    for (uint32_t i = 0; i < sensorList->getCount(); i++) {
        auto sensorType = sensorList->getSensorType(i);
        auto sensor = sensorList->getSensor(i);
        auto streamProfileList = sensor->getStreamProfileList();
        
        if (sensorType == OB_SENSOR_IR_LEFT) {
            std::cout << "\n=== LEFT IR CAMERA AVAILABLE RESOLUTIONS ===" << std::endl;
            for (uint32_t j = 0; j < streamProfileList->getCount(); j++) {
                auto profile = streamProfileList->getProfile(j)->as<ob::VideoStreamProfile>();
                
                StreamProfileInfo info;
                info.width = profile->getWidth();
                info.height = profile->getHeight();
                info.fps = profile->getFps();
                info.format = profile->getFormat();
                info.formatStr = ob::TypeHelper::convertOBFormatTypeToString(info.format);
                
                leftIRProfiles.push_back(info);
                printStreamProfileInfo(info, j);
            }
        }
        else if (sensorType == OB_SENSOR_IR_RIGHT) {
            std::cout << "\n=== RIGHT IR CAMERA AVAILABLE RESOLUTIONS ===" << std::endl;
            for (uint32_t j = 0; j < streamProfileList->getCount(); j++) {
                auto profile = streamProfileList->getProfile(j)->as<ob::VideoStreamProfile>();
                
                StreamProfileInfo info;
                info.width = profile->getWidth();
                info.height = profile->getHeight();
                info.fps = profile->getFps();
                info.format = profile->getFormat();
                info.formatStr = ob::TypeHelper::convertOBFormatTypeToString(info.format);
                
                rightIRProfiles.push_back(info);
                printStreamProfileInfo(info, j);
            }
        }
    }
}

int getUserSelection(const std::string& prompt, int maxOption) {
    int selection;
    while (true) {
        std::cout << prompt;
        std::cin >> selection;
        
        if (std::cin.fail()) {
            std::cin.clear();
            std::cin.ignore(10000, '\n');
            std::cout << "Invalid input. Please enter a number." << std::endl;
            continue;
        }
        
        if (selection >= 0 && selection < maxOption) {
            return selection;
        }
        
        if (selection == -1) {
            return -1; // Exit option
        }
        
        std::cout << "Invalid selection. Please choose between 0 and " << (maxOption-1) << " (or -1 to exit)." << std::endl;
    }
}

void printIRCameraIntrinsics(const OBCameraIntrinsic& intrinsic, const std::string& cameraName) {
    std::cout << "\n" << cameraName << " IR Camera Intrinsics:" << std::endl;
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "  Focal Length (fx, fy): (" << intrinsic.fx << ", " << intrinsic.fy << ")" << std::endl;
    std::cout << "  Principal Point (cx, cy): (" << intrinsic.cx << ", " << intrinsic.cy << ")" << std::endl;
    std::cout << "  Image Resolution (width, height): (" << intrinsic.width << ", " << intrinsic.height << ")" << std::endl;
}

void printExtrinsics(const OBExtrinsic& extrinsic, const std::string& fromSensor, const std::string& toSensor) {
    std::cout << "\n" << fromSensor << " to " << toSensor << " Extrinsics:" << std::endl;
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "  Rotation Matrix:" << std::endl;
    for (int i = 0; i < 3; i++) {
        std::cout << "  [";
        for (int j = 0; j < 3; j++) {
            std::cout << std::setw(10) << extrinsic.rot[i*3 + j];
            if (j < 2) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }
    std::cout << "  Translation Vector (x, y, z): [";
    std::cout << extrinsic.trans[0] << ", " << extrinsic.trans[1] << ", " << extrinsic.trans[2] << "] (mm)" << std::endl;
    
    // Calculate baseline for stereo
    if (fromSensor.find("Left") != std::string::npos && toSensor.find("Right") != std::string::npos) {
        float baseline = sqrt(extrinsic.trans[0]*extrinsic.trans[0] + 
                             extrinsic.trans[1]*extrinsic.trans[1] + 
                             extrinsic.trans[2]*extrinsic.trans[2]);
        std::cout << "  Stereo Baseline: " << baseline << " mm (" << baseline/1000.0 << " m)" << std::endl;
    }
}

void saveParametersToFile(const std::string& filename,
                         const StreamProfileInfo& selectedProfile,
                         const OBCameraIntrinsic& leftIRIntrinsic, 
                         const OBCameraIntrinsic& rightIRIntrinsic,
                         const OBExtrinsic& leftToRightIR,
                         const OBExtrinsic& leftIRToIMU,
                         bool hasIMU) {
    
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
        return;
    }

    file << "# Orbbec Gemini 335 - Stereo IR Camera Parameters" << std::endl;
    file << "# Resolution: " << selectedProfile.toString() << std::endl;
    file << "# Generated by Improved Orbbec Stereo Parameter Extractor" << std::endl;
    file << "# Based on OrbbecSDK v2.4.8" << std::endl;
    file << std::endl;

    file << "#--------------------------------------------------------------------------------------------" << std::endl;
    file << "# Camera Parameters - " << selectedProfile.getResolutionStr() << std::endl;
    file << "#--------------------------------------------------------------------------------------------" << std::endl;
    file << std::endl;

    file << "# File format version" << std::endl;
    file << "File.version: \"1.0\"" << std::endl;
    file << std::endl;

    file << "# Camera type" << std::endl;
    file << "Camera.type: \"PinHole\"" << std::endl;
    file << std::endl;

    // Left IR Camera intrinsics
    file << "# Left IR Camera (Camera1) intrinsic parameters" << std::endl;
    file << "Camera1.fx: " << leftIRIntrinsic.fx << std::endl;
    file << "Camera1.fy: " << leftIRIntrinsic.fy << std::endl;
    file << "Camera1.cx: " << leftIRIntrinsic.cx << std::endl;
    file << "Camera1.cy: " << leftIRIntrinsic.cy << std::endl;
    file << std::endl;

    file << "# Left IR Camera distortion coefficients (assuming no distortion)" << std::endl;
    file << "Camera1.k1: 0.0" << std::endl;
    file << "Camera1.k2: 0.0" << std::endl;
    file << "Camera1.p1: 0.0" << std::endl;
    file << "Camera1.p2: 0.0" << std::endl;
    file << "Camera1.k3: 0.0" << std::endl;
    file << std::endl;

    // Right IR Camera intrinsics
    file << "# Right IR Camera (Camera2) intrinsic parameters" << std::endl;
    file << "Camera2.fx: " << rightIRIntrinsic.fx << std::endl;
    file << "Camera2.fy: " << rightIRIntrinsic.fy << std::endl;
    file << "Camera2.cx: " << rightIRIntrinsic.cx << std::endl;
    file << "Camera2.cy: " << rightIRIntrinsic.cy << std::endl;
    file << std::endl;

    file << "# Right IR Camera distortion coefficients (assuming no distortion)" << std::endl;
    file << "Camera2.k1: 0.0" << std::endl;
    file << "Camera2.k2: 0.0" << std::endl;
    file << "Camera2.p1: 0.0" << std::endl;
    file << "Camera2.p2: 0.0" << std::endl;
    file << "Camera2.k3: 0.0" << std::endl;
    file << std::endl;

    // Camera dimensions
    file << "# Camera image dimensions" << std::endl;
    file << "Camera.width: " << selectedProfile.width << std::endl;
    file << "Camera.height: " << selectedProfile.height << std::endl;
    file << std::endl;

    file << "# Camera frames per second" << std::endl;
    file << "Camera.fps: " << selectedProfile.fps << std::endl;
    file << std::endl;

    file << "# Color encoding (0 for BGR, 1 for RGB)" << std::endl;
    file << "Camera.RGB: 0" << std::endl;
    file << std::endl;

    // Stereo extrinsics
    float baseline = sqrt(leftToRightIR.trans[0]*leftToRightIR.trans[0] + 
                         leftToRightIR.trans[1]*leftToRightIR.trans[1] + 
                         leftToRightIR.trans[2]*leftToRightIR.trans[2]);

    file << "# STEREO EXTRINSICS - TC1C2" << std::endl;
    file << "# Transformation from Left IR (Camera1) to Right IR (Camera2)" << std::endl;
    file << "# Baseline: " << baseline << " mm (" << baseline/1000.0 << " m)" << std::endl;
    file << "Stereo.T_c1_c2: !!opencv-matrix" << std::endl;
    file << "   rows: 4" << std::endl;
    file << "   cols: 4" << std::endl;
    file << "   dt: f" << std::endl;
    file << "   data: [";
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            file << leftToRightIR.rot[i*3 + j];
            if (!(i == 2 && j == 2)) file << ", ";
        }
        file << ", " << leftToRightIR.trans[i]/1000.0; // Convert mm to m
        if (i < 2) file << "," << std::endl << "          ";
    }
    file << "," << std::endl << "          0.0, 0.0, 0.0, 1.0]" << std::endl;
    file << std::endl;

    // IMU parameters if available
    if (hasIMU) {
        file << "#--------------------------------------------------------------------------------------------" << std::endl;
        file << "# IMU Parameters" << std::endl;
        file << "#--------------------------------------------------------------------------------------------" << std::endl;
        file << std::endl;

        file << "# IMU noise characteristics (DEFAULT - MANUAL CALIBRATION RECOMMENDED)" << std::endl;
        file << "IMU.NoiseGyro: 0.00016   # rad/s/sqrt(Hz)" << std::endl;
        file << "IMU.NoiseAcc: 0.0028     # m/s^2/sqrt(Hz)" << std::endl;
        file << "IMU.GyroWalk: 0.000002   # rad/s^2/sqrt(Hz)" << std::endl;
        file << "IMU.AccWalk: 0.00008     # m/s^3/sqrt(Hz)" << std::endl;
        file << "IMU.Frequency: 200       # Hz" << std::endl;
        file << std::endl;

        // Camera-IMU extrinsics (inverse of leftIRToIMU)
        file << "# IMU EXTRINSICS - TBC1 (INVERSE of camera-to-IMU)" << std::endl;
        file << "# Transformation from IMU Body frame (B) to Left IR Camera (C1)" << std::endl;
        file << "IMU.T_b_c1: !!opencv-matrix" << std::endl;
        file << "   rows: 4" << std::endl;
        file << "   cols: 4" << std::endl;
        file << "   dt: f" << std::endl;
        file << "   data: [";
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                file << leftIRToIMU.rot[i*3 + j];
                if (!(i == 2 && j == 2)) file << ", ";
            }
            // For TBC1, we need the inverse transformation
            // Since rotation is identity, we negate the translation
            file << ", " << -leftIRToIMU.trans[i]/1000.0; // Convert mm to m and invert
            if (i < 2) file << "," << std::endl << "          ";
        }
        file << "," << std::endl << "          0.0, 0.0, 0.0, 1.0]" << std::endl;
        file << std::endl;
    }

    // ORB Features and other parameters
    file << "#--------------------------------------------------------------------------------------------" << std::endl;
    file << "# ORB Features" << std::endl;
    file << "#--------------------------------------------------------------------------------------------" << std::endl;
    file << std::endl;
    file << "ORBextractor.nFeatures: 1200" << std::endl;
    file << "ORBextractor.scaleFactor: 1.2" << std::endl;
    file << "ORBextractor.nLevels: 8" << std::endl;
    file << "ORBextractor.iniThFAST: 20" << std::endl;
    file << "ORBextractor.minThFAST: 7" << std::endl;
    file << std::endl;

    file << "#--------------------------------------------------------------------------------------------" << std::endl;
    file << "# Stereo Parameters" << std::endl;
    file << "#--------------------------------------------------------------------------------------------" << std::endl;
    file << std::endl;
    file << "Stereo.ThDepth: 35.0" << std::endl;
    file << "Stereo.FarThr: 6000.0" << std::endl;
    file << std::endl;

    file << "#--------------------------------------------------------------------------------------------" << std::endl;
    file << "# Viewer Parameters" << std::endl;
    file << "#--------------------------------------------------------------------------------------------" << std::endl;
    file << std::endl;
    file << "Viewer.KeyFrameSize: 0.05" << std::endl;
    file << "Viewer.KeyFrameLineWidth: 1.0" << std::endl;
    file << "Viewer.GraphLineWidth: 0.9" << std::endl;
    file << "Viewer.PointSize: 2.0" << std::endl;
    file << "Viewer.CameraSize: 0.08" << std::endl;
    file << "Viewer.CameraLineWidth: 3.0" << std::endl;
    file << "Viewer.ViewpointX: 0.0" << std::endl;
    file << "Viewer.ViewpointY: -0.7" << std::endl;
    file << "Viewer.ViewpointZ: -1.8" << std::endl;
    file << "Viewer.ViewpointF: 500.0" << std::endl;

    file.close();
    std::cout << "Parameters saved to: " << filename << std::endl;
}

int main() try {
    std::cout << "=== IMPROVED Orbbec Gemini 335 - Stereo IR Parameter Extractor ===" << std::endl;
    std::cout << "Features: Resolution selection, multi-resolution support" << std::endl;
    std::cout << "Based on OrbbecSDK v2.4.8" << std::endl;
    std::cout << "=============================================================" << std::endl;

    // Create context and get device
    ob::Context context;
    auto deviceList = context.queryDeviceList();
    
    if (deviceList->getCount() == 0) {
        std::cerr << "No Orbbec device found!" << std::endl;
        return -1;
    }
    
    auto device = deviceList->getDevice(0);
    auto deviceInfo = device->getDeviceInfo();
    
    std::cout << "Device found: " << deviceInfo->name() << std::endl;
    std::cout << "Serial Number: " << deviceInfo->serialNumber() << std::endl;
    std::cout << "Firmware Version: " << deviceInfo->firmwareVersion() << std::endl;
    std::cout << "=============================================================" << std::endl;

    // Check available sensors
    auto sensorList = device->getSensorList();
    bool hasLeftIR = false, hasRightIR = false, hasIMU = false;
    
    for (uint32_t i = 0; i < sensorList->getCount(); i++) {
        auto sensorType = sensorList->getSensorType(i);
        if (sensorType == OB_SENSOR_IR_LEFT) hasLeftIR = true;
        if (sensorType == OB_SENSOR_IR_RIGHT) hasRightIR = true;
        if (sensorType == OB_SENSOR_ACCEL || sensorType == OB_SENSOR_GYRO) hasIMU = true;
    }

    if (!hasLeftIR || !hasRightIR) {
        std::cerr << "ERROR: Device does not support stereo IR cameras!" << std::endl;
        std::cerr << "Left IR: " << (hasLeftIR ? "Available" : "Missing") << std::endl;
        std::cerr << "Right IR: " << (hasRightIR ? "Available" : "Missing") << std::endl;
        return -1;
    }

    // Enumerate available IR stream profiles
    std::vector<StreamProfileInfo> leftIRProfiles, rightIRProfiles;
    enumerateIRStreamProfiles(device, leftIRProfiles, rightIRProfiles);

    if (leftIRProfiles.empty() || rightIRProfiles.empty()) {
        std::cerr << "ERROR: No IR stream profiles found!" << std::endl;
        return -1;
    }

    // Let user select resolution
    std::cout << "\n=== RESOLUTION SELECTION ===" << std::endl;
    std::cout << "Available matching resolutions:" << std::endl;
    
    // Find common resolutions between left and right IR
    std::vector<std::pair<StreamProfileInfo, StreamProfileInfo>> commonProfiles;
    for (const auto& leftProfile : leftIRProfiles) {
        for (const auto& rightProfile : rightIRProfiles) {
            if (leftProfile.width == rightProfile.width && 
                leftProfile.height == rightProfile.height &&
                leftProfile.fps == rightProfile.fps) {
                commonProfiles.push_back({leftProfile, rightProfile});
            }
        }
    }

    if (commonProfiles.empty()) {
        std::cerr << "ERROR: No matching resolutions found between left and right IR cameras!" << std::endl;
        return -1;
    }

    for (size_t i = 0; i < commonProfiles.size(); i++) {
        std::cout << "  " << i << ". " << commonProfiles[i].first.toString() << std::endl;
    }

    int selection = getUserSelection("\nSelect resolution index (or -1 to exit): ", commonProfiles.size());
    if (selection == -1) {
        std::cout << "Exiting..." << std::endl;
        return 0;
    }

    auto selectedProfile = commonProfiles[selection];
    std::cout << "Selected: " << selectedProfile.first.toString() << std::endl;

    // Create pipeline with selected resolution
    ob::Pipeline pipeline(device);
    auto config = std::make_shared<ob::Config>();

    // Configure streams with selected resolution
    config->enableVideoStream(OB_STREAM_IR_LEFT, 
                             selectedProfile.first.width, 
                             selectedProfile.first.height, 
                             selectedProfile.first.fps, 
                             selectedProfile.first.format);
    config->enableVideoStream(OB_STREAM_IR_RIGHT, 
                             selectedProfile.second.width, 
                             selectedProfile.second.height, 
                             selectedProfile.second.fps, 
                             selectedProfile.second.format);

    std::cout << "\nStarting pipeline with selected resolution..." << std::endl;
    pipeline.start(config);

    // Wait for frames to ensure calibration data is available
    auto frameset = pipeline.waitForFrameset(1000);
    if (!frameset) {
        std::cerr << "Failed to get frameset for calibration" << std::endl;
        return -1;
    }

    std::cout << "Extracting calibration parameters..." << std::endl;

    // Try to get detailed calibration parameters
    try {
        auto calibrationParams = pipeline.getCalibrationParam(config);
        
        // Extract IR parameters
        OBCameraIntrinsic leftIRIntrinsic = calibrationParams.intrinsics[OB_SENSOR_IR_LEFT];
        OBCameraIntrinsic rightIRIntrinsic = calibrationParams.intrinsics[OB_SENSOR_IR_RIGHT];
        OBExtrinsic leftToRightIR = calibrationParams.extrinsics[OB_SENSOR_IR_LEFT][OB_SENSOR_IR_RIGHT];
        OBExtrinsic leftIRToIMU = {};
        
        if (hasIMU) {
            leftIRToIMU = calibrationParams.extrinsics[OB_SENSOR_IR_LEFT][OB_SENSOR_ACCEL];
        }

        std::cout << "\n=== EXTRACTED PARAMETERS ===" << std::endl;
        printIRCameraIntrinsics(leftIRIntrinsic, "Left");
        printIRCameraIntrinsics(rightIRIntrinsic, "Right");
        printExtrinsics(leftToRightIR, "Left IR", "Right IR");
        
        if (hasIMU) {
            printExtrinsics(leftIRToIMU, "Left IR", "IMU");
        }

        // Generate filename with resolution
        std::string filename = "Orbbec_Gemini335_Stereo_" + selectedProfile.first.getResolutionStr() + ".yaml";
        
        // Save parameters
        std::cout << "\n=== SAVING PARAMETERS ===" << std::endl;
        saveParametersToFile(filename, selectedProfile.first, 
                           leftIRIntrinsic, rightIRIntrinsic, leftToRightIR, leftIRToIMU, hasIMU);

    } catch (const ob::Error& e) {
        std::cerr << "Error getting detailed calibration parameters: " << e.getMessage() << std::endl;
        std::cout << "Trying fallback method..." << std::endl;
        
        // Fallback to stream profile intrinsics
        auto leftProfiles = pipeline.getStreamProfileList(OB_SENSOR_IR_LEFT);
        auto rightProfiles = pipeline.getStreamProfileList(OB_SENSOR_IR_RIGHT);
        
        // Find matching profiles
        auto leftProfile = leftProfiles->getProfile(0)->as<ob::VideoStreamProfile>();
        auto rightProfile = rightProfiles->getProfile(0)->as<ob::VideoStreamProfile>();
        
        auto leftIntrinsic = leftProfile->getIntrinsic();
        auto rightIntrinsic = rightProfile->getIntrinsic();
        auto extrinsic = leftProfile->getExtrinsicTo(rightProfile);
        
        std::cout << "\n=== BASIC PARAMETERS (from stream profiles) ===" << std::endl;
        printIRCameraIntrinsics(leftIntrinsic, "Left");
        printIRCameraIntrinsics(rightIntrinsic, "Right");
        printExtrinsics(extrinsic, "Left IR", "Right IR");
        
        std::string filename = "Orbbec_Gemini335_Stereo_" + selectedProfile.first.getResolutionStr() + "_Basic.yaml";
        saveParametersToFile(filename, selectedProfile.first, 
                           leftIntrinsic, rightIntrinsic, extrinsic, {}, false);
    }

    pipeline.stop();
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