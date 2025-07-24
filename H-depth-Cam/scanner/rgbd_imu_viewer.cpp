#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <mutex>
#include <thread>
#include <cmath>
#include <algorithm>
#include <memory>
#include <deque>
#include <map>
#include <random>

// Include the actual MV3D RGBD SDK headers from the correct path
#include "../common/common.hpp"
#include "Mv3dRgbdAdvancedApi.h"
#include "Mv3dRgbdAdvancedDefine.h"

// OpenCV for real-time display
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>

#ifdef _WIN32
#include <conio.h>
#include <windows.h>
#define KBHIT() _kbhit()
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#else
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
struct termios oldt, newt;
int KBHIT() {
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    newt.c_cc[VMIN] = 0;
    newt.c_cc[VTIME] = 0;
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    if (ch != -1) {
        ungetc(ch, stdin);
        return 1;
    }
    return 0;
}
#endif

// Global IMU data structure with thread safety
struct IMUData {
    float xAcc, yAcc, zAcc;
    float xGyro, yGyro, zGyro;
    std::mutex dataMutex;
    bool hasNewData;
    std::chrono::steady_clock::time_point lastUpdateTime;

    IMUData() : xAcc(0), yAcc(0), zAcc(0), xGyro(0), yGyro(0), zGyro(0), hasNewData(false) {
        lastUpdateTime = std::chrono::steady_clock::now();
    }

    void updateData(float xa, float ya, float za, float xg, float yg, float zg) {
        std::lock_guard<std::mutex> lock(dataMutex);
        xAcc = xa; yAcc = ya; zAcc = za;
        xGyro = xg; yGyro = yg; zGyro = zg;
        hasNewData = true;
        lastUpdateTime = std::chrono::steady_clock::now();
    }

    void getData(float& xa, float& ya, float& za, float& xg, float& yg, float& zg, bool& newData) {
        std::lock_guard<std::mutex> lock(dataMutex);
        xa = xAcc; ya = yAcc; za = zAcc;
        xg = xGyro; yg = yGyro; zg = zGyro;

        // Check if data is recent (within last 2 seconds)
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - lastUpdateTime);
        newData = hasNewData && (elapsed.count() < 2);
        hasNewData = false;
    }

    bool isRecentData() {
        std::lock_guard<std::mutex> lock(dataMutex);
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - lastUpdateTime);
        return elapsed.count() < 2;
    }
};

// Global IMU data instance
IMUData g_imuData;
bool g_hasIMUData = false;
MV3D_RGBD_IMU_DATA g_lastIMUData;

// IMU callback function - FIXED VERSION
void __stdcall IMUCallBackFunc(MV3D_RGBD_IMU_DATA* pstIMUData, void* pUser)
{
    if (NULL != pstIMUData)
    {
        // Update global IMU state for SLAM
        g_hasIMUData = true;
        g_lastIMUData = *pstIMUData;

        // Update the thread-safe IMU data structure
        g_imuData.updateData(
            pstIMUData->fXAccSpeed, pstIMUData->fYAccSpeed, pstIMUData->fZAccSpeed,
            pstIMUData->fXAngSpeed, pstIMUData->fYAngSpeed, pstIMUData->fZAngSpeed
        );

        // Print IMU data for debugging (same format as working imu_test.cpp)
        std::cout << "IMU Data: X_Acc(" << std::fixed << std::setprecision(1) << pstIMUData->fXAccSpeed << ") "
            << "Y_Acc(" << pstIMUData->fYAccSpeed << ") "
            << "Z_Acc(" << pstIMUData->fZAccSpeed << ") "
            << "X_Gyro(" << pstIMUData->fXAngSpeed << ") "
            << "Y_Gyro(" << pstIMUData->fYAngSpeed << ") "
            << "Z_Gyro(" << pstIMUData->fZAngSpeed << ")" << std::endl;
    }
}

// Enhanced SLAM structures
struct KeyFrame {
    int id;
    cv::Mat rgb_image;
    cv::Mat depth_image;
    cv::Mat pose; // 4x4 transformation matrix
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    std::chrono::steady_clock::time_point timestamp;
};

struct MapPoint {
    int id;
    cv::Point3f world_pos;
    std::vector<int> observed_keyframes;
    cv::Mat descriptor;
};

class SimpleSLAM {
private:
    std::vector<std::shared_ptr<KeyFrame>> keyframes;
    std::vector<std::shared_ptr<MapPoint>> mappoints;
    cv::Ptr<cv::ORB> orb_detector;
    cv::BFMatcher matcher;

    // Camera intrinsics - using your calibrated parameters
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;

    // SLAM state
    cv::Mat current_pose;
    int next_keyframe_id;
    int next_mappoint_id;

    // Tracking state
    bool tracking_lost;
    int frames_since_last_keyframe;

    // IMU integration variables
    cv::Mat velocity; // 3x1 velocity vector
    cv::Mat angular_velocity; // 3x1 angular velocity vector
    std::chrono::steady_clock::time_point last_imu_time;
    std::chrono::steady_clock::time_point last_frame_time;
    bool imu_initialized;

public:
    SimpleSLAM() : next_keyframe_id(0), next_mappoint_id(0), tracking_lost(false),
        frames_since_last_keyframe(0), imu_initialized(false) {
        // Initialize ORB detector
        orb_detector = cv::ORB::create(1000);

        // Initialize camera intrinsics with your calibrated parameters
        camera_matrix = (cv::Mat_<double>(3, 3) <<
            735.749, 0, 617.785,
            0, 731.258, 358.336,
            0, 0, 1);

        dist_coeffs = cv::Mat::zeros(4, 1, CV_64F);

        // Initialize pose as identity
        current_pose = cv::Mat::eye(4, 4, CV_64F);

        // Initialize IMU integration
        velocity = cv::Mat::zeros(3, 1, CV_64F);
        angular_velocity = cv::Mat::zeros(3, 1, CV_64F);
        last_frame_time = std::chrono::steady_clock::now();

        std::cout << "SLAM system initialized with calibrated camera parameters:" << std::endl;
        std::cout << "fx=" << 735.749 << ", fy=" << 731.258 << ", cx=" << 617.785 << ", cy=" << 358.336 << std::endl;
    }

    // Add method to update IMU data
    void UpdateIMU(float ax, float ay, float az, float gx, float gy, float gz) {
        auto now = std::chrono::steady_clock::now();

        if (!imu_initialized) {
            last_imu_time = now;
            imu_initialized = true;
            return;
        }

        // Calculate time delta
        auto dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_imu_time);
        double dt = dt_ms.count() / 1000.0; // Convert to seconds

        if (dt > 0.001 && dt < 0.1) { // Valid time range (1ms to 100ms)
            // Update velocity with acceleration (simple integration)
            // Note: Remove gravity component (assuming camera is roughly upright)
            double gravity_removed_ay = ay - 9.8; // Remove gravity from Y axis

            velocity.at<double>(0) += ax * dt;
            velocity.at<double>(1) += gravity_removed_ay * dt;
            velocity.at<double>(2) += az * dt;

            // Update angular velocity
            angular_velocity.at<double>(0) = gx * (M_PI / 180.0); // Convert to rad/s
            angular_velocity.at<double>(1) = gy * (M_PI / 180.0);
            angular_velocity.at<double>(2) = gz * (M_PI / 180.0);

            // Apply velocity damping to prevent drift
            velocity *= 0.95;

            // Update pose with IMU prediction
            if (!keyframes.empty()) {
                // Create rotation matrix from angular velocity
                cv::Mat rotation_delta = cv::Mat::eye(3, 3, CV_64F);

                // Small angle approximation for rotation
                double angle_mag = cv::norm(angular_velocity) * dt;
                if (angle_mag > 0.001) {
                    cv::Mat axis = angular_velocity / cv::norm(angular_velocity);

                    // Rodrigues formula for small rotations
                    cv::Mat K = (cv::Mat_<double>(3, 3) <<
                        0, -axis.at<double>(2), axis.at<double>(1),
                        axis.at<double>(2), 0, -axis.at<double>(0),
                        -axis.at<double>(1), axis.at<double>(0), 0);

                    rotation_delta = cv::Mat::eye(3, 3, CV_64F) + sin(angle_mag) * K +
                        (1 - cos(angle_mag)) * K * K;
                }

                // Update current pose with IMU prediction
                cv::Mat translation_delta = velocity * dt;

                // Apply to current pose
                cv::Mat R = current_pose(cv::Rect(0, 0, 3, 3));
                cv::Mat t = current_pose(cv::Rect(3, 0, 1, 3));

                // Update rotation
                R = R * rotation_delta;

                // Update translation
                t = t + R * translation_delta;
            }
        }

        last_imu_time = now;
    }

    void ProcessFrame(const cv::Mat& rgb, const cv::Mat& depth) {
        if (rgb.empty() || depth.empty()) return;

        auto now = std::chrono::steady_clock::now();

        // Get latest IMU data and integrate it
        float xa, ya, za, xg, yg, zg;
        bool newData;
        g_imuData.getData(xa, ya, za, xg, yg, zg, newData);

        if (newData) {
            UpdateIMU(xa, ya, za, xg, yg, zg);
        }

        // Detect features
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        orb_detector->detectAndCompute(rgb, cv::Mat(), keypoints, descriptors);

        if (keyframes.empty()) {
            // First frame - create initial keyframe
            CreateKeyFrame(rgb, depth, keypoints, descriptors);
            std::cout << "Added keyframe " << (keyframes.size() - 1) << " (total: " << keyframes.size() << ")" << std::endl;
        }
        else {
            // Track against previous keyframes
            TrackFrame(rgb, depth, keypoints, descriptors);
        }

        last_frame_time = now;
        frames_since_last_keyframe++;
    }

    void CreateKeyFrame(const cv::Mat& rgb, const cv::Mat& depth,
        const std::vector<cv::KeyPoint>& keypoints,
        const cv::Mat& descriptors) {
        auto kf = std::make_shared<KeyFrame>();
        kf->id = next_keyframe_id++;
        kf->rgb_image = rgb.clone();
        kf->depth_image = depth.clone();
        kf->keypoints = keypoints;
        kf->descriptors = descriptors.clone();
        kf->pose = current_pose.clone();
        kf->timestamp = std::chrono::steady_clock::now();

        keyframes.push_back(kf);

        // Create map points from this keyframe
        CreateMapPoints(kf);

        frames_since_last_keyframe = 0;
    }

    void CreateMapPoints(std::shared_ptr<KeyFrame> kf) {
        for (size_t i = 0; i < kf->keypoints.size(); i++) {
            cv::Point2f pixel = kf->keypoints[i].pt;

            // Check if depth is valid
            int x = static_cast<int>(pixel.x);
            int y = static_cast<int>(pixel.y);

            if (x >= 0 && x < kf->depth_image.cols && y >= 0 && y < kf->depth_image.rows) {
                uint16_t depth_val = kf->depth_image.at<uint16_t>(y, x);

                if (depth_val > 0 && depth_val < 4000) { // Valid depth range
                    // Convert to 3D point
                    float z = depth_val / 1000.0f; // Convert mm to m
                    float x_3d = (pixel.x - 617.785f) * z / 735.749f;
                    float y_3d = (pixel.y - 358.336f) * z / 731.258f;

                    auto mp = std::make_shared<MapPoint>();
                    mp->id = next_mappoint_id++;
                    mp->world_pos = cv::Point3f(x_3d, y_3d, z);
                    mp->observed_keyframes.push_back(kf->id);
                    if (i < static_cast<size_t>(kf->descriptors.rows)) {
                        mp->descriptor = kf->descriptors.row(static_cast<int>(i)).clone();
                    }

                    mappoints.push_back(mp);
                }
            }
        }
    }

    void TrackFrame(const cv::Mat& rgb, const cv::Mat& depth,
        const std::vector<cv::KeyPoint>& keypoints,
        const cv::Mat& descriptors) {
        if (keyframes.empty() || descriptors.empty()) return;

        // Match with last keyframe
        auto last_kf = keyframes.back();
        if (last_kf->descriptors.empty()) return;

        std::vector<cv::DMatch> matches;
        matcher.match(descriptors, last_kf->descriptors, matches);

        // Filter good matches
        std::vector<cv::DMatch> good_matches;
        if (!matches.empty()) {
            double min_dist = std::min_element(matches.begin(), matches.end(),
                [](const cv::DMatch& a, const cv::DMatch& b) { return a.distance < b.distance; })->distance;

            for (const auto& match : matches) {
                if (match.distance <= std::max(2.0 * min_dist, 30.0)) {
                    good_matches.push_back(match);
                }
            }
        }

        // Check if we have IMU data to help with tracking
        bool has_imu_motion = false;
        if (imu_initialized) {
            double motion_magnitude = cv::norm(velocity) + cv::norm(angular_velocity);
            has_imu_motion = motion_magnitude > 0.1; // Threshold for detecting motion
        }

        // Update tracking state with IMU assistance
        if (good_matches.size() < 20 && !has_imu_motion) {
            tracking_lost = true;
            std::cout << "SLAM tracking lost - trying to recover..." << std::endl;
        }
        else if (good_matches.size() < 10 && has_imu_motion) {
            // IMU detects motion but few visual matches - keep tracking but create keyframe
            tracking_lost = false;
            std::cout << "Low visual matches but IMU motion detected - maintaining track" << std::endl;
        }
        else {
            tracking_lost = false;
        }

        // Create new keyframe if needed (more aggressive with IMU data)
        bool should_create_keyframe = false;
        if (has_imu_motion && frames_since_last_keyframe > 5) {
            should_create_keyframe = true;
        }
        else if (frames_since_last_keyframe > 10 || good_matches.size() < 50) {
            should_create_keyframe = true;
        }

        if (should_create_keyframe) {
            CreateKeyFrame(rgb, depth, keypoints, descriptors);
            std::cout << "Added keyframe " << (keyframes.size() - 1) << " (total: " << keyframes.size() << ")" << std::endl;
        }
    }

    void GetSLAMInfo(float& x, float& y, float& z, int& kf_count, int& mp_count) {
        // Extract position from current pose
        x = static_cast<float>(current_pose.at<double>(0, 3));
        y = static_cast<float>(current_pose.at<double>(1, 3));
        z = static_cast<float>(current_pose.at<double>(2, 3));

        kf_count = static_cast<int>(keyframes.size());
        mp_count = static_cast<int>(mappoints.size());
    }

    bool HasIMUMotion() {
        if (!imu_initialized) return false;
        double motion_magnitude = cv::norm(velocity) + cv::norm(angular_velocity);
        return motion_magnitude > 0.05; // Lower threshold for display
    }

    void Reset() {
        keyframes.clear();
        mappoints.clear();
        current_pose = cv::Mat::eye(4, 4, CV_64F);
        next_keyframe_id = 0;
        next_mappoint_id = 0;
        tracking_lost = false;
        frames_since_last_keyframe = 0;
        velocity = cv::Mat::zeros(3, 1, CV_64F);
        angular_velocity = cv::Mat::zeros(3, 1, CV_64F);
        imu_initialized = false;
        std::cout << "SLAM system reset." << std::endl;
    }

    bool IsTrackingLost() const { return tracking_lost; }
};

class RGBDIRViewer {
private:
    void* m_handle;
    bool m_bDeviceOpen;
    bool m_bGrabbing;
    int m_frameCount;

    // SLAM system
    SimpleSLAM m_slam;

    // Display resolution settings
    struct DisplayResolution {
        int width;
        int height;
        std::string name;
    };

    std::vector<DisplayResolution> m_resolutions;
    int m_selectedResolution;

public:
    RGBDIRViewer() : m_handle(nullptr), m_bDeviceOpen(false), m_bGrabbing(false), m_frameCount(0), m_selectedResolution(0) {
        // Initialize available display resolutions - all 16:9 aspect ratio
        m_resolutions = {
            {640, 360, "640x360 (16:9)"},
            {854, 480, "854x480 (16:9)"},
            {960, 540, "960x540 (16:9)"},
            {1280, 720, "1280x720 (16:9) - HD"},
            {1920, 1080, "1920x1080 (16:9) - Full HD"}
        };
    }

    ~RGBDIRViewer() {
        if (m_bGrabbing) {
            StopGrabbing();
        }
        if (m_bDeviceOpen) {
            CloseDevice();
        }
        Finalize();
    }

    int SelectDisplayResolution() {
        std::cout << "Select display resolution:" << std::endl;
        for (size_t i = 0; i < m_resolutions.size(); i++) {
            std::cout << i << ": " << m_resolutions[i].name << std::endl;
        }

        std::cout << "Enter selection (0-" << (m_resolutions.size() - 1) << "): ";
        if (!(std::cin >> m_selectedResolution) ||
            m_selectedResolution < 0 ||
            m_selectedResolution >= static_cast<int>(m_resolutions.size())) {
            std::cout << "Invalid selection. Using default resolution." << std::endl;
            m_selectedResolution = 0;
            return MV3D_RGBD_E_PARAMETER;
        }

        std::cout << "Selected: " << m_resolutions[m_selectedResolution].name << std::endl;
        return MV3D_RGBD_OK;
    }

    int Initialize() {
        int nRet = MV3D_RGBD_Initialize();
        if (MV3D_RGBD_OK != nRet) {
            std::cerr << "MV3D_RGBD_Initialize failed! Error: 0x" << std::hex << nRet << std::endl;
            return nRet;
        }
        std::cout << "SDK initialized successfully." << std::endl;
        return MV3D_RGBD_OK;
    }

    int EnumerateDevices(std::vector<MV3D_RGBD_DEVICE_INFO>& deviceList) {
        deviceList.clear();

        // Get device number first
        unsigned int nDevNum = 0;
        int nRet = MV3D_RGBD_GetDeviceNumber(DeviceType_Ethernet | DeviceType_USB, &nDevNum);
        if (MV3D_RGBD_OK != nRet) {
            std::cerr << "MV3D_RGBD_GetDeviceNumber failed! Error: 0x" << std::hex << nRet << std::endl;
            return nRet;
        }

        if (nDevNum == 0) {
            std::cerr << "No devices found!" << std::endl;
            return MV3D_RGBD_E_NODATA;
        }

        std::cout << "Found " << nDevNum << " device(s):" << std::endl;

        // Get device list
        deviceList.resize(nDevNum);
        nRet = MV3D_RGBD_GetDeviceList(DeviceType_Ethernet | DeviceType_USB, &deviceList[0], nDevNum, &nDevNum);
        if (MV3D_RGBD_OK != nRet) {
            std::cerr << "MV3D_RGBD_GetDeviceList failed! Error: 0x" << std::hex << nRet << std::endl;
            return nRet;
        }

        for (unsigned int i = 0; i < nDevNum; i++) {
            std::cout << "[" << i << "] Model: " << deviceList[i].chModelName
                << ", SN: " << deviceList[i].chSerialNumber << std::endl;
        }

        return MV3D_RGBD_OK;
    }

    int OpenDevice(const MV3D_RGBD_DEVICE_INFO& deviceInfo) {
        int nRet = MV3D_RGBD_OpenDevice(&m_handle, const_cast<MV3D_RGBD_DEVICE_INFO*>(&deviceInfo));
        if (MV3D_RGBD_OK != nRet) {
            std::cerr << "MV3D_RGBD_OpenDevice failed! Error: 0x" << std::hex << nRet << std::endl;
            return nRet;
        }

        m_bDeviceOpen = true;
        std::cout << "Device opened successfully." << std::endl;

        // FIXED IMU REGISTRATION - following working imu_test.cpp pattern exactly
        MV3D_RGBD_PARAM stParam;
        stParam.enParamType = ParamType_Enum;
        stParam.ParamInfo.stEnumParam.nCurValue = 1;
        nRet = MV3D_RGBD_SetParam(m_handle, "EventNotification", &stParam);
        if (MV3D_RGBD_OK == nRet) {
            std::cout << "EventNotification set successfully." << std::endl;

            nRet = MV3D_RGBD_RegisterIMUDataCallBack(m_handle, IMUCallBackFunc, m_handle);
            if (MV3D_RGBD_OK == nRet) {
                std::cout << "IMU callback registered successfully." << std::endl;
                std::cout << "IMU data should now be available when camera moves." << std::endl;

                // Don't try to set additional IMU enable parameters - they're not needed for MV-EB435i
                // The camera starts sending IMU data automatically after callback registration
            }
            else {
                std::cout << "Warning: Failed to register IMU callback: 0x" << std::hex << nRet << std::endl;
            }
        }
        else {
            std::cout << "Warning: Failed to set EventNotification: 0x" << std::hex << nRet << std::endl;
        }

        return MV3D_RGBD_OK;
    }

    int StartGrabbing() {
        if (!m_bDeviceOpen) {
            std::cerr << "Device is not open!" << std::endl;
            return MV3D_RGBD_E_HANDLE;
        }

        int nRet = MV3D_RGBD_Start(m_handle);
        if (MV3D_RGBD_OK != nRet) {
            std::cerr << "MV3D_RGBD_Start failed! Error: 0x" << std::hex << nRet << std::endl;
            return nRet;
        }

        m_bGrabbing = true;
        std::cout << "Started grabbing successfully." << std::endl;
        return MV3D_RGBD_OK;
    }

    int StopGrabbing() {
        if (!m_bGrabbing) {
            return MV3D_RGBD_OK;
        }

        int nRet = MV3D_RGBD_Stop(m_handle);
        if (MV3D_RGBD_OK != nRet) {
            std::cerr << "MV3D_RGBD_Stop failed! Error: 0x" << std::hex << nRet << std::endl;
            return nRet;
        }

        m_bGrabbing = false;
        std::cout << "Stopped grabbing successfully." << std::endl;
        return MV3D_RGBD_OK;
    }

    int CloseDevice() {
        if (!m_bDeviceOpen) {
            return MV3D_RGBD_OK;
        }

        int nRet = MV3D_RGBD_CloseDevice(&m_handle);
        if (MV3D_RGBD_OK != nRet) {
            std::cerr << "MV3D_RGBD_CloseDevice failed! Error: 0x" << std::hex << nRet << std::endl;
            return nRet;
        }

        m_bDeviceOpen = false;
        m_handle = nullptr;
        std::cout << "Device closed successfully." << std::endl;
        return MV3D_RGBD_OK;
    }

    int Finalize() {
        int nRet = MV3D_RGBD_Release();
        if (MV3D_RGBD_OK != nRet) {
            std::cerr << "MV3D_RGBD_Release failed! Error: 0x" << std::hex << nRet << std::endl;
            return nRet;
        }

        std::cout << "SDK finalized successfully." << std::endl;
        return MV3D_RGBD_OK;
    }

    cv::Mat ConvertDepthToDisplay(const MV3D_RGBD_IMAGE_DATA& imageData) {
        if (imageData.enImageType != ImageType_Depth) {
            return cv::Mat();
        }

        cv::Mat depth(imageData.nHeight, imageData.nWidth, CV_16UC1, imageData.pData);
        cv::Mat depthDisplay;

        // Convert depth to colormap for visualization
        cv::Mat depth8;
        depth.convertTo(depth8, CV_8UC1, 255.0 / 4000.0);
        cv::applyColorMap(depth8, depthDisplay, cv::COLORMAP_JET);

        return depthDisplay;
    }

    cv::Mat ConvertColorToDisplay(const MV3D_RGBD_IMAGE_DATA& imageData) {
        if (ImageType_RGB8_Planar == imageData.enImageType) {
            cv::Mat rgbDisplay(imageData.nHeight, imageData.nWidth, CV_8UC3);
            unsigned char* srcData = (unsigned char*)imageData.pData;
            unsigned char* dstData = rgbDisplay.data;

            int pixelCount = imageData.nWidth * imageData.nHeight;
            for (int i = 0; i < pixelCount; i++) {
                dstData[i * 3 + 2] = srcData[i];                    // R
                dstData[i * 3 + 1] = srcData[i + pixelCount];       // G
                dstData[i * 3 + 0] = srcData[i + 2 * pixelCount];   // B
            }
            return rgbDisplay;
        }
        else if (ImageType_YUV422 == imageData.enImageType) {
            cv::Mat yuvMat(imageData.nHeight, imageData.nWidth, CV_8UC2, imageData.pData);
            cv::Mat rgbMat;
            cv::cvtColor(yuvMat, rgbMat, cv::COLOR_YUV2BGR_YUYV);
            return rgbMat;
        }

        return cv::Mat();
    }

    cv::Mat ConvertIRToDisplay(const MV3D_RGBD_IMAGE_DATA& imageData) {
        if (ImageType_Mono8 == imageData.enImageType) {
            cv::Mat irMat(imageData.nHeight, imageData.nWidth, CV_8UC1, imageData.pData);
            cv::Mat irDisplay;
            cv::cvtColor(irMat, irDisplay, cv::COLOR_GRAY2BGR);
            return irDisplay;
        }
        else if (ImageType_Mono16 == imageData.enImageType) {
            cv::Mat irMat(imageData.nHeight, imageData.nWidth, CV_16UC1, imageData.pData);
            cv::Mat ir8, irDisplay;
            irMat.convertTo(ir8, CV_8UC1, 255.0 / 65535.0);
            cv::cvtColor(ir8, irDisplay, cv::COLOR_GRAY2BGR);
            return irDisplay;
        }

        return cv::Mat();
    }

    void ResizeForDisplay(const cv::Mat& src, cv::Mat& dst) {
        if (src.empty()) {
            dst = cv::Mat();
            return;
        }

        const DisplayResolution& res = m_resolutions[m_selectedResolution];

        // Calculate scaling to fit within target resolution while maintaining aspect ratio
        double scaleX = static_cast<double>(res.width) / src.cols;
        double scaleY = static_cast<double>(res.height) / src.rows;
        double scale = std::min(scaleX, scaleY);

        int newWidth = static_cast<int>(src.cols * scale);
        int newHeight = static_cast<int>(src.rows * scale);

        cv::resize(src, dst, cv::Size(newWidth, newHeight));
    }

    void AddIMUOverlay(cv::Mat& image) {
        if (image.empty()) return;

        float xa, ya, za, xg, yg, zg;
        bool newData;
        g_imuData.getData(xa, ya, za, xg, yg, zg, newData);

        std::vector<std::string> imuText;
        std::stringstream ss;

        imuText.push_back("=== IMU Data ===");

        if (newData && g_imuData.isRecentData()) {
            ss.str(""); ss << std::fixed << std::setprecision(2);
            ss << "Acc: (" << xa << ", " << ya << ", " << za << ") m/s²";
            imuText.push_back(ss.str());

            ss.str(""); ss << std::fixed << std::setprecision(2);
            ss << "Gyro: (" << xg << ", " << yg << ", " << zg << ") rad/s";
            imuText.push_back(ss.str());

            imuText.push_back("IMU Status: ACTIVE");
        }
        else {
            imuText.push_back("IMU Status: NO DATA");
            imuText.push_back("Check IMU connection");
        }

        // Draw IMU overlay
        int y_offset = image.rows - 100;
        for (size_t i = 0; i < imuText.size(); i++) {
            cv::Scalar color = (i == 0) ? cv::Scalar(0, 255, 255) :
                (newData ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255));
            cv::putText(image, imuText[i], cv::Point(10, y_offset + static_cast<int>(i) * 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
        }
    }

    void AddSLAMOverlay(cv::Mat& image) {
        if (image.empty()) return;

        float slam_x, slam_y, slam_z;
        int keyframe_count, mappoint_count;
        m_slam.GetSLAMInfo(slam_x, slam_y, slam_z, keyframe_count, mappoint_count);

        // Better IMU status detection
        std::string imu_status = "OFF";
        if (g_imuData.isRecentData() && g_hasIMUData) {
            if (m_slam.HasIMUMotion()) {
                imu_status = "MOTION";
            }
            else {
                imu_status = "STATIC";
            }
        }

        std::stringstream ss;
        ss << "SLAM - Position: (" << std::fixed << std::setprecision(2)
            << slam_x << ", " << slam_y << ", " << slam_z << ") m | "
            << "KF: " << keyframe_count << " | "
            << "MP: " << mappoint_count << " | "
            << "IMU: " << imu_status;

        cv::Scalar slamColor = m_slam.IsTrackingLost() ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);
        cv::putText(image, ss.str(), cv::Point(10, 30),
            cv::FONT_HERSHEY_SIMPLEX, 0.6, slamColor, 2);

        // Also print to console periodically
        static int console_counter = 0;
        if (console_counter++ % 30 == 0) { // Print every 30 frames
            std::cout << ss.str() << std::endl;
        }
    }

    void ProcessFrameData(const MV3D_RGBD_FRAME_DATA& frameData) {
        cv::Mat rgb_image, depth_image;

        for (unsigned int i = 0; i < frameData.nImageCount; i++) {
            const MV3D_RGBD_IMAGE_DATA& imageData = frameData.stImageData[i];

            std::string windowName;
            cv::Mat displayMat;

            switch (imageData.enImageType) {
            case ImageType_Depth:
                windowName = "Depth Stream with IMU & SLAM";
                displayMat = ConvertDepthToDisplay(imageData);
                depth_image = cv::Mat(imageData.nHeight, imageData.nWidth, CV_16UC1, imageData.pData).clone();
                break;

            case ImageType_RGB8_Planar:
            case ImageType_YUV422:
            case ImageType_YUV420SP_NV12:
            case ImageType_YUV420SP_NV21:
                windowName = "Color Stream with IMU & SLAM";
                displayMat = ConvertColorToDisplay(imageData);
                rgb_image = displayMat.clone();
                break;

            case ImageType_Mono8:
            case ImageType_Mono16:
                windowName = "IR Stream with IMU & SLAM";
                displayMat = ConvertIRToDisplay(imageData);
                break;

            default:
                // Skip unsupported formats
                continue;
            }

            if (!displayMat.empty()) {
                // Resize for display
                cv::Mat resizedMat;
                ResizeForDisplay(displayMat, resizedMat);

                // Add overlays
                AddIMUOverlay(resizedMat);
                AddSLAMOverlay(resizedMat);

                cv::imshow(windowName, resizedMat);
                m_frameCount++;
            }
        }

        // Process SLAM if we have both RGB and depth
        if (!rgb_image.empty() && !depth_image.empty()) {
            m_slam.ProcessFrame(rgb_image, depth_image);
        }
    }

    int RunViewer() {
        if (!m_bGrabbing) {
            std::cerr << "Grabbing not started!" << std::endl;
            return MV3D_RGBD_E_HANDLE;
        }

        std::cout << "\n=== Real-Time Viewer with IMU and SLAM Started ===" << std::endl;
        std::cout << "Display Resolution: " << m_resolutions[m_selectedResolution].name << std::endl;
        std::cout << "\nControls:" << std::endl;
        std::cout << "  - Press 'q' on any window to quit" << std::endl;
        std::cout << "  - Press 'ESC' on any window to quit" << std::endl;
        std::cout << "  - Press 'r' to change resolution" << std::endl;
        std::cout << "  - Press 's' to reset SLAM" << std::endl;
        std::cout << "  - Press 'c' to calibrate camera (info)" << std::endl;
        std::cout << "===============================" << std::endl;

        MV3D_RGBD_FRAME_DATA frameData = { 0 };

        while (true) {
            int nRet = MV3D_RGBD_FetchFrame(m_handle, &frameData, 1000);
            if (MV3D_RGBD_OK == nRet) {
                ProcessFrameData(frameData);
            }

            // Handle keyboard input
            int key = cv::waitKey(1) & 0xFF;
            if (key == 'q' || key == 27) { // 'q' or ESC
                break;
            }
            else if (key == 'r') {
                std::cout << "Change resolution feature - please restart application" << std::endl;
            }
            else if (key == 's') {
                std::cout << "Resetting SLAM..." << std::endl;
                m_slam.Reset();
            }
            else if (key == 'c') {
                std::cout << "Camera calibration info:" << std::endl;
                std::cout << "fx=735.749, fy=731.258, cx=617.785, cy=358.336" << std::endl;
                std::cout << "1. Check if IMU is enabled in camera settings" << std::endl;
                std::cout << "2. Verify IMU callback registration" << std::endl;
                std::cout << "3. Move camera gently to generate IMU data" << std::endl;
                std::cout << "===============================" << std::endl;
            }

            // Also check console input (for systems without window focus)
            if (KBHIT()) {
                break;
            }
        }

        std::cout << "\nViewer stopped. Total frames displayed: " << m_frameCount << std::endl;
        return MV3D_RGBD_OK;
    }
};

int main(int argc, char* argv[]) {
    std::cout << "=== RGBD, IR Real-Time Viewer with IMU Data and SLAM ===" << std::endl;
    std::cout << "This program displays RGBD and IR streams with IMU data overlay and SLAM." << std::endl;

    RGBDIRViewer viewer;

    // Select display resolution first
    int nRet = viewer.SelectDisplayResolution();
    if (MV3D_RGBD_OK != nRet) {
        return -1;
    }

    // Initialize SDK
    nRet = viewer.Initialize();
    if (MV3D_RGBD_OK != nRet) {
        std::cerr << "Failed to initialize. Make sure the MV3D RGBD SDK is properly installed." << std::endl;
        return -1;
    }

    // Enumerate devices
    std::vector<MV3D_RGBD_DEVICE_INFO> deviceList;
    nRet = viewer.EnumerateDevices(deviceList);
    if (MV3D_RGBD_OK != nRet || deviceList.empty()) {
        std::cerr << "No devices found. Make sure the camera is connected." << std::endl;
        return -1;
    }

    // Select device (use first device by default)
    unsigned int deviceIndex = 0;
    if (deviceList.size() > 1) {
        std::cout << "Multiple devices found. Enter device index (0-" << (deviceList.size() - 1) << "): ";
        std::cin >> deviceIndex;
        if (deviceIndex >= deviceList.size()) {
            std::cerr << "Invalid device index!" << std::endl;
            return -1;
        }
    }

    // Open device
    nRet = viewer.OpenDevice(deviceList[deviceIndex]);
    if (MV3D_RGBD_OK != nRet) {
        return -1;
    }

    // Start grabbing
    nRet = viewer.StartGrabbing();
    if (MV3D_RGBD_OK != nRet) {
        return -1;
    }

    // Run real-time viewer with IMU overlay and SLAM
    nRet = viewer.RunViewer();

    std::cout << "Real-time viewing with IMU data and SLAM completed!" << std::endl;
    return 0;
}