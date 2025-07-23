#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <mutex>
#include <memory>
#include <thread>
#include <deque>
#include <fstream>

// Include the actual MV3D RGBD SDK headers from the correct path
#include "../common/common.hpp"
#include "Mv3dRgbdAdvancedApi.h"
#include "Mv3dRgbdAdvancedDefine.h"

// OpenCV for image processing and feature detection
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

// Open3D for point cloud visualization and processing
#include <Open3D/Open3D.h>

// Eigen for matrix operations
#include <Eigen/Dense>
#include <Eigen/Geometry>

#ifdef _WIN32
#include <conio.h>
#include <windows.h>
#define KBHIT() _kbhit()
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

// SLAM Data Structures (following SLAMBOOK concepts)
struct IMUData {
    float xAcc, yAcc, zAcc;
    float xGyro, yGyro, zGyro;
    std::chrono::high_resolution_clock::time_point timestamp;
    std::mutex dataMutex;
    bool hasNewData;

    IMUData() : xAcc(0), yAcc(0), zAcc(0), xGyro(0), yGyro(0), zGyro(0), hasNewData(false) {}

    void updateData(float xa, float ya, float za, float xg, float yg, float zg) {
        std::lock_guard<std::mutex> lock(dataMutex);
        xAcc = xa; yAcc = ya; zAcc = za;
        xGyro = xg; yGyro = yg; zGyro = zg;
        timestamp = std::chrono::high_resolution_clock::now();
        hasNewData = true;
    }

    void getData(float& xa, float& ya, float& za, float& xg, float& yg, float& zg, bool& newData) {
        std::lock_guard<std::mutex> lock(dataMutex);
        xa = xAcc; ya = yAcc; za = zAcc;
        xg = xGyro; yg = yGyro; zg = zGyro;
        newData = hasNewData;
        hasNewData = false;
    }
};

struct SLAMPose {
    Eigen::Vector3d translation;
    Eigen::Quaterniond rotation;
    double confidence;
    std::chrono::high_resolution_clock::time_point timestamp;

    SLAMPose() : translation(0, 0, 0), rotation(Eigen::Quaterniond::Identity()), confidence(1.0) {
        timestamp = std::chrono::high_resolution_clock::now();
    }

    Eigen::Matrix4d toMatrix() const {
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        pose.block<3, 3>(0, 0) = rotation.toRotationMatrix();
        pose.block<3, 1>(0, 3) = translation;
        return pose;
    }

    void fromMatrix(const Eigen::Matrix4d& matrix) {
        translation = matrix.block<3, 1>(0, 3);
        rotation = Eigen::Quaterniond(matrix.block<3, 3>(0, 0));
    }
};

struct SLAMKeyframe {
    int id;
    cv::Mat rgb_image;
    cv::Mat depth_image;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    std::shared_ptr<open3d::geometry::PointCloud> point_cloud;
    SLAMPose pose;
    std::chrono::high_resolution_clock::time_point timestamp;
};

// Global IMU data instance
IMUData g_imuData;

// IMU callback function
void __stdcall IMUCallBackFunc(MV3D_RGBD_IMU_DATA* pstIMUData, void* pUser)
{
    if (NULL != pstIMUData)
    {
        g_imuData.updateData(
            pstIMUData->fXAccSpeed, pstIMUData->fYAccSpeed, pstIMUData->fZAccSpeed,
            pstIMUData->fXAngSpeed, pstIMUData->fYAngSpeed, pstIMUData->fZAngSpeed
        );
    }
}

class SLAMPointCloudAccumulator {
private:
    void* m_handle;
    bool m_bDeviceOpen;
    bool m_bGrabbing;
    int m_frameCount;

    // Display resolution settings
    struct DisplayResolution {
        int width;
        int height;
        std::string name;
    };

    std::vector<DisplayResolution> m_resolutions;
    int m_selectedResolution;

    // SLAM components
    open3d::camera::PinholeCameraIntrinsic m_intrinsics;
    bool m_intrinsicsInitialized;

    // SLAM state
    std::deque<SLAMKeyframe> m_keyframes;
    std::shared_ptr<open3d::geometry::PointCloud> m_globalMap;
    SLAMPose m_currentPose;

    // Visual odometry
    cv::Ptr<cv::ORB> m_orb_detector;
    cv::Ptr<cv::BFMatcher> m_matcher;

    // Point cloud parameters
    float m_depthScale;
    float m_maxDepth;
    double m_voxelSize;

    // Visualization
    std::shared_ptr<open3d::visualization::Visualizer> m_mapVisualizer;
    std::shared_ptr<open3d::visualization::Visualizer> m_liveVisualizer;
    std::mutex m_mapMutex;
    bool m_showLivePointCloud;
    bool m_showAccumulatedMap;

    // SLAM parameters
    int m_maxKeyframes;
    double m_keyframeDistanceThreshold;  // meters
    double m_keyframeAngleThreshold;     // radians
    int m_minFeatureMatches;

    // IMU integration (simplified VIO)
    std::deque<IMUData> m_imuBuffer;
    std::chrono::high_resolution_clock::time_point m_lastImuTime;
    bool m_useIMUPrediction;

public:
    SLAMPointCloudAccumulator() :
        m_handle(nullptr),
        m_bDeviceOpen(false),
        m_bGrabbing(false),
        m_frameCount(0),
        m_selectedResolution(0),
        m_intrinsicsInitialized(false),
        m_depthScale(1000.0f),
        m_maxDepth(4.0f),
        m_voxelSize(0.02),  // 2cm voxels
        m_showLivePointCloud(true),
        m_showAccumulatedMap(true),
        m_maxKeyframes(50),
        m_keyframeDistanceThreshold(0.3),  // 30cm
        m_keyframeAngleThreshold(0.3),     // ~17 degrees
        m_minFeatureMatches(50),
        m_useIMUPrediction(true)
    {
        // Initialize available display resolutions
        m_resolutions = {
            {426, 240, "426x240 (16:9 Small)"},
            {640, 360, "640x360 (16:9 Medium)"},
            {854, 480, "854x480 (16:9 480p)"},
            {1280, 720, "1280x720 (16:9 720p HD)"},
            {1920, 1080, "1920x1080 (16:9 1080p Full HD)"},
            {-1, -1, "Original Size (No Resize)"}
        };

        m_globalMap = std::make_shared<open3d::geometry::PointCloud>();

        // Initialize ORB detector for visual odometry
        m_orb_detector = cv::ORB::create(1000);  // 1000 features
        m_matcher = cv::BFMatcher::create(cv::NORM_HAMMING, true);

        m_lastImuTime = std::chrono::high_resolution_clock::now();
    }

    ~SLAMPointCloudAccumulator() {
        if (m_bGrabbing) {
            StopGrabbing();
        }
        if (m_bDeviceOpen) {
            CloseDevice();
        }
        if (m_mapVisualizer) {
            m_mapVisualizer->DestroyVisualizerWindow();
        }
        if (m_liveVisualizer) {
            m_liveVisualizer->DestroyVisualizerWindow();
        }
        MV3D_RGBD_Release();
        cv::destroyAllWindows();
    }

    int SelectSLAMParameters() {
        std::cout << "\n=== SLAM Point Cloud Accumulator Configuration ===" << std::endl;

        // Display resolution selection
        std::cout << "Choose the display resolution for the viewer windows:" << std::endl;
        for (int i = 0; i < static_cast<int>(m_resolutions.size()); i++) {
            std::cout << "[" << i << "] " << m_resolutions[i].name << std::endl;
        }

        std::cout << "\nEnter your choice (0-" << (m_resolutions.size() - 1) << "): ";
        int choice;
        std::cin >> choice;

        if (choice >= 0 && choice < static_cast<int>(m_resolutions.size())) {
            m_selectedResolution = choice;
            std::cout << "Selected: " << m_resolutions[choice].name << std::endl;
        }
        else {
            std::cerr << "Invalid choice!" << std::endl;
            return MV3D_RGBD_E_PARAMETER;
        }

        // SLAM configuration
        char enableLive, enableMap, useIMU;
        std::cout << "\n=== SLAM Configuration ===" << std::endl;
        std::cout << "Show live point cloud? (y/n): ";
        std::cin >> enableLive;
        m_showLivePointCloud = (enableLive == 'y' || enableLive == 'Y');

        std::cout << "Show accumulated SLAM map? (y/n): ";
        std::cin >> enableMap;
        m_showAccumulatedMap = (enableMap == 'y' || enableMap == 'Y');

        std::cout << "Use IMU for motion prediction? (y/n): ";
        std::cin >> useIMU;
        m_useIMUPrediction = (useIMU == 'y' || useIMU == 'Y');

        std::cout << "\n=== SLAM Parameters ===" << std::endl;
        std::cout << "Max keyframes: " << m_maxKeyframes << std::endl;
        std::cout << "Keyframe distance threshold: " << m_keyframeDistanceThreshold << "m" << std::endl;
        std::cout << "Voxel size for downsampling: " << m_voxelSize << "m" << std::endl;
        std::cout << "Using " << (m_useIMUPrediction ? "Visual-Inertial" : "Visual-only") << " odometry" << std::endl;

        return MV3D_RGBD_OK;
    }

    int Initialize() {
        int nRet = MV3D_RGBD_Initialize();
        if (MV3D_RGBD_OK != nRet) {
            std::cerr << "Failed to initialize SDK: " << nRet << std::endl;
            return nRet;
        }

        // Get SDK version
        MV3D_RGBD_VERSION_INFO stVersion;
        nRet = MV3D_RGBD_GetSDKVersion(&stVersion);
        if (MV3D_RGBD_OK == nRet) {
            std::cout << "SDK Version: " << stVersion.nMajor << "." << stVersion.nMinor << "." << stVersion.nRevision << std::endl;
        }

        return MV3D_RGBD_OK;
    }

    int EnumerateDevices(std::vector<MV3D_RGBD_DEVICE_INFO>& deviceList) {
        unsigned int nDevNum = 0;
        int nRet = MV3D_RGBD_GetDeviceNumber(DeviceType_Ethernet | DeviceType_USB, &nDevNum);
        if (MV3D_RGBD_OK != nRet || nDevNum == 0) {
            std::cerr << "No devices found!" << std::endl;
            return nRet;
        }

        deviceList.resize(nDevNum);
        nRet = MV3D_RGBD_GetDeviceList(DeviceType_Ethernet | DeviceType_USB, &deviceList[0], nDevNum, &nDevNum);
        if (MV3D_RGBD_OK != nRet) {
            std::cerr << "Failed to get device list!" << std::endl;
            return nRet;
        }

        std::cout << "\nFound " << nDevNum << " device(s):" << std::endl;
        for (unsigned int i = 0; i < nDevNum; i++) {
            if (DeviceType_Ethernet == deviceList[i].enDeviceType) {
                std::cout << "[" << i << "] Serial: " << deviceList[i].chSerialNumber
                    << " IP: " << deviceList[i].SpecialInfo.stNetInfo.chCurrentIp
                    << " Model: " << deviceList[i].chModelName << std::endl;
            }
            else if (DeviceType_USB == deviceList[i].enDeviceType) {
                std::cout << "[" << i << "] Serial: " << deviceList[i].chSerialNumber
                    << " USB Protocol: " << deviceList[i].SpecialInfo.stUsbInfo.enUsbProtocol
                    << " Model: " << deviceList[i].chModelName << std::endl;
            }
        }

        return MV3D_RGBD_OK;
    }

    int OpenDevice(const MV3D_RGBD_DEVICE_INFO& deviceInfo) {
        MV3D_RGBD_DEVICE_INFO deviceInfoCopy = deviceInfo;
        int nRet = MV3D_RGBD_OpenDevice(&m_handle, &deviceInfoCopy);
        if (MV3D_RGBD_OK != nRet) {
            std::cerr << "Failed to open device: " << nRet << std::endl;
            return nRet;
        }

        m_bDeviceOpen = true;

        // Setup IMU callback
        MV3D_RGBD_PARAM stParam;
        stParam.enParamType = ParamType_Enum;
        stParam.ParamInfo.stEnumParam.nCurValue = 1;
        nRet = MV3D_RGBD_SetParam(m_handle, "EventNotification", &stParam);
        if (MV3D_RGBD_OK == nRet) {
            nRet = MV3D_RGBD_RegisterIMUDataCallBack(m_handle, IMUCallBackFunc, m_handle);
            if (MV3D_RGBD_OK == nRet) {
                std::cout << "IMU callback registered successfully" << std::endl;
            }
            else {
                std::cerr << "Failed to register IMU callback: " << nRet << std::endl;
            }
        }
        else {
            std::cerr << "Failed to set EventNotification: " << nRet << std::endl;
        }

        std::cout << "Device opened successfully." << std::endl;

        // Initialize camera intrinsics
        InitializeCameraIntrinsics();

        return MV3D_RGBD_OK;
    }

    void InitializeCameraIntrinsics() {
        // Default intrinsics (replace with actual camera calibration)
        float fx = 525.0f, fy = 525.0f, cx = 320.0f, cy = 240.0f;
        int width = 640, height = 480;

        // Use the selected resolution
        if (m_selectedResolution < static_cast<int>(m_resolutions.size()) - 1) {
            width = m_resolutions[m_selectedResolution].width;
            height = m_resolutions[m_selectedResolution].height;

            // Scale intrinsics proportionally
            cx = width / 2.0f;
            cy = height / 2.0f;
            fx = width * 525.0f / 640.0f;
            fy = height * 525.0f / 480.0f;
        }

        m_intrinsics.SetIntrinsics(width, height, fx, fy, cx, cy);
        m_intrinsicsInitialized = true;

        std::cout << "Camera intrinsics initialized: " << width << "x" << height
            << " fx=" << fx << " fy=" << fy << " cx=" << cx << " cy=" << cy << std::endl;
    }

    int StartGrabbing() {
        if (!m_bDeviceOpen) {
            std::cerr << "Device not opened!" << std::endl;
            return MV3D_RGBD_E_HANDLE;
        }

        int nRet = MV3D_RGBD_Start(m_handle);
        if (MV3D_RGBD_OK != nRet) {
            std::cerr << "Failed to start grabbing: " << nRet << std::endl;
            return nRet;
        }

        m_bGrabbing = true;
        std::cout << "Started grabbing successfully." << std::endl;

        // Initialize visualizers
        if (m_showLivePointCloud) {
            InitializeLiveVisualizer();
        }

        if (m_showAccumulatedMap) {
            InitializeMapVisualizer();
        }

        return MV3D_RGBD_OK;
    }

    void InitializeLiveVisualizer() {
        try {
            m_liveVisualizer = std::make_shared<open3d::visualization::Visualizer>();
            m_liveVisualizer->CreateVisualizerWindow("Live Point Cloud", 800, 600);

            auto& view_control = m_liveVisualizer->GetViewControl();
            view_control.SetFront(Eigen::Vector3d(0, 0, 1));
            view_control.SetUp(Eigen::Vector3d(0, -1, 0));
            view_control.SetLookat(Eigen::Vector3d(0, 0, 2));

            std::cout << "Live point cloud visualizer initialized." << std::endl;
        }
        catch (const std::exception& e) {
            std::cerr << "Failed to initialize live visualizer: " << e.what() << std::endl;
            m_showLivePointCloud = false;
        }
    }

    void InitializeMapVisualizer() {
        try {
            m_mapVisualizer = std::make_shared<open3d::visualization::Visualizer>();
            m_mapVisualizer->CreateVisualizerWindow("SLAM Accumulated Map", 1000, 800);

            auto& view_control = m_mapVisualizer->GetViewControl();
            view_control.SetFront(Eigen::Vector3d(0, 0, 1));
            view_control.SetUp(Eigen::Vector3d(0, -1, 0));
            view_control.SetLookat(Eigen::Vector3d(0, 0, 0));

            // Add coordinate frame
            auto coordinate_frame = open3d::geometry::TriangleMesh::CreateCoordinateFrame(0.3);
            m_mapVisualizer->AddGeometry(coordinate_frame);

            std::cout << "SLAM map visualizer initialized." << std::endl;
        }
        catch (const std::exception& e) {
            std::cerr << "Failed to initialize map visualizer: " << e.what() << std::endl;
            m_showAccumulatedMap = false;
        }
    }

    int StopGrabbing() {
        if (!m_bGrabbing) {
            return MV3D_RGBD_OK;
        }

        int nRet = MV3D_RGBD_Stop(m_handle);
        if (MV3D_RGBD_OK != nRet) {
            std::cerr << "Failed to stop grabbing: " << nRet << std::endl;
            return nRet;
        }

        m_bGrabbing = false;
        std::cout << "Stopped grabbing successfully." << std::endl;
        return MV3D_RGBD_OK;
    }

    int CloseDevice() {
        if (m_handle) {
            int nRet = MV3D_RGBD_CloseDevice(&m_handle);
            if (MV3D_RGBD_OK != nRet) {
                std::cerr << "Failed to close device: " << nRet << std::endl;
                return nRet;
            }
            m_handle = nullptr;
            m_bDeviceOpen = false;
            std::cout << "Device closed successfully." << std::endl;
        }
        return MV3D_RGBD_OK;
    }

    std::shared_ptr<open3d::geometry::PointCloud> GeneratePointCloud(const cv::Mat& rgbFrame, const cv::Mat& depthFrame) {
        if (!m_intrinsicsInitialized || rgbFrame.empty() || depthFrame.empty()) {
            return nullptr;
        }

        try {
            // Convert to Open3D images
            open3d::geometry::Image colorImage, depthImage;

            // Convert BGR to RGB
            cv::Mat rgbConverted;
            cv::cvtColor(rgbFrame, rgbConverted, cv::COLOR_BGR2RGB);

            // Prepare color image
            colorImage.Prepare(rgbFrame.cols, rgbFrame.rows, 3, sizeof(uint8_t));
            std::memcpy(colorImage.data_.data(), rgbConverted.data,
                rgbConverted.total() * rgbConverted.elemSize());

            // Convert depth to float
            cv::Mat depthFloat;
            depthFrame.convertTo(depthFloat, CV_32F, 1.0 / m_depthScale);

            depthImage.Prepare(depthFrame.cols, depthFrame.rows, 1, sizeof(float));
            std::memcpy(depthImage.data_.data(), depthFloat.data,
                depthFloat.total() * depthFloat.elemSize());

            // Create RGBD image
            auto rgbd = open3d::geometry::RGBDImage::CreateFromColorAndDepth(
                colorImage, depthImage, 1.0, m_maxDepth, false);

            // Generate point cloud
            auto pointCloud = open3d::geometry::PointCloud::CreateFromRGBDImage(*rgbd, m_intrinsics);

            // Apply coordinate system correction
            Eigen::Matrix4d correction = Eigen::Matrix4d::Identity();
            correction(1, 1) = -1; // Flip Y
            correction(2, 2) = -1; // Flip Z
            pointCloud->Transform(correction);

            // Remove outliers and downsample
            if (pointCloud->points_.size() > 100) {
                // Statistical outlier removal
                auto filtered_result = pointCloud->RemoveStatisticalOutliers(20, 2.0);
                pointCloud = std::get<0>(filtered_result);  // Get the filtered cloud

                // Voxel downsampling
                if (pointCloud->points_.size() > 1000) {
                    pointCloud = pointCloud->VoxelDownSample(m_voxelSize);
                }
            }

            return pointCloud;
        }
        catch (const std::exception& e) {
            std::cerr << "Point cloud generation error: " << e.what() << std::endl;
            return nullptr;
        }
    }

    // Visual Odometry using ORB features
    bool EstimateMotion(const cv::Mat& prevFrame, const cv::Mat& currentFrame, SLAMPose& relativePose) {
        if (prevFrame.empty() || currentFrame.empty()) {
            return false;
        }

        // Detect ORB features
        std::vector<cv::KeyPoint> kp1, kp2;
        cv::Mat desc1, desc2;

        m_orb_detector->detectAndCompute(prevFrame, cv::noArray(), kp1, desc1);
        m_orb_detector->detectAndCompute(currentFrame, cv::noArray(), kp2, desc2);

        if (kp1.size() < 50 || kp2.size() < 50) {
            return false;
        }

        // Match features
        std::vector<cv::DMatch> matches;
        m_matcher->match(desc1, desc2, matches);

        // Filter matches
        if (matches.size() < m_minFeatureMatches) {
            return false;
        }

        // Sort matches by distance
        std::sort(matches.begin(), matches.end(),
            [](const cv::DMatch& a, const cv::DMatch& b) { return a.distance < b.distance; });

        // Keep only good matches (top 70%)
        matches.resize(static_cast<size_t>(matches.size() * 0.7));

        // Extract matched points
        std::vector<cv::Point2f> pts1, pts2;
        for (const auto& match : matches) {
            pts1.push_back(kp1[match.queryIdx].pt);
            pts2.push_back(kp2[match.trainIdx].pt);
        }

        // Estimate essential matrix
        cv::Mat E, mask;
        cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) <<
            m_intrinsics.GetFocalLength().first, 0, m_intrinsics.GetPrincipalPoint().first,
            0, m_intrinsics.GetFocalLength().second, m_intrinsics.GetPrincipalPoint().second,
            0, 0, 1);

        E = cv::findEssentialMat(pts1, pts2, cameraMatrix, cv::RANSAC, 0.999, 1.0, mask);

        // Recover pose
        cv::Mat R, t;
        int inliers = cv::recoverPose(E, pts1, pts2, cameraMatrix, R, t, mask);

        if (inliers < m_minFeatureMatches / 2) {
            return false;
        }

        // Convert to Eigen
        Eigen::Matrix3d rotation;
        Eigen::Vector3d translation;

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                rotation(i, j) = R.at<double>(i, j);
            }
            translation(i) = t.at<double>(i, 0);
        }

        relativePose.rotation = Eigen::Quaterniond(rotation);
        relativePose.translation = translation;
        relativePose.confidence = static_cast<double>(inliers) / matches.size();
        relativePose.timestamp = std::chrono::high_resolution_clock::now();

        return true;
    }

    // IMU integration for motion prediction (simplified)
    SLAMPose PredictMotionWithIMU(const SLAMPose& lastPose) {
        if (!m_useIMUPrediction) {
            return lastPose;
        }

        float xa, ya, za, xg, yg, zg;
        bool hasNewData;
        g_imuData.getData(xa, ya, za, xg, yg, zg, hasNewData);

        if (!hasNewData) {
            return lastPose;
        }

        // Simple IMU integration (in real SLAM, this would be much more sophisticated)
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto dt = std::chrono::duration<double>(currentTime - m_lastImuTime).count();
        m_lastImuTime = currentTime;

        if (dt > 0.1 || dt <= 0) {  // Ignore large gaps or invalid time
            return lastPose;
        }

        SLAMPose predictedPose = lastPose;

        // Simple integration (this is a very basic approach)
        // In real VIO, you'd use proper IMU preintegration
        Eigen::Vector3d imuAccel(xa, ya, za);
        Eigen::Vector3d imuGyro(xg, yg, zg);

        // Rotate acceleration to world frame
        Eigen::Vector3d worldAccel = lastPose.rotation * imuAccel;

        // Simple motion model (constant acceleration)
        predictedPose.translation += worldAccel * dt * dt * 0.5;

        // Integrate angular velocity
        Eigen::Vector3d deltaAngle = imuGyro * dt;
        if (deltaAngle.norm() > 0) {
            Eigen::Quaterniond deltaRotation(Eigen::AngleAxisd(deltaAngle.norm(), deltaAngle.normalized()));
            predictedPose.rotation = lastPose.rotation * deltaRotation;
            predictedPose.rotation.normalize();
        }

        return predictedPose;
    }

    bool ShouldCreateKeyframe(const SLAMPose& currentPose) {
        if (m_keyframes.empty()) {
            return true;  // First keyframe
        }

        if (m_keyframes.size() >= m_maxKeyframes) {
            // Remove oldest keyframe to maintain memory
            m_keyframes.pop_front();
        }

        const SLAMPose& lastKeyframePose = m_keyframes.back().pose;

        // Check distance threshold
        double distance = (currentPose.translation - lastKeyframePose.translation).norm();
        if (distance > m_keyframeDistanceThreshold) {
            return true;
        }

        // Check rotation threshold
        double angle = lastKeyframePose.rotation.angularDistance(currentPose.rotation);
        if (angle > m_keyframeAngleThreshold) {
            return true;
        }

        return false;
    }

    void CreateKeyframe(const cv::Mat& rgbFrame, const cv::Mat& depthFrame,
        std::shared_ptr<open3d::geometry::PointCloud> pointCloud,
        const SLAMPose& pose) {
        SLAMKeyframe keyframe;
        keyframe.id = static_cast<int>(m_keyframes.size());
        keyframe.rgb_image = rgbFrame.clone();
        keyframe.depth_image = depthFrame.clone();
        keyframe.point_cloud = pointCloud;
        keyframe.pose = pose;
        keyframe.timestamp = std::chrono::high_resolution_clock::now();

        // Extract ORB features for this keyframe
        m_orb_detector->detectAndCompute(rgbFrame, cv::noArray(), keyframe.keypoints, keyframe.descriptors);

        m_keyframes.push_back(keyframe);

        // Transform point cloud to world coordinates and add to global map
        if (pointCloud && !pointCloud->points_.empty()) {
            auto transformedCloud = std::make_shared<open3d::geometry::PointCloud>(*pointCloud);
            transformedCloud->Transform(pose.toMatrix());

            std::lock_guard<std::mutex> lock(m_mapMutex);
            *m_globalMap += *transformedCloud;

            // Downsample global map to keep it manageable
            if (m_globalMap->points_.size() > 100000) {
                m_globalMap = m_globalMap->VoxelDownSample(m_voxelSize * 2);
            }
        }

        std::cout << "Created keyframe " << keyframe.id << " at pose: ("
            << pose.translation.x() << ", " << pose.translation.y() << ", " << pose.translation.z()
            << ") with " << (pointCloud ? pointCloud->points_.size() : 0) << " points" << std::endl;
    }

    void UpdateMapVisualization() {
        if (!m_showAccumulatedMap || !m_mapVisualizer) {
            return;
        }

        std::lock_guard<std::mutex> lock(m_mapMutex);

        try {
            m_mapVisualizer->ClearGeometries();

            // Add coordinate frame
            auto coordinate_frame = open3d::geometry::TriangleMesh::CreateCoordinateFrame(0.3);
            m_mapVisualizer->AddGeometry(coordinate_frame);

            // Add global map
            if (m_globalMap && !m_globalMap->points_.empty()) {
                m_mapVisualizer->AddGeometry(m_globalMap);
            }

            // Add camera trajectory
            if (m_keyframes.size() > 1) {
                auto trajectory = std::make_shared<open3d::geometry::LineSet>();
                std::vector<Eigen::Vector3d> points;
                std::vector<Eigen::Vector2i> lines;

                for (size_t i = 0; i < m_keyframes.size(); i++) {
                    points.push_back(m_keyframes[i].pose.translation);
                    if (i > 0) {
                        lines.push_back(Eigen::Vector2i(i - 1, i));
                    }
                }

                trajectory->points_ = points;
                trajectory->lines_ = lines;

                // Set trajectory color to red
                std::vector<Eigen::Vector3d> colors(lines.size(), Eigen::Vector3d(1.0, 0.0, 0.0));
                trajectory->colors_ = colors;

                m_mapVisualizer->AddGeometry(trajectory);
            }

            m_mapVisualizer->UpdateGeometry();
            m_mapVisualizer->PollEvents();
            m_mapVisualizer->UpdateRender();
        }
        catch (const std::exception& e) {
            std::cerr << "Map visualization error: " << e.what() << std::endl;
        }
    }

    void UpdateLiveVisualization(std::shared_ptr<open3d::geometry::PointCloud> pointCloud) {
        if (!m_showLivePointCloud || !m_liveVisualizer || !pointCloud) {
            return;
        }

        try {
            m_liveVisualizer->ClearGeometries();
            m_liveVisualizer->AddGeometry(pointCloud);
            m_liveVisualizer->UpdateGeometry(pointCloud);
            m_liveVisualizer->PollEvents();
            m_liveVisualizer->UpdateRender();
        }
        catch (const std::exception& e) {
            std::cerr << "Live visualization error: " << e.what() << std::endl;
        }
    }

    cv::Mat ConvertDepthToDisplay(const MV3D_RGBD_IMAGE_DATA& imageData) {
        cv::Mat depthMat(imageData.nHeight, imageData.nWidth, CV_16UC1, imageData.pData);
        cv::Mat depthDisplay;

        double minVal, maxVal;
        cv::minMaxLoc(depthMat, &minVal, &maxVal);

        if (maxVal > minVal) {
            depthMat.convertTo(depthDisplay, CV_8UC1, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));
        }
        else {
            depthDisplay = cv::Mat::zeros(imageData.nHeight, imageData.nWidth, CV_8UC1);
        }

        cv::Mat coloredDepth;
        cv::applyColorMap(depthDisplay, coloredDepth, cv::COLORMAP_JET);
        return coloredDepth;
    }

    cv::Mat ConvertColorToDisplay(const MV3D_RGBD_IMAGE_DATA& imageData) {
        if (ImageType_RGB8_Planar == imageData.enImageType) {
            cv::Mat rgbDisplay(imageData.nHeight, imageData.nWidth, CV_8UC3);
            unsigned char* srcData = (unsigned char*)imageData.pData;
            unsigned char* dstData = rgbDisplay.data;

            int pixelCount = imageData.nWidth * imageData.nHeight;
            for (int i = 0; i < pixelCount; i++) {
                dstData[i * 3 + 2] = srcData[i];
                dstData[i * 3 + 1] = srcData[i + pixelCount];
                dstData[i * 3 + 0] = srcData[i + 2 * pixelCount];
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

    void DrawSLAMInfo(cv::Mat& image) {
        // Get IMU data
        float xa, ya, za, xg, yg, zg;
        bool newData;
        g_imuData.getData(xa, ya, za, xg, yg, zg, newData);

        // Create SLAM status overlay
        std::vector<std::string> slamText;
        std::stringstream ss;

        slamText.push_back("=== SLAM Status ===");

        // IMU data
        ss.str(""); ss << "IMU Acc: (" << std::fixed << std::setprecision(2) << xa << ", " << ya << ", " << za << ")";
        slamText.push_back(ss.str());

        ss.str(""); ss << "IMU Gyro: (" << std::fixed << std::setprecision(2) << xg << ", " << yg << ", " << zg << ")";
        slamText.push_back(ss.str());

        // SLAM state
        ss.str(""); ss << "Keyframes: " << m_keyframes.size() << "/" << m_maxKeyframes;
        slamText.push_back(ss.str());

        ss.str(""); ss << "Map Points: " << m_globalMap->points_.size();
        slamText.push_back(ss.str());

        // Current pose
        ss.str(""); ss << "Pose: (" << std::fixed << std::setprecision(2)
            << m_currentPose.translation.x() << ", "
            << m_currentPose.translation.y() << ", "
            << m_currentPose.translation.z() << ")";
        slamText.push_back(ss.str());

        // Draw overlay
        int fontFace = cv::FONT_HERSHEY_SIMPLEX;
        double fontScale = 0.4;
        int thickness = 1;
        cv::Scalar textColor(0, 255, 0);
        cv::Scalar bgColor(0, 0, 0);

        for (int i = 0; i < static_cast<int>(slamText.size()); i++) {
            cv::Point textPos(15, 15 + i * 15);

            int baseline = 0;
            cv::Size textSize = cv::getTextSize(slamText[i], fontFace, fontScale, thickness, &baseline);

            cv::rectangle(image,
                cv::Point(textPos.x - 2, textPos.y - textSize.height - 2),
                cv::Point(textPos.x + textSize.width + 2, textPos.y + baseline + 2),
                bgColor, cv::FILLED);

            cv::Scalar color = (i == 0) ? cv::Scalar(255, 255, 0) : textColor;
            cv::putText(image, slamText[i], textPos, fontFace, fontScale, color, thickness);
        }

        // Add frame counter
        ss.str(""); ss << "F:" << m_frameCount;
        cv::putText(image, ss.str(), cv::Point(15, 25 + static_cast<int>(slamText.size()) * 15), fontFace, fontScale, cv::Scalar(255, 255, 255), thickness);
    }

    void ProcessFrameData(const MV3D_RGBD_FRAME_DATA& frameData) {
        cv::Mat rgbFrame, depthFrame;
        static cv::Mat prevRgbFrame;

        for (unsigned int i = 0; i < frameData.nImageCount; i++) {
            const MV3D_RGBD_IMAGE_DATA& imageData = frameData.stImageData[i];

            std::string windowName;
            cv::Mat displayMat;

            switch (imageData.enImageType) {
            case ImageType_Depth:
                windowName = "Depth Stream with SLAM";
                displayMat = ConvertDepthToDisplay(imageData);
                depthFrame = cv::Mat(imageData.nHeight, imageData.nWidth, CV_16UC1, imageData.pData).clone();
                break;

            case ImageType_RGB8_Planar:
            case ImageType_YUV422:
            case ImageType_YUV420SP_NV12:
            case ImageType_YUV420SP_NV21:
                windowName = "Color Stream with SLAM";
                displayMat = ConvertColorToDisplay(imageData);
                if (!displayMat.empty()) {
                    rgbFrame = displayMat.clone();
                }
                break;

            default:
                continue;
            }

            if (!displayMat.empty()) {
                DrawSLAMInfo(displayMat);
                cv::imshow(windowName, displayMat);
                m_frameCount++;
            }
        }

        // Perform SLAM processing if both RGB and depth are available
        if (!rgbFrame.empty() && !depthFrame.empty()) {
            ProcessSLAMFrame(rgbFrame, depthFrame, prevRgbFrame);
            prevRgbFrame = rgbFrame.clone();
        }
    }

    void ProcessSLAMFrame(const cv::Mat& rgbFrame, const cv::Mat& depthFrame, const cv::Mat& prevRgbFrame) {
        // Generate point cloud
        auto currentPointCloud = GeneratePointCloud(rgbFrame, depthFrame);
        if (!currentPointCloud) {
            return;
        }

        // Update live visualization
        UpdateLiveVisualization(currentPointCloud);

        // Motion estimation
        SLAMPose estimatedPose = m_currentPose;

        if (!prevRgbFrame.empty() && m_frameCount > 1) {
            // Try visual odometry
            SLAMPose relativePose;
            if (EstimateMotion(prevRgbFrame, rgbFrame, relativePose)) {
                // Apply relative motion
                estimatedPose.translation = m_currentPose.translation + m_currentPose.rotation * relativePose.translation;
                estimatedPose.rotation = m_currentPose.rotation * relativePose.rotation;
                estimatedPose.rotation.normalize();
                estimatedPose.confidence = relativePose.confidence;
            }
            else {
                // Fall back to IMU prediction
                estimatedPose = PredictMotionWithIMU(m_currentPose);
            }
        }

        // Update current pose
        m_currentPose = estimatedPose;

        // Check if we should create a keyframe
        if (ShouldCreateKeyframe(m_currentPose)) {
            CreateKeyframe(rgbFrame, depthFrame, currentPointCloud, m_currentPose);
            UpdateMapVisualization();
        }
    }

    void SaveMap(const std::string& filename) {
        std::lock_guard<std::mutex> lock(m_mapMutex);

        if (m_globalMap && !m_globalMap->points_.empty()) {
            bool success = open3d::io::WritePointCloud(filename, *m_globalMap);
            if (success) {
                std::cout << "Map saved to " << filename << " with " << m_globalMap->points_.size() << " points" << std::endl;
            }
            else {
                std::cerr << "Failed to save map to " << filename << std::endl;
            }
        }
        else {
            std::cerr << "No map data to save!" << std::endl;
        }
    }

    void SaveTrajectory(const std::string& filename) {
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Failed to open trajectory file: " << filename << std::endl;
            return;
        }

        file << "# Timestamp tx ty tz qx qy qz qw" << std::endl;
        for (const auto& keyframe : m_keyframes) {
            auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                keyframe.timestamp.time_since_epoch()).count();

            const auto& t = keyframe.pose.translation;
            const auto& q = keyframe.pose.rotation;

            file << timestamp << " "
                << t.x() << " " << t.y() << " " << t.z() << " "
                << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
        }

        file.close();
        std::cout << "Trajectory saved to " << filename << " with " << m_keyframes.size() << " poses" << std::endl;
    }

    int RunSLAM() {
        if (!m_bGrabbing) {
            std::cerr << "Grabbing not started!" << std::endl;
            return MV3D_RGBD_E_HANDLE;
        }

        std::cout << "\n=== SLAM Point Cloud Accumulator Started ===" << std::endl;
        std::cout << "Display Resolution: " << m_resolutions[m_selectedResolution].name << std::endl;
        std::cout << "Live Point Cloud: " << (m_showLivePointCloud ? "Enabled" : "Disabled") << std::endl;
        std::cout << "SLAM Map: " << (m_showAccumulatedMap ? "Enabled" : "Disabled") << std::endl;
        std::cout << "IMU Integration: " << (m_useIMUPrediction ? "Enabled" : "Disabled") << std::endl;
        std::cout << "\nControls:" << std::endl;
        std::cout << "  - Press 'q' on any window to quit" << std::endl;
        std::cout << "  - Press 'ESC' on any window to quit" << std::endl;
        std::cout << "  - Press 's' to save map and trajectory" << std::endl;
        std::cout << "  - Press 'r' to reset SLAM" << std::endl;
        std::cout << "========================================================" << std::endl;

        MV3D_RGBD_FRAME_DATA frameData = { 0 };

        while (true) {
            int nRet = MV3D_RGBD_FetchFrame(m_handle, &frameData, 100);
            if (MV3D_RGBD_OK == nRet) {
                if (!frameData.nValidInfo) {
                    ProcessFrameData(frameData);
                }
            }

            // Check for key press
            int key = cv::waitKey(1) & 0xFF;
            if (key == 'q' || key == 27) {
                break;
            }
            else if (key == 's') {
                // Save map and trajectory
                SaveMap("slam_map.pcd");
                SaveTrajectory("slam_trajectory.txt");
                std::cout << "Map and trajectory saved!" << std::endl;
            }
            else if (key == 'r') {
                // Reset SLAM
                m_keyframes.clear();
                m_globalMap->Clear();
                m_currentPose = SLAMPose();
                std::cout << "SLAM reset!" << std::endl;
            }

            // Also check console input
            if (KBHIT()) {
                break;
            }

            // Check if visualization windows were closed
            if (m_showLivePointCloud && m_liveVisualizer) {
                if (!m_liveVisualizer->PollEvents()) {
                    m_showLivePointCloud = false;
                    m_liveVisualizer.reset();
                }
            }

            if (m_showAccumulatedMap && m_mapVisualizer) {
                if (!m_mapVisualizer->PollEvents()) {
                    break;  // Exit if main map window is closed
                }
            }
        }

        // Final save
        SaveMap("final_slam_map.pcd");
        SaveTrajectory("final_slam_trajectory.txt");

        std::cout << "\nSLAM completed! Total frames: " << m_frameCount
            << ", Keyframes: " << m_keyframes.size()
            << ", Map points: " << m_globalMap->points_.size() << std::endl;
        return MV3D_RGBD_OK;
    }
};

int main(int argc, char* argv[]) {
    std::cout << "=== SLAM Point Cloud Accumulator with IMU ===" << std::endl;
    std::cout << "This program performs real-time SLAM using RGB-D camera and IMU data," << std::endl;
    std::cout << "accumulating point clouds into a 3D map using visual-inertial odometry." << std::endl;

    SLAMPointCloudAccumulator slam;

    // Configure SLAM parameters
    int nRet = slam.SelectSLAMParameters();
    if (MV3D_RGBD_OK != nRet) {
        return -1;
    }

    // Initialize SDK
    nRet = slam.Initialize();
    if (MV3D_RGBD_OK != nRet) {
        std::cerr << "Failed to initialize. Make sure the MV3D RGBD SDK is properly installed." << std::endl;
        return -1;
    }

    // Enumerate devices
    std::vector<MV3D_RGBD_DEVICE_INFO> deviceList;
    nRet = slam.EnumerateDevices(deviceList);
    if (MV3D_RGBD_OK != nRet || deviceList.empty()) {
        std::cerr << "No devices found. Make sure the camera is connected." << std::endl;
        return -1;
    }

    // Select device
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
    nRet = slam.OpenDevice(deviceList[deviceIndex]);
    if (MV3D_RGBD_OK != nRet) {
        return -1;
    }

    // Start grabbing
    nRet = slam.StartGrabbing();
    if (MV3D_RGBD_OK != nRet) {
        return -1;
    }

    // Run SLAM
    nRet = slam.RunSLAM();

    std::cout << "SLAM Point Cloud Accumulation completed!" << std::endl;
    return 0;
}