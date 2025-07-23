#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <mutex>
#include <memory>
#include <thread>

// Include the actual MV3D RGBD SDK headers from the correct path
#include "../common/common.hpp"
#include "Mv3dRgbdAdvancedApi.h"
#include "Mv3dRgbdAdvancedDefine.h"

// OpenCV for real-time display
#include <opencv2/opencv.hpp>

// Open3D for point cloud visualization
#include <Open3D/Open3D.h>

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

// Global IMU data structure with thread safety
struct IMUData {
    float xAcc, yAcc, zAcc;
    float xGyro, yGyro, zGyro;
    std::mutex dataMutex;
    bool hasNewData;

    IMUData() : xAcc(0), yAcc(0), zAcc(0), xGyro(0), yGyro(0), zGyro(0), hasNewData(false) {}

    void updateData(float xa, float ya, float za, float xg, float yg, float zg) {
        std::lock_guard<std::mutex> lock(dataMutex);
        xAcc = xa; yAcc = ya; zAcc = za;
        xGyro = xg; yGyro = yg; zGyro = zg;
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

class RGBDIMUPointCloudViewer {
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

    // Point cloud visualization
    std::shared_ptr<open3d::visualization::Visualizer> m_pointCloudVisualizer;
    std::shared_ptr<open3d::geometry::PointCloud> m_pointCloud;
    std::mutex m_pointCloudMutex;
    bool m_showPointCloud;

    // Camera intrinsics for point cloud generation
    open3d::camera::PinholeCameraIntrinsic m_intrinsics;
    bool m_intrinsicsInitialized;

    // Point cloud parameters
    float m_depthScale;
    float m_maxDepth;

public:
    RGBDIMUPointCloudViewer() :
        m_handle(nullptr),
        m_bDeviceOpen(false),
        m_bGrabbing(false),
        m_frameCount(0),
        m_selectedResolution(0),
        m_showPointCloud(false),
        m_intrinsicsInitialized(false),
        m_depthScale(1000.0f),  // Depth in mm
        m_maxDepth(4.0f)        // Max depth 4 meters
    {
        // Initialize available display resolutions - match original rgbd+imu code
        m_resolutions = {
            {426, 240, "426x240 (16:9 Small)"},
            {640, 360, "640x360 (16:9 Medium)"},
            {854, 480, "854x480 (16:9 480p)"},
            {1280, 720, "1280x720 (16:9 720p HD)"},
            {1920, 1080, "1920x1080 (16:9 1080p Full HD)"},
            {-1, -1, "Original Size (No Resize)"}
        };

        m_pointCloud = std::make_shared<open3d::geometry::PointCloud>();
    }

    ~RGBDIMUPointCloudViewer() {
        if (m_bGrabbing) {
            StopGrabbing();
        }
        if (m_bDeviceOpen) {
            CloseDevice();
        }
        if (m_pointCloudVisualizer) {
            m_pointCloudVisualizer->DestroyVisualizerWindow();
            m_pointCloudVisualizer.reset();
        }
        MV3D_RGBD_Release();
        cv::destroyAllWindows();
    }

    int SelectDisplayResolution() {
        std::cout << "\n=== Select Display Resolution ===" << std::endl;
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

            // Ask about point cloud visualization after resolution selection
            char enablePointCloud;
            std::cout << "Enable point cloud visualization? (y/n): ";
            std::cin >> enablePointCloud;
            m_showPointCloud = (enablePointCloud == 'y' || enablePointCloud == 'Y');

            return MV3D_RGBD_OK;
        }
        else {
            std::cerr << "Invalid choice!" << std::endl;
            return MV3D_RGBD_E_PARAMETER;
        }
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

    int Finalize() {
        if (m_pointCloudVisualizer) {
            m_pointCloudVisualizer->DestroyVisualizerWindow();
            m_pointCloudVisualizer.reset();
        }
        // Note: MV3D_RGBD_Finalize may not exist in some SDK versions
        // Return success for compatibility
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
        // Remove const to match API requirements
        MV3D_RGBD_DEVICE_INFO deviceInfoCopy = deviceInfo;
        int nRet = MV3D_RGBD_OpenDevice(&m_handle, &deviceInfoCopy);
        if (MV3D_RGBD_OK != nRet) {
            std::cerr << "Failed to open device: " << nRet << std::endl;
            return nRet;
        }

        m_bDeviceOpen = true;

        // Setup IMU callback - following original rgbd+imu implementation
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

        // Get camera parameters for point cloud generation
        if (m_showPointCloud) {
            InitializeCameraIntrinsics();
        }

        return MV3D_RGBD_OK;
    }

    void InitializeCameraIntrinsics() {
        // Try to get camera parameters from the device
        MV3D_RGBD_PARAM param;
        param.enParamType = ParamType_Float;

        // Default intrinsics (you may need to adjust these for your specific camera)
        // These are example values - replace with actual camera calibration
        float fx = 525.0f, fy = 525.0f, cx = 320.0f, cy = 240.0f;
        int width = 640, height = 480;

        // Use the selected resolution
        width = m_resolutions[m_selectedResolution].width;
        height = m_resolutions[m_selectedResolution].height;

        // Scale intrinsics proportionally
        cx = width / 2.0f;
        cy = height / 2.0f;
        fx = width * 525.0f / 640.0f;
        fy = height * 525.0f / 480.0f;

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

        // Initialize point cloud visualizer if enabled
        if (m_showPointCloud) {
            InitializePointCloudVisualizer();
        }

        return MV3D_RGBD_OK;
    }

    void InitializePointCloudVisualizer() {
        try {
            m_pointCloudVisualizer = std::make_shared<open3d::visualization::Visualizer>();
            m_pointCloudVisualizer->CreateVisualizerWindow("Point Cloud with IMU Data",
                m_resolutions[m_selectedResolution].width,
                m_resolutions[m_selectedResolution].height);

            // Set up the view
            auto& view_control = m_pointCloudVisualizer->GetViewControl();
            view_control.SetFront(Eigen::Vector3d(0, 0, 1));
            view_control.SetUp(Eigen::Vector3d(0, -1, 0));
            view_control.SetLookat(Eigen::Vector3d(0, 0, 2));

            std::cout << "Point cloud visualizer initialized." << std::endl;
        }
        catch (const std::exception& e) {
            std::cerr << "Failed to initialize point cloud visualizer: " << e.what() << std::endl;
            m_showPointCloud = false;
        }
    }

    int StopGrabbing() {
        if (!m_bGrabbing) {
            return MV3D_RGBD_OK;
        }

        int nRet = MV3D_RGBD_Stop(m_handle);
        if (MV3D_RGBD_OK != nRet) {
            std::cerr << "Failed to stop grabbing! Error: 0x" << std::hex << nRet << std::endl;
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

    cv::Mat ConvertDepthToDisplay(const MV3D_RGBD_IMAGE_DATA& imageData) {
        cv::Mat depthMat(imageData.nHeight, imageData.nWidth, CV_16UC1, imageData.pData);
        cv::Mat depthDisplay;

        // Normalize depth for better visualization
        double minVal, maxVal;
        cv::minMaxLoc(depthMat, &minVal, &maxVal);

        if (maxVal > minVal) {
            depthMat.convertTo(depthDisplay, CV_8UC1, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));
        }
        else {
            depthDisplay = cv::Mat::zeros(imageData.nHeight, imageData.nWidth, CV_8UC1);
        }

        // Apply colormap for better visualization
        cv::Mat coloredDepth;
        cv::applyColorMap(depthDisplay, coloredDepth, cv::COLORMAP_JET);

        return coloredDepth;
    }

    cv::Mat ConvertColorToDisplay(const MV3D_RGBD_IMAGE_DATA& imageData) {
        if (ImageType_RGB8_Planar == imageData.enImageType) {
            // RGB8 Planar format: RRRRR...GGGGG...BBBBB
            cv::Mat rgbDisplay(imageData.nHeight, imageData.nWidth, CV_8UC3);
            unsigned char* srcData = (unsigned char*)imageData.pData;
            unsigned char* dstData = rgbDisplay.data;

            int pixelCount = imageData.nWidth * imageData.nHeight;
            for (int i = 0; i < pixelCount; i++) {
                dstData[i * 3 + 2] = srcData[i];                    // R -> BGR format
                dstData[i * 3 + 1] = srcData[i + pixelCount];       // G
                dstData[i * 3 + 0] = srcData[i + 2 * pixelCount];   // B
            }
            return rgbDisplay;
        }
        else if (ImageType_YUV422 == imageData.enImageType) {
            // Convert YUV422 to RGB
            cv::Mat yuvMat(imageData.nHeight, imageData.nWidth, CV_8UC2, imageData.pData);
            cv::Mat rgbMat;
            cv::cvtColor(yuvMat, rgbMat, cv::COLOR_YUV2BGR_YUYV);
            return rgbMat;
        }

        // For other formats, return empty mat
        return cv::Mat();
    }

    cv::Mat ConvertIRToDisplay(const MV3D_RGBD_IMAGE_DATA& imageData) {
        if (ImageType_Mono8 == imageData.enImageType) {
            cv::Mat irMat(imageData.nHeight, imageData.nWidth, CV_8UC1, imageData.pData);
            return irMat;
        }
        else if (ImageType_Mono16 == imageData.enImageType) {
            cv::Mat irMat16(imageData.nHeight, imageData.nWidth, CV_16UC1, imageData.pData);
            cv::Mat irMat8;
            irMat16.convertTo(irMat8, CV_8UC1, 1.0 / 256.0);
            return irMat8;
        }

        return cv::Mat();
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

            // Convert depth to float (assuming input is in millimeters)
            cv::Mat depthFloat;
            depthFrame.convertTo(depthFloat, CV_32F, 1.0 / m_depthScale); // mm to meters

            depthImage.Prepare(depthFrame.cols, depthFrame.rows, 1, sizeof(float));
            std::memcpy(depthImage.data_.data(), depthFloat.data,
                depthFloat.total() * depthFloat.elemSize());

            // Create RGBD image
            auto rgbd = open3d::geometry::RGBDImage::CreateFromColorAndDepth(
                colorImage, depthImage, 1.0, m_maxDepth, false);

            // Generate point cloud
            auto pointCloud = open3d::geometry::PointCloud::CreateFromRGBDImage(*rgbd, m_intrinsics);

            // Apply coordinate system correction (following test_scanner pattern)
            Eigen::Matrix4d correction = Eigen::Matrix4d::Identity();
            correction(1, 1) = -1; // Flip Y
            correction(2, 2) = -1; // Flip Z
            pointCloud->Transform(correction);

            // Simple filtering
            if (pointCloud->points_.size() > 1000) {
                pointCloud = pointCloud->VoxelDownSample(0.01); // 1cm voxels
            }

            return pointCloud;
        }
        catch (const std::exception& e) {
            std::cerr << "Point cloud generation error: " << e.what() << std::endl;
            return nullptr;
        }
    }

    void UpdatePointCloudVisualization(std::shared_ptr<open3d::geometry::PointCloud> pointCloud) {
        if (!m_showPointCloud || !m_pointCloudVisualizer || !pointCloud) {
            return;
        }

        std::lock_guard<std::mutex> lock(m_pointCloudMutex);

        try {
            m_pointCloudVisualizer->ClearGeometries();
            m_pointCloudVisualizer->AddGeometry(pointCloud);
            m_pointCloudVisualizer->UpdateGeometry(pointCloud);
            m_pointCloudVisualizer->PollEvents();
            m_pointCloudVisualizer->UpdateRender();
        }
        catch (const std::exception& e) {
            std::cerr << "Point cloud visualization error: " << e.what() << std::endl;
        }
    }

    void DrawIMUInfo(cv::Mat& image) {
        float xa, ya, za, xg, yg, zg;
        bool newData;
        g_imuData.getData(xa, ya, za, xg, yg, zg, newData);

        // Create IMU overlay text
        std::vector<std::string> imuText;
        std::stringstream ss;

        imuText.push_back("=== IMU Data ===");

        ss.str(""); ss << "Acc: (" << std::fixed << std::setprecision(2) << xa << ", " << ya << ", " << za << ")";
        imuText.push_back(ss.str());

        ss.str(""); ss << "Gyro: (" << std::fixed << std::setprecision(2) << xg << ", " << yg << ", " << zg << ")";
        imuText.push_back(ss.str());

        if (m_showPointCloud) {
            ss.str(""); ss << "PointCloud: ON";
            imuText.push_back(ss.str());
        }

        // Draw text with background for better readability
        int fontFace = cv::FONT_HERSHEY_SIMPLEX;
        double fontScale = 0.4;
        int thickness = 1;
        cv::Scalar textColor(0, 255, 0); // Green
        cv::Scalar bgColor(0, 0, 0);     // Black background

        for (int i = 0; i < static_cast<int>(imuText.size()); i++) {
            cv::Point textPos(15, 15 + i * 15);

            // Get text size for background rectangle
            int baseline = 0;
            cv::Size textSize = cv::getTextSize(imuText[i], fontFace, fontScale, thickness, &baseline);

            // Draw background rectangle
            cv::rectangle(image,
                cv::Point(textPos.x - 2, textPos.y - textSize.height - 2),
                cv::Point(textPos.x + textSize.width + 2, textPos.y + baseline + 2),
                bgColor, cv::FILLED);

            // Draw text
            cv::Scalar color = (i == 0) ? cv::Scalar(255, 255, 0) : textColor; // Title in yellow
            cv::putText(image, imuText[i], textPos, fontFace, fontScale, color, thickness);
        }

        // Add frame counter - match original format
        ss.str(""); ss << "F:" << m_frameCount;  // Shorter format
        cv::putText(image, ss.str(), cv::Point(15, 25 + static_cast<int>(imuText.size()) * 15), fontFace, fontScale, cv::Scalar(255, 255, 255), thickness);
    }

    void ProcessFrameData(const MV3D_RGBD_FRAME_DATA& frameData) {
        cv::Mat rgbFrame, depthFrame;

        for (unsigned int i = 0; i < frameData.nImageCount; i++) {
            const MV3D_RGBD_IMAGE_DATA& imageData = frameData.stImageData[i];

            std::string windowName;
            cv::Mat displayMat;

            switch (imageData.enImageType) {
            case ImageType_Depth:
                windowName = "Depth Stream with IMU";
                displayMat = ConvertDepthToDisplay(imageData);
                depthFrame = cv::Mat(imageData.nHeight, imageData.nWidth, CV_16UC1, imageData.pData).clone();
                break;

            case ImageType_RGB8_Planar:
            case ImageType_YUV422:
            case ImageType_YUV420SP_NV12:
            case ImageType_YUV420SP_NV21:
                windowName = "Color Stream with IMU";
                displayMat = ConvertColorToDisplay(imageData);
                if (!displayMat.empty()) {
                    rgbFrame = displayMat.clone();
                }
                break;

            case ImageType_Mono8:
            case ImageType_Mono16:
                windowName = "IR Stream with IMU";
                displayMat = ConvertIRToDisplay(imageData);
                break;

            default:
                // Skip unsupported formats
                continue;
            }

            if (!displayMat.empty()) {
                // Draw IMU information overlay
                DrawIMUInfo(displayMat);

                cv::imshow(windowName, displayMat);
                m_frameCount++;
            }
        }

        // Generate and update point cloud if both RGB and depth are available
        if (m_showPointCloud && !rgbFrame.empty() && !depthFrame.empty()) {
            auto pointCloud = GeneratePointCloud(rgbFrame, depthFrame);
            if (pointCloud) {
                UpdatePointCloudVisualization(pointCloud);
            }
        }
    }

    int RunViewer() {
        if (!m_bGrabbing) {
            std::cerr << "Grabbing not started!" << std::endl;
            return MV3D_RGBD_E_HANDLE;
        }

        std::cout << "\n=== Real-Time Viewer with IMU and Point Cloud Started ===" << std::endl;
        std::cout << "Display Resolution: " << m_resolutions[m_selectedResolution].name << std::endl;
        std::cout << "Point Cloud: " << (m_showPointCloud ? "Enabled" : "Disabled") << std::endl;
        std::cout << "\nControls:" << std::endl;
        std::cout << "  - Press 'q' on any window to quit" << std::endl;
        std::cout << "  - Press 'ESC' on any window to quit" << std::endl;
        std::cout << "  - Press 'r' to change resolution" << std::endl;
        std::cout << "  - Press 'p' to toggle point cloud" << std::endl;
        std::cout << "=============================================================" << std::endl;

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
            if (key == 'q' || key == 27) { // 'q' or ESC
                break;
            }
            else if (key == 'r') {
                // Change resolution
                cv::destroyAllWindows();
                SelectDisplayResolution();
                std::cout << "Resolution changed to: " << m_resolutions[m_selectedResolution].name << std::endl;
                if (m_showPointCloud) {
                    InitializeCameraIntrinsics();
                }
            }
            else if (key == 'p') {
                // Toggle point cloud
                m_showPointCloud = !m_showPointCloud;
                std::cout << "Point cloud " << (m_showPointCloud ? "enabled" : "disabled") << std::endl;

                if (m_showPointCloud && !m_pointCloudVisualizer) {
                    InitializePointCloudVisualizer();
                    InitializeCameraIntrinsics();
                }
                else if (!m_showPointCloud && m_pointCloudVisualizer) {
                    m_pointCloudVisualizer->DestroyVisualizerWindow();
                    m_pointCloudVisualizer.reset();
                }
            }

            // Also check console input (for systems without window focus)
            if (KBHIT()) {
                break;
            }

            // Check if point cloud window was closed
            if (m_showPointCloud && m_pointCloudVisualizer) {
                if (!m_pointCloudVisualizer->PollEvents()) {
                    // Point cloud window was closed, disable point cloud
                    m_showPointCloud = false;
                    m_pointCloudVisualizer.reset();
                    std::cout << "Point cloud window closed." << std::endl;
                }
            }
        }

        std::cout << "\nViewer stopped. Total frames displayed: " << m_frameCount << std::endl;
        return MV3D_RGBD_OK;
    }
};

int main(int argc, char* argv[]) {
    std::cout << "=== RGBD, IR Real-Time Viewer with IMU Data and Point Cloud ===" << std::endl;
    std::cout << "This program displays RGBD and IR streams with IMU data overlay and optional point cloud visualization." << std::endl;

    RGBDIMUPointCloudViewer viewer;

    // Select display resolution and point cloud option
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

    // Run real-time viewer with IMU overlay and point cloud
    nRet = viewer.RunViewer();

    std::cout << "Real-time viewing with IMU data and point cloud completed!" << std::endl;
    return 0;
}