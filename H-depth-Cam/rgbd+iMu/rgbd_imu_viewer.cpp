#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <mutex>

// Include the actual MV3D RGBD SDK headers from the correct path
#include "../common/common.hpp"
#include "Mv3dRgbdAdvancedApi.h"
#include "Mv3dRgbdAdvancedDefine.h"

// OpenCV for real-time display
#include <opencv2/opencv.hpp>

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

class RGBDIRViewer {
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

public:
    RGBDIRViewer() : m_handle(nullptr), m_bDeviceOpen(false), m_bGrabbing(false), m_frameCount(0), m_selectedResolution(0) {
        // Initialize available display resolutions - all 16:9 aspect ratio
        m_resolutions = {
            {426, 240, "426x240 (16:9 Small)"},
            {640, 360, "640x360 (16:9 Medium)"},
            {854, 480, "854x480 (16:9 480p)"},
            {1280, 720, "1280x720 (16:9 720p HD)"},
            {1920, 1080, "1920x1080 (16:9 1080p Full HD)"},
            {-1, -1, "Original Size (No Resize)"}
        };
    }

    ~RGBDIRViewer() {
        if (m_bGrabbing) {
            StopGrabbing();
        }
        if (m_bDeviceOpen) {
            CloseDevice();
        }
        MV3D_RGBD_Release();
        cv::destroyAllWindows();
    }

    int SelectDisplayResolution() {
        std::cout << "\n=== Select Display Resolution ===" << std::endl;
        std::cout << "Choose the display resolution for the viewer windows:" << std::endl;

        for (size_t i = 0; i < m_resolutions.size(); i++) {
            std::cout << "[" << i << "] " << m_resolutions[i].name << std::endl;
        }

        std::cout << "\nEnter your choice (0-" << (m_resolutions.size() - 1) << "): ";

        int choice;
        std::cin >> choice;

        if (choice >= 0 && choice < static_cast<int>(m_resolutions.size())) {
            m_selectedResolution = choice;
            std::cout << "Selected: " << m_resolutions[choice].name << std::endl;
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
        int nRet = MV3D_RGBD_OpenDevice(&m_handle, const_cast<MV3D_RGBD_DEVICE_INFO*>(&deviceInfo));
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
        return MV3D_RGBD_OK;
    }

    int StopGrabbing() {
        if (m_bGrabbing) {
            int nRet = MV3D_RGBD_Stop(m_handle);
            if (MV3D_RGBD_OK != nRet) {
                std::cerr << "Failed to stop grabbing: " << nRet << std::endl;
                return nRet;
            }
            m_bGrabbing = false;
            std::cout << "Stopped grabbing successfully." << std::endl;
        }
        return MV3D_RGBD_OK;
    }

    cv::Mat ResizeForDisplay(const cv::Mat& image) {
        if (m_selectedResolution == m_resolutions.size() - 1) {
            // Original size
            return image;
        }

        int targetWidth = m_resolutions[m_selectedResolution].width;
        int targetHeight = m_resolutions[m_selectedResolution].height;

        cv::Mat resized;
        cv::resize(image, resized, cv::Size(targetWidth, targetHeight));
        return resized;
    }

    cv::Mat ConvertDepthToDisplay(const MV3D_RGBD_IMAGE_DATA& imageData) {
        // Create depth matrix
        cv::Mat depthMat(imageData.nHeight, imageData.nWidth, CV_16UC1, imageData.pData);

        // Create mask for valid depth values (non-zero)
        cv::Mat validMask;
        cv::threshold(depthMat, validMask, 0, 255, cv::THRESH_BINARY);
        validMask.convertTo(validMask, CV_8UC1);

        cv::Mat depthDisplay;
        double minVal, maxVal;
        cv::minMaxLoc(depthMat, &minVal, &maxVal, nullptr, nullptr, validMask);

        if (maxVal > minVal && maxVal > 0) {
            // INVERTED mapping: closer objects (higher depth values) = 255 (red)
            // farther objects (lower depth values) = 0 (blue)
            depthMat.convertTo(depthDisplay, CV_8UC1, -255.0 / (maxVal - minVal), 255.0 * maxVal / (maxVal - minVal));

            // Set invalid areas to 0 (will become black)
            depthDisplay.setTo(0, ~validMask);
        }
        else {
            depthDisplay = cv::Mat::zeros(imageData.nHeight, imageData.nWidth, CV_8UC1);
        }

        // Apply JET colormap: high values=red (close), low values=blue (far), 0=black (unknown)
        cv::Mat coloredDepth;
        cv::applyColorMap(depthDisplay, coloredDepth, cv::COLORMAP_JET);

        // Ensure unknown/invalid areas are completely black
        std::vector<cv::Mat> channels;
        cv::split(coloredDepth, channels);
        for (int i = 0; i < 3; i++) {
            channels[i].setTo(0, ~validMask);
        }
        cv::merge(channels, coloredDepth);

        return ResizeForDisplay(coloredDepth);
    }

    cv::Mat ConvertColorToDisplay(const MV3D_RGBD_IMAGE_DATA& imageData) {
        cv::Mat displayMat;

        if (imageData.enImageType == ImageType_RGB8_Planar) {
            // RGB planar format conversion - proper handling
            int height = imageData.nHeight;
            int width = imageData.nWidth;
            unsigned char* pData = (unsigned char*)imageData.pData;

            // RGB8_Planar format: R channel, G channel, B channel stored separately
            cv::Mat rChannel(height, width, CV_8UC1, pData);
            cv::Mat gChannel(height, width, CV_8UC1, pData + height * width);
            cv::Mat bChannel(height, width, CV_8UC1, pData + 2 * height * width);

            // Merge channels in BGR order (OpenCV format)
            std::vector<cv::Mat> channels = { bChannel, gChannel, rChannel };
            cv::merge(channels, displayMat);
        }
        else if (imageData.enImageType == ImageType_YUV422) {
            // YUV422 format conversion
            int height = imageData.nHeight;
            int width = imageData.nWidth;
            unsigned char* pData = (unsigned char*)imageData.pData;

            // Convert YUV422 to RGB
            cv::Mat yuvMat(height, width, CV_8UC2, pData);
            cv::cvtColor(yuvMat, displayMat, cv::COLOR_YUV2BGR_YUYV);
        }
        else if (imageData.enImageType == ImageType_YUV420SP_NV12) {
            // NV12 format conversion
            int height = imageData.nHeight;
            int width = imageData.nWidth;
            unsigned char* pData = (unsigned char*)imageData.pData;

            cv::Mat yuvMat(height * 3 / 2, width, CV_8UC1, pData);
            cv::cvtColor(yuvMat, displayMat, cv::COLOR_YUV2BGR_NV12);
        }
        else if (imageData.enImageType == ImageType_YUV420SP_NV21) {
            // NV21 format conversion
            int height = imageData.nHeight;
            int width = imageData.nWidth;
            unsigned char* pData = (unsigned char*)imageData.pData;

            cv::Mat yuvMat(height * 3 / 2, width, CV_8UC1, pData);
            cv::cvtColor(yuvMat, displayMat, cv::COLOR_YUV2BGR_NV21);
        }
        else {
            // Fallback: treat as BGR
            int height = imageData.nHeight;
            int width = imageData.nWidth;
            unsigned char* pData = (unsigned char*)imageData.pData;
            displayMat = cv::Mat(height, width, CV_8UC3, pData).clone();
        }

        return ResizeForDisplay(displayMat);
    }

    cv::Mat ConvertIRToDisplay(const MV3D_RGBD_IMAGE_DATA& imageData) {
        cv::Mat irMat(imageData.nHeight, imageData.nWidth, CV_8UC1, imageData.pData);
        cv::Mat irColor;
        cv::applyColorMap(irMat, irColor, cv::COLORMAP_BONE);
        return ResizeForDisplay(irColor);
    }

    void DrawIMUInfo(cv::Mat& image) {
        float xAcc, yAcc, zAcc, xGyro, yGyro, zGyro;
        bool hasNewData;

        g_imuData.getData(xAcc, yAcc, zAcc, xGyro, yGyro, zGyro, hasNewData);

        // Smaller info box - adjust these values to make it smaller
        cv::Rect infoRect(10, 10, 80, 130);  // Reduced from 300x150 to 200x120
        cv::Mat overlay = image.clone();
        cv::rectangle(overlay, infoRect, cv::Scalar(0, 0, 0), cv::FILLED);
        cv::addWeighted(image, 0.7, overlay, 0.3, 0, image);

        // Draw border
        cv::rectangle(image, infoRect, cv::Scalar(255, 255, 255), 1);  // Thinner border

        // Smaller text properties
        int fontFace = cv::FONT_HERSHEY_SIMPLEX;
        double fontScale = 0.35;  // Reduced from 0.5 to 0.35
        int thickness = 1;
        cv::Scalar textColor = hasNewData ? cv::Scalar(0, 255, 0) : cv::Scalar(128, 128, 128);

        // Create text strings with shorter format
        std::stringstream ss;
        ss << std::fixed << std::setprecision(1);  // Reduced precision to save space

        std::vector<std::string> imuText;
        imuText.push_back("IMU Data:");

        ss.str(""); ss << "AccX:" << xAcc; imuText.push_back(ss.str());
        ss.str(""); ss << "AccY:" << yAcc; imuText.push_back(ss.str());
        ss.str(""); ss << "AccZ:" << zAcc; imuText.push_back(ss.str());
        ss.str(""); ss << "GyrX:" << xGyro; imuText.push_back(ss.str());
        ss.str(""); ss << "GyrY:" << yGyro; imuText.push_back(ss.str());
        ss.str(""); ss << "GyrZ:" << zGyro; imuText.push_back(ss.str());

        // Draw text with smaller spacing
        for (size_t i = 0; i < imuText.size(); i++) {
            cv::Point textPos(15, 25 + i * 15);  // Reduced spacing from 20 to 15
            cv::Scalar color = (i == 0) ? cv::Scalar(255, 255, 0) : textColor; // Title in yellow
            cv::putText(image, imuText[i], textPos, fontFace, fontScale, color, thickness);
        }

        // Add frame counter
        ss.str(""); ss << "F:" << m_frameCount;  // Shorter format
        cv::putText(image, ss.str(), cv::Point(15, 25 + imuText.size() * 15), fontFace, fontScale, cv::Scalar(255, 255, 255), thickness);
    }

    void ProcessFrameData(const MV3D_RGBD_FRAME_DATA& frameData) {
        for (unsigned int i = 0; i < frameData.nImageCount; i++) {
            const MV3D_RGBD_IMAGE_DATA& imageData = frameData.stImageData[i];

            std::string windowName;
            cv::Mat displayMat;

            switch (imageData.enImageType) {
            case ImageType_Depth:
                windowName = "Depth Stream with IMU";
                displayMat = ConvertDepthToDisplay(imageData);
                break;

            case ImageType_RGB8_Planar:
            case ImageType_YUV422:
            case ImageType_YUV420SP_NV12:
            case ImageType_YUV420SP_NV21:
                windowName = "Color Stream with IMU";
                displayMat = ConvertColorToDisplay(imageData);
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
    }

    int RunViewer() {
        if (!m_bGrabbing) {
            std::cerr << "Grabbing not started!" << std::endl;
            return MV3D_RGBD_E_HANDLE;
        }

        std::cout << "\n=== Real-Time Viewer with IMU Started ===" << std::endl;
        std::cout << "Display Resolution: " << m_resolutions[m_selectedResolution].name << std::endl;
        std::cout << "\nControls:" << std::endl;
        std::cout << "  - Press 'q' on any window to quit" << std::endl;
        std::cout << "  - Press 'ESC' on any window to quit" << std::endl;
        std::cout << "  - Press 'r' to change resolution" << std::endl;
        std::cout << "=======================================" << std::endl;

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
    std::cout << "=== RGBD, IR Real-Time Viewer with IMU Data ===" << std::endl;
    std::cout << "This program displays RGBD and IR streams with IMU data overlay using OpenCV." << std::endl;

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

    // Run real-time viewer with IMU overlay
    nRet = viewer.RunViewer();

    std::cout << "Real-time viewing with IMU data completed!" << std::endl;
    return 0;
}