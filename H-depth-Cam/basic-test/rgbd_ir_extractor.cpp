#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <iomanip>
#include <sstream>

// Include the actual MV3D RGBD SDK headers from the correct path
#include "../common/common.hpp"

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
            std::cerr << "Invalid choice! Using default (854x480 - 480p)" << std::endl;
            m_selectedResolution = 2; // Default to 854x480 (480p)
            return MV3D_RGBD_OK;
        }
    }

    cv::Mat ResizeForDisplay(const cv::Mat& inputMat) {
        if (inputMat.empty()) {
            return inputMat;
        }

        // If "Original Size" is selected, return as-is
        if (m_selectedResolution == m_resolutions.size() - 1) {
            return inputMat;
        }

        int targetWidth = m_resolutions[m_selectedResolution].width;
        int targetHeight = m_resolutions[m_selectedResolution].height;

        cv::Mat resized;
        cv::resize(inputMat, resized, cv::Size(targetWidth, targetHeight), 0, 0, cv::INTER_LINEAR);
        return resized;
    }

    int Initialize() {
        std::cout << "Initializing MV3D RGBD SDK..." << std::endl;

        // Get SDK version
        MV3D_RGBD_VERSION_INFO stVersion;
        int nRet = MV3D_RGBD_GetSDKVersion(&stVersion);
        if (MV3D_RGBD_OK != nRet) {
            std::cerr << "Failed to get SDK version! Error: 0x" << std::hex << nRet << std::endl;
            return nRet;
        }
        std::cout << "SDK Version: " << stVersion.nMajor << "." << stVersion.nMinor << "." << stVersion.nRevision << std::endl;

        // Initialize SDK
        nRet = MV3D_RGBD_Initialize();
        if (MV3D_RGBD_OK != nRet) {
            std::cerr << "Failed to initialize SDK! Error: 0x" << std::hex << nRet << std::endl;
            return nRet;
        }

        return MV3D_RGBD_OK;
    }

    int EnumerateDevices(std::vector<MV3D_RGBD_DEVICE_INFO>& deviceList) {
        unsigned int nDevNum = 0;
        int nRet = MV3D_RGBD_GetDeviceNumber(DeviceType_Ethernet | DeviceType_USB |
            DeviceType_Ethernet_Vir | DeviceType_USB_Vir, &nDevNum);
        if (MV3D_RGBD_OK != nRet) {
            std::cerr << "Failed to get device number! Error: 0x" << std::hex << nRet << std::endl;
            return nRet;
        }

        std::cout << "Found " << nDevNum << " device(s)." << std::endl;
        if (nDevNum == 0) {
            std::cerr << "No devices found!" << std::endl;
            return MV3D_RGBD_E_NOENOUGH_BUF;
        }

        deviceList.resize(nDevNum);
        nRet = MV3D_RGBD_GetDeviceList(DeviceType_Ethernet | DeviceType_USB |
            DeviceType_Ethernet_Vir | DeviceType_USB_Vir,
            &deviceList[0], nDevNum, &nDevNum);
        if (MV3D_RGBD_OK != nRet) {
            std::cerr << "Failed to get device list! Error: 0x" << std::hex << nRet << std::endl;
            return nRet;
        }

        // Print device information
        for (unsigned int i = 0; i < nDevNum; i++) {
            std::cout << "Device [" << i << "]: ";
            if (DeviceType_Ethernet == deviceList[i].enDeviceType ||
                DeviceType_Ethernet_Vir == deviceList[i].enDeviceType) {
                std::cout << "SerialNum[" << deviceList[i].chSerialNumber << "] "
                    << "IP[" << deviceList[i].SpecialInfo.stNetInfo.chCurrentIp << "] "
                    << "Name[" << deviceList[i].chModelName << "]" << std::endl;
            }
            else if (DeviceType_USB == deviceList[i].enDeviceType ||
                DeviceType_USB_Vir == deviceList[i].enDeviceType) {
                std::cout << "SerialNum[" << deviceList[i].chSerialNumber << "] "
                    << "USB Protocol[" << deviceList[i].SpecialInfo.stUsbInfo.enUsbProtocol << "] "
                    << "Name[" << deviceList[i].chModelName << "]" << std::endl;
            }
        }

        return MV3D_RGBD_OK;
    }

    int OpenDevice(MV3D_RGBD_DEVICE_INFO& deviceInfo) {
        int nRet = MV3D_RGBD_OpenDevice(&m_handle, &deviceInfo);
        if (MV3D_RGBD_OK != nRet) {
            std::cerr << "Failed to open device! Error: 0x" << std::hex << nRet << std::endl;
            return nRet;
        }

        m_bDeviceOpen = true;
        std::cout << "Device opened successfully." << std::endl;

        // Get camera parameters
        MV3D_RGBD_CAMERA_PARAM stCameraParam;
        nRet = MV3D_RGBD_GetCameraParam(m_handle, &stCameraParam);
        if (MV3D_RGBD_OK == nRet) {
            std::cout << "Camera parameters retrieved successfully." << std::endl;
            PrintCameraParams(stCameraParam);
        }

        return MV3D_RGBD_OK;
    }

    void PrintCameraParams(const MV3D_RGBD_CAMERA_PARAM& params) {
        std::cout << "=== Camera Parameters ===" << std::endl;
        std::cout << "Depth Camera:" << std::endl;
        std::cout << "  Width: " << params.stDepthCalibInfo.nWidth << ", Height: " << params.stDepthCalibInfo.nHeight << std::endl;
        std::cout << "  fx: " << params.stDepthCalibInfo.stIntrinsic.fData[0] << ", fy: " << params.stDepthCalibInfo.stIntrinsic.fData[4] << std::endl;
        std::cout << "  cx: " << params.stDepthCalibInfo.stIntrinsic.fData[2] << ", cy: " << params.stDepthCalibInfo.stIntrinsic.fData[5] << std::endl;
        std::cout << "RGB Camera:" << std::endl;
        std::cout << "  Width: " << params.stRgbCalibInfo.nWidth << ", Height: " << params.stRgbCalibInfo.nHeight << std::endl;
        std::cout << "  fx: " << params.stRgbCalibInfo.stIntrinsic.fData[0] << ", fy: " << params.stRgbCalibInfo.stIntrinsic.fData[4] << std::endl;
        std::cout << "  cx: " << params.stRgbCalibInfo.stIntrinsic.fData[2] << ", cy: " << params.stRgbCalibInfo.stIntrinsic.fData[5] << std::endl;
        std::cout << "=========================" << std::endl;
    }

    int StartGrabbing() {
        if (!m_bDeviceOpen) {
            std::cerr << "Device not opened!" << std::endl;
            return MV3D_RGBD_E_HANDLE;
        }

        int nRet = MV3D_RGBD_Start(m_handle);
        if (MV3D_RGBD_OK != nRet) {
            std::cerr << "Failed to start grabbing! Error: 0x" << std::hex << nRet << std::endl;
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
            std::cerr << "Failed to stop grabbing! Error: 0x" << std::hex << nRet << std::endl;
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
            std::cerr << "Failed to close device! Error: 0x" << std::hex << nRet << std::endl;
            return nRet;
        }

        m_bDeviceOpen = false;
        m_handle = nullptr;
        std::cout << "Device closed successfully." << std::endl;
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
        else if (ImageType_YUV420SP_NV12 == imageData.enImageType) {
            // YUV420 NV12 format
            cv::Mat yuvMat(imageData.nHeight * 3 / 2, imageData.nWidth, CV_8UC1, imageData.pData);
            cv::Mat rgbMat;
            cv::cvtColor(yuvMat, rgbMat, cv::COLOR_YUV2BGR_NV12);
            return rgbMat;
        }

        // Default: return empty mat
        return cv::Mat();
    }

    cv::Mat ConvertMonoToDisplay(const MV3D_RGBD_IMAGE_DATA& imageData) {
        if (ImageType_Mono8 == imageData.enImageType) {
            cv::Mat monoMat(imageData.nHeight, imageData.nWidth, CV_8UC1, imageData.pData);
            cv::Mat colorMono;
            cv::cvtColor(monoMat, colorMono, cv::COLOR_GRAY2BGR);
            return colorMono;
        }
        else if (ImageType_Mono16 == imageData.enImageType) {
            cv::Mat mono16Mat(imageData.nHeight, imageData.nWidth, CV_16UC1, imageData.pData);
            cv::Mat mono8Mat;
            mono16Mat.convertTo(mono8Mat, CV_8UC1, 255.0 / 65535.0);
            cv::Mat colorMono;
            cv::cvtColor(mono8Mat, colorMono, cv::COLOR_GRAY2BGR);
            return colorMono;
        }

        return cv::Mat();
    }

    cv::Mat ConvertRGBDToDisplay(const MV3D_RGBD_IMAGE_DATA& imageData) {
        // RGBD format contains 5 channels: R, G, B, D_low, D_high
        unsigned char* data = (unsigned char*)imageData.pData;
        int width = imageData.nWidth;
        int height = imageData.nHeight;

        // Extract RGB channels
        cv::Mat rgbMat(height, width, CV_8UC3);
        for (int i = 0; i < height * width; i++) {
            rgbMat.data[i * 3 + 2] = data[i * 5 + 0]; // R -> BGR format
            rgbMat.data[i * 3 + 1] = data[i * 5 + 1]; // G
            rgbMat.data[i * 3 + 0] = data[i * 5 + 2]; // B
        }

        // Extract depth channels and combine
        cv::Mat depthMat(height, width, CV_16UC1);
        unsigned short* depthData = (unsigned short*)depthMat.data;
        for (int i = 0; i < height * width; i++) {
            unsigned short depthLow = data[i * 5 + 3];
            unsigned short depthHigh = data[i * 5 + 4];
            depthData[i] = (depthHigh << 8) | depthLow;
        }

        // Convert depth to colored visualization
        cv::Mat depthDisplay = ConvertDepthToDisplay_Internal(depthMat);

        // Combine RGB and depth side by side
        cv::Mat combined;
        cv::hconcat(rgbMat, depthDisplay, combined);

        return combined;
    }

    cv::Mat ConvertDepthToDisplay_Internal(const cv::Mat& depthMat) {
        cv::Mat depthDisplay;

        // Normalize depth for better visualization
        double minVal, maxVal;
        cv::minMaxLoc(depthMat, &minVal, &maxVal);

        if (maxVal > minVal) {
            depthMat.convertTo(depthDisplay, CV_8UC1, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));
        }
        else {
            depthDisplay = cv::Mat::zeros(depthMat.rows, depthMat.cols, CV_8UC1);
        }

        // Apply colormap for better visualization
        cv::Mat coloredDepth;
        cv::applyColorMap(depthDisplay, coloredDepth, cv::COLORMAP_JET);

        return coloredDepth;
    }

    void ProcessFrameData(const MV3D_RGBD_FRAME_DATA& frameData) {
        for (unsigned int i = 0; i < frameData.nImageCount; i++) {
            const MV3D_RGBD_IMAGE_DATA& imageData = frameData.stImageData[i];

            std::string windowName;
            cv::Mat displayMat;

            switch (imageData.enImageType) {
            case ImageType_Depth:
                windowName = "Depth Stream";
                displayMat = ConvertDepthToDisplay(imageData);
                break;

            case ImageType_RGB8_Planar:
            case ImageType_YUV422:
            case ImageType_YUV420SP_NV12:
            case ImageType_YUV420SP_NV21:
                windowName = "Color Stream";
                displayMat = ConvertColorToDisplay(imageData);
                break;

            case ImageType_Mono8:
            case ImageType_Mono16:
                // Check coordinate type to determine if it's IR or regular mono
                if (imageData.enCoordinateType == CoordinateType_Depth) {
                    windowName = "IR Stream";
                }
                else {
                    windowName = "Mono Stream";
                }
                displayMat = ConvertMonoToDisplay(imageData);
                break;

            case ImageType_Rgbd:
                windowName = "RGBD Stream (RGB | Depth)";
                displayMat = ConvertRGBDToDisplay(imageData);
                break;

            default:
                continue; // Skip unsupported types
            }

            if (!displayMat.empty()) {
                // Resize for display
                cv::Mat resizedMat = ResizeForDisplay(displayMat);

                // Add frame info text
                std::string frameInfo = "Frame: " + std::to_string(imageData.nFrameNum) +
                    " | Original: " + std::to_string(imageData.nWidth) + "x" + std::to_string(imageData.nHeight) +
                    " | Display: " + std::to_string(resizedMat.cols) + "x" + std::to_string(resizedMat.rows) +
                    " | Type: " + std::to_string(imageData.enImageType);

                // Scale text size based on display resolution
                double textScale = std::max(0.4, std::min(1.0, resizedMat.cols / 640.0 * 0.7));
                int textThickness = std::max(1, static_cast<int>(textScale * 2));

                cv::putText(resizedMat, frameInfo, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, textScale, cv::Scalar(0, 255, 0), textThickness);

                // Show the image
                cv::imshow(windowName, resizedMat);
            }
        }

        m_frameCount++;
        if (m_frameCount % 30 == 0) {
            std::cout << "Displayed " << m_frameCount << " frames..." << std::endl;
        }
    }

    int RunViewer() {
        if (!m_bGrabbing) {
            std::cerr << "Not grabbing!" << std::endl;
            return MV3D_RGBD_E_HANDLE;
        }

        std::cout << "\n=== Real-Time Viewer Started ===" << std::endl;
        std::cout << "Display Resolution: " << m_resolutions[m_selectedResolution].name << std::endl;
        std::cout << "\nControls:" << std::endl;
        std::cout << "  - Press 'q' on any window to quit" << std::endl;
        std::cout << "  - Press 'ESC' on any window to quit" << std::endl;
        std::cout << "  - Press 'r' to change resolution" << std::endl;
        std::cout << "=================================" << std::endl;

        MV3D_RGBD_FRAME_DATA frameData = { 0 };

        while (true) {
            int nRet = MV3D_RGBD_FetchFrame(m_handle, &frameData, 100); // Shorter timeout for real-time
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
    std::cout << "=== RGBD and IR Real-Time Viewer ===" << std::endl;
    std::cout << "This program displays RGBD and IR streams in real-time using OpenCV." << std::endl;

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

    // Run real-time viewer
    nRet = viewer.RunViewer();

    std::cout << "Real-time viewing completed!" << std::endl;
    return 0;
}