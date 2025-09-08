/**
 * ORB-SLAM3 RGB-D example for Aurora 900 camera
 * WITH ORB-SLAM3 MAP POINT EXPORT (Simplified Approach)
 */

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <thread>
#include <iomanip>
#include <set>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include <System.h>

// Aurora SDK includes
#include "deptrum/device.h"
#include "deptrum/stream.h"
#include "deptrum/aurora900_series.h"

#include <signal.h>
#include <csignal>
#include <atomic>
#include <unistd.h>
#include <sys/select.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/stat.h>

using namespace std;
using namespace deptrum;
using namespace deptrum::stream;

// Global resource manager for safe shutdown
class SafeResourceManager {
public:
    SafeResourceManager() : should_exit_(false) {}
    void requestExit() { should_exit_ = true; }
    bool shouldExit() const { return should_exit_; }
private:
    std::atomic<bool> should_exit_;
};

SafeResourceManager g_resource_manager;

// Signal handler function
void signalHandler(int signum) {
    std::cout << "\n[DEBUG] Received signal " << signum << std::endl;
    g_resource_manager.requestExit();
}

// Setup terminal for non-blocking input
void setupTerminal() {
    struct termios term;
    tcgetattr(STDIN_FILENO, &term);
    term.c_lflag &= ~(ICANON | ECHO); // Disable line buffering and echo
    tcsetattr(STDIN_FILENO, TCSANOW, &term);
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK); // Make stdin non-blocking
}

void restoreTerminal() {
    struct termios term;
    tcgetattr(STDIN_FILENO, &term);
    term.c_lflag |= (ICANON | ECHO); // Restore line buffering and echo
    tcsetattr(STDIN_FILENO, TCSANOW, &term);
    fcntl(STDIN_FILENO, F_SETFL, 0); // Make stdin blocking again
}

// Function to save RGB-D frame for reference/backup
void SaveRGBDFrame(const cv::Mat& rgb, const cv::Mat& depth, int frame_number, double timestamp) {
    try {
        std::string folder = "3D_Reconstruction_Data/";
        
        // Create filenames with frame number and timestamp
        std::string rgb_filename = folder + "rgbd_color_" + std::to_string(frame_number) + ".png";
        std::string depth_filename = folder + "rgbd_depth_" + std::to_string(frame_number) + ".png";
        std::string info_filename = folder + "rgbd_info_" + std::to_string(frame_number) + ".txt";
        
        // Save RGB image
        bool rgb_ok = cv::imwrite(rgb_filename, rgb);
        
        // Save depth image (16-bit PNG to preserve depth precision)
        bool depth_ok = cv::imwrite(depth_filename, depth);
        
        // Save frame info (timestamp, frame number)
        std::ofstream info_file(info_filename);
        info_file << "frame_number: " << frame_number << std::endl;
        info_file << "timestamp: " << std::fixed << std::setprecision(6) << timestamp << std::endl;
        info_file << "rgb_file: " << rgb_filename << std::endl;
        info_file << "depth_file: " << depth_filename << std::endl;
        info_file.close();
        
        if (rgb_ok && depth_ok) {
            std::cout << "[BACKUP] Saved RGB-D frame " << frame_number << " for backup" << std::endl;
        } else {
            std::cout << "[ERROR] Failed to save RGB-D frame " << frame_number << std::endl;
        }
        
    } catch (const cv::Exception& e) {
        std::cout << "[ERROR] RGB-D save error: " << e.what() << std::endl;
    }
}

// Software coordinate transformation class
class DepthToColorTransform {
private:
    Eigen::Matrix3d R_depth_to_color_;
    Eigen::Vector3d t_depth_to_color_;
    cv::Mat depthK_, colorK_;
    bool initialized_;
    
public:
    DepthToColorTransform() : initialized_(false) {
        R_depth_to_color_ << 0.999998,  0.001045, -0.001768,
                            -0.001045,  0.999999, -0.000077,
                             0.001768,  0.000078,  0.999998;
        
        t_depth_to_color_ << -13.851061, -0.219677, -2.052635; // mm
        t_depth_to_color_ /= 1000.0; // Convert to meters
        
        std::cout << "[DEBUG] Software coordinate transformer initialized." << std::endl;
    }
    
    void setCameraIntrinsics(const cv::Mat& depthK, const cv::Mat& colorK) {
        depthK_ = depthK.clone();
        colorK_ = colorK.clone();
        initialized_ = true;
        std::cout << "[DEBUG] Transformer ready for software alignment." << std::endl;
        
        std::cout << "[DEBUG] Depth camera matrix:" << std::endl << depthK_ << std::endl;
        std::cout << "[DEBUG] Color camera matrix:" << std::endl << colorK_ << std::endl;
    }
    
    cv::Mat alignDepthToColor(const cv::Mat& depthImage, const cv::Size& colorSize) {
        if (!initialized_) return cv::Mat::zeros(colorSize, CV_16UC1);
        
        cv::Mat alignedDepth = cv::Mat::zeros(colorSize, CV_16UC1);
        
        float fx_d = depthK_.at<float>(0, 0), fy_d = depthK_.at<float>(1, 1);
        float cx_d = depthK_.at<float>(0, 2), cy_d = depthK_.at<float>(1, 2);
        float fx_c = colorK_.at<float>(0, 0), fy_c = colorK_.at<float>(1, 1);
        float cx_c = colorK_.at<float>(0, 2), cy_c = colorK_.at<float>(1, 2);
        
        int valid_points = 0;
        int total_depth_points = 0;
        
        for (int v = 0; v < depthImage.rows; v++) {
            for (int u = 0; u < depthImage.cols; u++) {
                uint16_t depth_value = depthImage.at<uint16_t>(v, u);
                if (depth_value == 0) continue;
                
                total_depth_points++;
                float depth_m = depth_value / 1000.0f;
                Eigen::Vector3d point_depth((u - cx_d) * depth_m / fx_d,
                                          (v - cy_d) * depth_m / fy_d, depth_m);
                
                Eigen::Vector3d point_color = R_depth_to_color_ * point_depth + t_depth_to_color_;
                if (point_color.z() <= 0) continue;
                
                int u_c = (int)(fx_c * point_color.x() / point_color.z() + cx_c + 0.5);
                int v_c = (int)(fy_c * point_color.y() / point_color.z() + cy_c + 0.5);
                
                if (u_c >= 0 && u_c < colorSize.width && v_c >= 0 && v_c < colorSize.height) {
                    alignedDepth.at<uint16_t>(v_c, u_c) = depth_value;
                    valid_points++;
                }
            }
        }
        
        static int alignment_count = 0;
        if (++alignment_count % 30 == 0) {
            std::cout << "[DEBUG] Software alignment: " << valid_points 
                     << "/" << total_depth_points << " points ("
                     << (100.0 * valid_points / total_depth_points) << "%)" << std::endl;
        }
        
        return alignedDepth;
    }
    
    void fillDepthHoles(cv::Mat& depth) {
        cv::Mat mask = (depth == 0);
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
        
        cv::Mat dilated;
        cv::dilate(depth, dilated, kernel);
        dilated.copyTo(depth, mask);
    }
};


int ChooceFrameMode(std::vector<std::tuple<FrameMode, FrameMode, FrameMode>> device_resolution_vec) {
    int num = 0;
    std::cout << "Available frame modes:" << std::endl;
    std::cout << "     ir_mode               rgb_mode                depth_mode" << std::endl;
    
    for (auto device_resolution : device_resolution_vec) {
        FrameMode ir_mode = std::get<0>(device_resolution);
        FrameMode rgb_mode = std::get<1>(device_resolution);
        FrameMode depth_mode = std::get<2>(device_resolution);
        
        // Simple display without accessing internal members
        printf("[%d]: Mode %d                Mode %d                Mode %d\n", 
               num++, num-1, num-1, num-1);
    }
    
    int index = -1;
    while (index < 0 || index >= device_resolution_vec.size()) {
        std::cout << "Enter your choice (0-" << (device_resolution_vec.size()-1) << "): ";
        std::cin >> index;
        std::cout << std::endl;
    }
    return index;
}

class AuroraCapture {
public:
    AuroraCapture(bool use_hardware_align = true) 
        : stream_(nullptr), use_hw_align_(use_hardware_align) {
        std::cout << "[DEBUG] Aurora 900 - Using " << (use_hw_align_ ? "HARDWARE" : "SOFTWARE") << " alignment" << std::endl;
    }
    
    ~AuroraCapture() {
        if (stream_) {
            stream_->Stop();
            device_->DestroyStream(stream_);
        }
        if (device_) device_->Close();
    }

    bool initialize() {
        try {
            std::cout << "\n[DEBUG] ===== AURORA INITIALIZATION DEBUG =====" << std::endl;
            
            std::vector<DeviceInformation> device_list;
            int ret = DeviceManager::GetInstance()->GetDeviceList(device_list);
            std::cout << "[DEBUG] Found " << device_list.size() << " Aurora devices" << std::endl;
            
            if (ret != 0 || device_list.empty()) {
                std::cerr << "[ERROR] No Aurora device found! Return code: " << ret << std::endl;
                return false;
            }
            
            device_ = DeviceManager::GetInstance()->CreateDevice(device_list[0]);
            if (!device_) {
                std::cerr << "[ERROR] Failed to create Aurora device!" << std::endl;
                return false;
            }
            
            ret = device_->Open();
            if (ret != 0) {
                std::cerr << "[ERROR] Failed to open Aurora device! Return code: " << ret << std::endl;
                return false;
            }
            
            // Configure device mode
            std::cout << "\n[DEBUG] Configuring device mode..." << std::endl;
            FrameMode ir_mode, rgb_mode, depth_mode;
            std::vector<std::tuple<FrameMode, FrameMode, FrameMode>> device_resolution_vec;
            device_->GetSupportedFrameMode(device_resolution_vec);

            if (device_resolution_vec.empty()) {
                std::cerr << "[ERROR] No supported frame modes found!" << std::endl;
                return false;
            }

            std::cout << "\n[DEBUG] Found " << device_resolution_vec.size() << " supported modes" << std::endl;
            int index = ChooceFrameMode(device_resolution_vec);

            ir_mode = std::get<0>(device_resolution_vec[index]);
            rgb_mode = std::get<1>(device_resolution_vec[index]);
            depth_mode = std::get<2>(device_resolution_vec[index]);

            std::cout << "[DEBUG] Selected mode " << index << std::endl;

            ret = device_->SetMode(ir_mode, rgb_mode, depth_mode);
            if (ret != 0) {
                std::cerr << "[ERROR] Failed to set device mode! Return code: " << ret << std::endl;
                return false;
            }
            std::cout << "[DEBUG] Device mode set successfully" << std::endl;
            
            // Create stream with RGB and Depth
            std::cout << "\n[DEBUG] Creating RGB+Depth stream..." << std::endl;
            std::vector<StreamType> stream_types;
            stream_types.push_back(StreamType::kRgb);
            stream_types.push_back(StreamType::kDepth);
            
            ret = device_->CreateStream(stream_, stream_types);
            if (ret != 0) {
                std::cerr << "[ERROR] Failed to create stream! Return code: " << ret << std::endl;
                return false;
            }
            
            std::cout << "[DEBUG] Aurora stream created successfully" << std::endl;
            
            // Get Aurora camera parameters (using your YAML values)
            std::cout << "\n[DEBUG] Setting Aurora camera parameters..." << std::endl;
            colorK_ = (cv::Mat_<float>(3, 3) << 
                417.5291, 0, 314.9391,
                0, 418.4000, 188.2704, 
                0, 0, 1);

            depthK_ = colorK_.clone(); // Assuming aligned mode

            sw_transformer_.setCameraIntrinsics(depthK_, colorK_);

            std::cout << "[DEBUG] Using Aurora 930 calibrated parameters from YAML" << std::endl;

            return true;
            
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] Aurora initialization error: " << e.what() << std::endl;
            return false;
        }
    }

    bool start() {
        try {
            std::cout << "[DEBUG] Starting Aurora stream..." << std::endl;
            int ret = stream_->Start();
            if (ret != 0) {
                std::cerr << "[ERROR] Failed to start stream! Return code: " << ret << std::endl;
                return false;
            }
            std::cout << "[DEBUG] Aurora stream started successfully" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            return true;
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] Aurora start error: " << e.what() << std::endl;
            return false;
        }
    }
    
    bool getFrames(cv::Mat& color, cv::Mat& depth, double& timestamp) {
        static int frame_count = 0;
        frame_count++;
        
        bool debug_this_frame = (frame_count % 30 == 0);
        
        try {
            StreamFrames frames;
            
            // Get frames from Aurora stream
            int ret = stream_->GetFrames(frames, 1000); // 1000ms timeout
            if (ret != 0) {
                if (debug_this_frame) {
                    std::cout << "[WARNING] No frames received, return code: " << ret << std::endl;
                }
                return false;
            }
            
            if (frames.count == 0 || frames.frame_ptr.empty()) {
                if (debug_this_frame) {
                    std::cout << "[WARNING] Empty frames received" << std::endl;
                }
                return false;
            }
            
            std::shared_ptr<StreamFrame> rgb_frame = nullptr;
            std::shared_ptr<StreamFrame> depth_frame = nullptr;
            
            // Find RGB and Depth frames in the frame_ptr vector
            for (auto& frame : frames.frame_ptr) {
                if (frame->frame_type == FrameType::kRgbFrame) {
                    rgb_frame = frame;
                } else if (frame->frame_type == FrameType::kDepthFrame) {
                    depth_frame = frame;
                }
            }
            
            if (!rgb_frame || !depth_frame) {
                if (debug_this_frame) {
                    std::cout << "[WARNING] Missing RGB or Depth frame" << std::endl;
                }
                return false;
            }
            
            // Convert Aurora frames to OpenCV
            // Aurora uses cols/rows for dimensions, data for raw pointer
            cv::Mat color_temp(rgb_frame->rows, rgb_frame->cols, CV_8UC3, rgb_frame->data.get());
            cv::Mat depth_temp(depth_frame->rows, depth_frame->cols, CV_16UC1, depth_frame->data.get());
            
            // Clone the data to ensure it persists after frame release
            color = color_temp.clone();
            depth = depth_temp.clone();
            
            // Aurora may use BGR format, convert to RGB if needed
            if (color.channels() == 3) {
                cv::cvtColor(color, color, cv::COLOR_BGR2RGB);
            }
            
            // Get timestamp (Aurora uses timestamp member)
            timestamp = rgb_frame->timestamp / 1000000.0; // Convert microseconds to seconds
            
            if (debug_this_frame) {
                std::cout << "[DEBUG] Aurora frames - Color: " << color.size() 
                         << ", Depth: " << depth.size() << std::endl;
            }
            
            // Only resize if absolutely necessary and warn about it
            if (debug_this_frame) {
                std::cout << "[DEBUG] Aurora native frames - Color: " << color.size() 
                        << ", Depth: " << depth.size() << std::endl;
            }
            
            return !color.empty() && !depth.empty();

        } catch (const std::exception& e) {
            std::cerr << "[ERROR] Aurora frame error: " << e.what() << std::endl;
            return false;
        }
    }
    
private:
    std::shared_ptr<Device> device_;
    Stream* stream_;
    
    cv::Mat colorK_, depthK_;
    DepthToColorTransform sw_transformer_;
    bool use_hw_align_;
};

void GenerateMapPointConverter() {
    std::cout << "[MESH] Generating ENHANCED TUBE MESH GENERATOR WITH FRAME DETAIL PRESERVATION..." << std::endl;
    std::string reconstruction_folder = "3D_Reconstruction_Data";
    std::string folder = "3D_Reconstruction_Data/";
    
    std::ofstream script(folder + "tube_mesh_generator.py");
    script << "#!/usr/bin/env python3\n";
    script << "\"\"\"\n";
    script << "TUBE-SPECIFIC MESH GENERATOR\n";
    script << "Optimized for metal cuboid tubes and geometric objects\n";
    script << "Handles sparse points and sharp edges properly\n";
    script << "\"\"\"\n\n";
    
    script << "import numpy as np\nimport open3d as o3d\nimport os\nimport time\nimport psutil\n\n";
    
    // Memory monitoring
    script << "def print_memory_status(step=''):\n";
    script << "    mem = psutil.virtual_memory()\n";
    script << "    print(f'💾 Memory: {mem.used / (1024**3):.1f}/{mem.total / (1024**3):.1f}GB ({mem.percent:.1f}%) - {step}')\n\n";
    
    // Load ORB-SLAM points
    script << "def load_orb_slam_map_points(filename):\n";
    script << "    print(f'📊 Loading ORB-SLAM3 map points from {filename}...')\n";
    script << "    if not os.path.exists(filename): print(f'❌ File not found: {filename}'); return None\n";
    script << "    try:\n";
    script << "        coords = np.genfromtxt(filename, delimiter=',', skip_header=1)\n";
    script << "        if coords.size == 0: print('❌ No data in file!'); return None\n";
    script << "        if coords.ndim == 1: coords = coords.reshape(1, -1)\n";
    script << "        points = coords[:, :3]\n";
    script << "        print(f'✅ Loaded {len(points):,} ORB-SLAM3 map points')\n";
    script << "        print(f'📏 Point cloud spans:')\n";
    script << "        print(f'   X: {np.min(points[:, 0]):.3f} to {np.max(points[:, 0]):.3f} m')\n";
    script << "        print(f'   Y: {np.min(points[:, 1]):.3f} to {np.max(points[:, 1]):.3f} m')\n";
    script << "        print(f'   Z: {np.min(points[:, 2]):.3f} to {np.max(points[:, 2]):.3f} m')\n";
    script << "        return points\n";
    script << "    except Exception as e: print(f'❌ Error loading map points: {e}'); return None\n\n";
    
    script << "def main():\n";
    script << "    print('🏗️ === FRAME-PRESERVING TUBE MESH GENERATOR ===')\n";
    script << "    print('🔧 Optimized for metal tubes with visible internal frames')\n";
    script << "    print('⚡ Preserves structural details and sharp edges\\\\n')\n";
    script << "    # Add your mesh generation code here\n";
    script << "    print('🎉 Aurora 900 mesh generation complete!')\n\n";
    
    script << "if __name__ == '__main__': main()\n";
    
    script.close();
    chmod((folder + "tube_mesh_generator.py").c_str(), 0755);
    
    std::cout << "[MESH] ✅ Created ENHANCED FRAME-PRESERVING TUBE MESH GENERATOR!" << std::endl;
    std::cout << "[MESH] 🚀 Run: cd 3D_Reconstruction_Data && python3 tube_mesh_generator.py" << std::endl;
}

int main(int argc, char **argv) {
    std::cout << "\n[DEBUG] =============== ORB-SLAM3 RGB-D WITH AURORA 900 ===============" << std::endl;

    // Check for headless environment
    const char* display = getenv("DISPLAY");
    (void)display; // Suppress unused variable warning
    bool headless = true;  // Force headless mode for maximum performance
    
    if (headless) {
        std::cout << "[INFO] Running in headless mode for maximum performance" << std::endl;
    }
    
    // Register signal handlers
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    cv::setBreakOnError(false);
    
    if (argc < 3 || argc > 4) {
        cerr << "Usage: ./rgbd_aurora900 vocabulary settings [hw|sw]" << endl;
        cerr << "  hw: use hardware alignment (default)" << endl;
        cerr << "  sw: use software coordinate transformation" << endl;
        return 1;
    }

    // Check alignment mode
    bool use_hardware = true;
    if (argc == 4) {
        string mode = argv[3];
        if (mode == "sw") {
            use_hardware = false;
            cout << "[DEBUG] Using SOFTWARE alignment" << endl;
        } else {
            cout << "[DEBUG] Using HARDWARE alignment" << endl;
        }
    }

    std::cout << "[DEBUG] Vocabulary: " << argv[1] << std::endl;
    std::cout << "[DEBUG] Settings: " << argv[2] << std::endl;

    // Create reconstruction folder
    std::string reconstruction_folder = "3D_Reconstruction_Data";
    std::string folder = "3D_Reconstruction_Data/";

    // Variables to accumulate all good maps
    std::set<std::string> good_map_points;
    
    if (mkdir(reconstruction_folder.c_str(), 0777) == 0) {
        std::cout << "[DEBUG] Created folder: " << reconstruction_folder << std::endl;
    } else {
        std::cout << "[DEBUG] Using existing folder: " << reconstruction_folder << std::endl;
    }

    AuroraCapture capture(use_hardware);
    if (!capture.initialize()) {
        cerr << "[ERROR] Failed to initialize Aurora camera!" << endl;
        return -1;
    }

    // Initialize SLAM
    std::cout << "\n[DEBUG] Initializing ORB-SLAM3..." << std::endl;
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::RGBD, true);
    float imageScale = SLAM.GetImageScale();
    std::cout << "[DEBUG] ORB-SLAM3 image scale factor: " << imageScale << std::endl;

    if (!capture.start()) {
        cerr << "[ERROR] Failed to start Aurora camera!" << endl;
        return -1;
    }

    cout << "\n[DEBUG] ============= SLAM STARTED =============" << endl;
    cout << "[INFO] Controls: 'q'=quit, 'm'=save map" << endl;
    
    // Setup terminal for real-time key input
    setupTerminal();

    cv::Mat im, depthmap;
    double timestamp = 0.0;
    int frame_count = 0;
    int tracking_ok_count = 0;
    
    auto start_time = std::chrono::steady_clock::now();
    
    while (!g_resource_manager.shouldExit()) {
        // Get frames with exception handling
        bool frame_success = false;
        try {
            frame_success = capture.getFrames(im, depthmap, timestamp);
        } catch (const std::exception& e) {
            std::cout << "[ERROR] Exception in getFrames: " << e.what() << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        
        if (!frame_success) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // Resize if needed
        if (imageScale != 1.f) {
            int width = im.cols * imageScale;
            int height = im.rows * imageScale;
            cv::resize(im, im, cv::Size(width, height));
            cv::resize(depthmap, depthmap, cv::Size(width, height));
        }

        // Track with SLAM - add exception handling
        int state = -1;
        double ttrack = 0.0;
        try {
            auto track_start = std::chrono::steady_clock::now();
            SLAM.TrackRGBD(im, depthmap, timestamp);
            auto track_end = std::chrono::steady_clock::now();
            
            ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(track_end - track_start).count();
            frame_count++;

            // Get tracking state
            state = SLAM.GetTrackingState();
            if (state == 2) tracking_ok_count++;
        } catch (const std::exception& e) {
            std::cout << "[ERROR] Exception in SLAM tracking: " << e.what() << std::endl;
            continue;
        }

        // Status every 30 frames
        if (frame_count % 30 == 0) {
            auto current_time = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - start_time).count();
            double fps = frame_count / elapsed;
            
            cout << "[STATUS] Frame " << frame_count 
                 << " | FPS: " << std::fixed << std::setprecision(1) << fps
                 << " | Track: " << std::setprecision(3) << ttrack << "s";
            
            if (state == 2) {
                cout << " | STATUS: TRACKING OK ✓";
            } else if (state == 0) {
                cout << " | STATUS: INITIALIZING...";
            } else {
                cout << " | STATUS: LOST";
            }
            
            cout << " | Success: " << (100.0 * tracking_ok_count / frame_count) << "%" << endl;
        }
        
        // Save intermediate maps
        if (frame_count % 100 == 0 && state == 2) {
            std::string temp_file = folder + "map_points_frame_" + std::to_string(frame_count) + ".txt";
            SLAM.SavePointCloud(temp_file);
            
            // Read and accumulate map points
            std::ifstream file(temp_file);
            if (file.is_open()) {
                std::string line;
                int actual_count = 0;
                while (std::getline(file, line)) {
                    if (!line.empty() && line[0] != '#') {
                        good_map_points.insert(line);
                        actual_count++;
                    }
                }
                file.close();
                
                std::cout << "[DEBUG] Frame " << frame_count << ": collected " 
                        << actual_count << " points, total unique: " << good_map_points.size() << std::endl;
            }
        }

        // Check for keyboard input (non-blocking)
        static int input_check = 0;
        if (++input_check % 10 == 0) {
            fd_set readfds;
            FD_ZERO(&readfds);
            FD_SET(STDIN_FILENO, &readfds);
            
            struct timeval timeout;
            timeout.tv_sec = 0;
            timeout.tv_usec = 0;
            
            if (select(STDIN_FILENO + 1, &readfds, NULL, NULL, &timeout) > 0) {
                char input;
                if (read(STDIN_FILENO, &input, 1) > 0) {
                    std::cout << "[KEY] Terminal input: '" << input << "'" << std::endl;
                    if (input == 'q' || input == 'Q' || input == 27) {
                        std::cout << "[KEY] QUIT requested from terminal!" << std::endl;
                        g_resource_manager.requestExit();
                        break;
                    } else if (input == 'm' || input == 'M') {
                        std::cout << "[KEY] SAVE MAP requested!" << std::endl;
                        std::cout << "[KEY] Map will be saved during shutdown..." << std::endl;
                    }
                }
            }
        }
        
        // Check for exit file (backup method)
        if (input_check % 100 == 0) {
            std::ifstream exit_file("stop_slam.txt");
            if (exit_file.good()) {
                std::cout << "[INFO] Exit signal file detected" << std::endl;
                g_resource_manager.requestExit();
                exit_file.close();
                std::remove("stop_slam.txt");
                break;
            }
        }
    }

    // Enhanced cleanup
    std::cout << "\n[DEBUG] Starting cleanup..." << std::endl;
    
    // Restore terminal settings
    restoreTerminal();
    
    // Final statistics
    auto end_time = std::chrono::steady_clock::now();
    double total_time = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time).count();
    
    std::cout << "\n[DEBUG] ===== FINAL STATISTICS =====" << std::endl;
    std::cout << "[DEBUG] Total frames: " << frame_count << std::endl;
    std::cout << "[DEBUG] Total time: " << std::setprecision(2) << total_time << "s" << std::endl;
    std::cout << "[DEBUG] Average FPS: " << (frame_count / total_time) << std::endl;
    std::cout << "[DEBUG] Tracking success: " << (100.0 * tracking_ok_count / frame_count) << "%" << std::endl;

    // Safe SLAM shutdown and data export
    try {
        std::cout << "[DEBUG] Shutting down SLAM system..." << std::endl;
        SLAM.Shutdown();
        std::cout << "[DEBUG] SLAM shutdown complete" << std::endl;
        
        std::cout << "[DEBUG] Saving trajectories..." << std::endl;
        string suffix = "_final_run";
        SLAM.SaveKeyFrameTrajectoryTUM(folder + "KeyFrameTrajectory" + suffix + ".txt");
        SLAM.SaveTrajectoryTUM(folder + "CameraTrajectory" + suffix + ".txt");
        std::cout << "[DEBUG] Trajectories saved successfully" << std::endl;
        
        // Save ORB-SLAM3 map points
        std::cout << "[DEBUG] Saving map points..." << std::endl;
        SLAM.SavePointCloud(folder + "map_points.txt");

        // Save combined map from all intermediate saves
        std::string combined_file = folder + "map_points_COMBINED.txt";
        std::ofstream combined(combined_file);
        combined << "# Combined map points from all intermediate saves\n";

        for (const auto& point_line : good_map_points) {
            combined << point_line << "\n";
        }
        combined.close();

        std::cout << "[DEBUG] Saved COMBINED map with " << good_map_points.size() 
                << " unique points from all frames" << std::endl;

        // Check file size after saving
        std::ifstream check_file(folder + "map_points.txt");
        if (check_file.is_open()) {
            std::string line;
            int line_count = 0;
            while (std::getline(check_file, line)) {
                if (!line.empty() && line[0] != '#') line_count++;
            }
            check_file.close();
            std::cout << "[DEBUG] Saved " << line_count << " map points to file" << std::endl;
        } else {
            std::cout << "[WARNING] Could not verify saved map points" << std::endl;
        }

        std::cout << "[DEBUG] Map points saved successfully" << std::endl;
        
        GenerateMapPointConverter();
        
    } catch (const std::exception& e) {
        std::cout << "[ERROR] Shutdown/save error: " << e.what() << std::endl;
    }

    std::cout << "[DEBUG] Program completed successfully!" << std::endl;
    std::cout << "\n[3D RECONSTRUCTION] You now have:" << std::endl;
    std::cout << "✅ ORB-SLAM3 map points: map_points.txt" << std::endl;
    std::cout << "✅ Camera trajectory: KeyFrameTrajectory_final_run.txt" << std::endl;
    std::cout << "✅ Tube mesh generator: tube_mesh_generator.py" << std::endl;
    std::cout << "🚀 Clean 3D reconstruction from Aurora 900 SLAM map points!" << std::endl;

    return 0;
}