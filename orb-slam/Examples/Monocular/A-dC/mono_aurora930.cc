/**
 * ORB-SLAM3 Monocular example for Aurora 930 IR camera
 * Modified from RGB-D version for monocular IR SLAM
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

// Function to save IR frame for reference/backup
void SaveIRFrame(const cv::Mat& ir, int frame_number, double timestamp) {
    try {
        std::string folder = "Monocular_Reconstruction_Data/";
        
        // Create filenames with frame number and timestamp
        std::string ir_filename = folder + "mono_ir_" + std::to_string(frame_number) + ".png";
        std::string info_filename = folder + "mono_info_" + std::to_string(frame_number) + ".txt";
        
        // Save IR image
        bool ir_ok = cv::imwrite(ir_filename, ir);
        
        // Save frame info (timestamp, frame number)
        std::ofstream info_file(info_filename);
        info_file << "frame_number: " << frame_number << std::endl;
        info_file << "timestamp: " << std::fixed << std::setprecision(6) << timestamp << std::endl;
        info_file << "ir_file: " << ir_filename << std::endl;
        info_file.close();
        
        if (ir_ok) {
            std::cout << "[BACKUP] Saved IR frame " << frame_number << " for backup" << std::endl;
        } else {
            std::cout << "[ERROR] Failed to save IR frame " << frame_number << std::endl;
        }
        
    } catch (const cv::Exception& e) {
        std::cout << "[ERROR] IR save error: " << e.what() << std::endl;
    }
}

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
    AuroraCapture() : stream_(nullptr) {
        std::cout << "[DEBUG] Aurora 930 - Monocular IR mode" << std::endl;
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
            std::cout << "\n[DEBUG] ===== AURORA IR INITIALIZATION DEBUG =====" << std::endl;
            
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
            
            // Create stream with IR only
            std::cout << "\n[DEBUG] Creating IR stream..." << std::endl;
            std::vector<StreamType> stream_types;
            stream_types.push_back(StreamType::kIr);
            
            ret = device_->CreateStream(stream_, stream_types);
            if (ret != 0) {
                std::cerr << "[ERROR] Failed to create stream! Return code: " << ret << std::endl;
                return false;
            }
            
            std::cout << "[DEBUG] Aurora IR stream created successfully" << std::endl;
            
            // Set Aurora 930 IR camera parameters
            std::cout << "\n[DEBUG] Setting Aurora 930 IR camera parameters..." << std::endl;
            irK_ = (cv::Mat_<float>(3, 3) << 
                423.1010, 0, 318.1030,
                0, 423.8235, 197.0865, 
                0, 0, 1);

            std::cout << "[DEBUG] Using Aurora 930 IR calibrated parameters:" << std::endl;
            std::cout << "[DEBUG] fx=" << 423.1010 << ", fy=" << 423.8235 << std::endl;
            std::cout << "[DEBUG] cx=" << 318.1030 << ", cy=" << 197.0865 << std::endl;

            return true;
            
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] Aurora initialization error: " << e.what() << std::endl;
            return false;
        }
    }

    bool start() {
        try {
            std::cout << "[DEBUG] Starting Aurora IR stream..." << std::endl;
            int ret = stream_->Start();
            if (ret != 0) {
                std::cerr << "[ERROR] Failed to start stream! Return code: " << ret << std::endl;
                return false;
            }
            std::cout << "[DEBUG] Aurora IR stream started successfully" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            return true;
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] Aurora start error: " << e.what() << std::endl;
            return false;
        }
    }
    
    bool getFrame(cv::Mat& ir_image, double& timestamp) {
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
            
            std::shared_ptr<StreamFrame> ir_frame = nullptr;
            
            // Find IR frame in the frame_ptr vector
            for (auto& frame : frames.frame_ptr) {
                if (frame->frame_type == FrameType::kIrFrame) {
                    ir_frame = frame;
                    break;
                }
            }
            
            if (!ir_frame) {
                if (debug_this_frame) {
                    std::cout << "[WARNING] Missing IR frame" << std::endl;
                }
                return false;
            }
            
            // Convert Aurora IR frame to OpenCV
            // Aurora uses cols/rows for dimensions, data for raw pointer
            cv::Mat ir_temp(ir_frame->rows, ir_frame->cols, CV_8UC1, ir_frame->data.get());
            
            // Clone the data to ensure it persists after frame release
            ir_image = ir_temp.clone();

            // Add this debug block
            if (debug_this_frame) {
                cv::Scalar mean, stddev;
                cv::meanStdDev(ir_image, mean, stddev);
                std::cout << "[DEBUG] IR stats - Mean: " << mean[0] 
                        << ", StdDev: " << stddev[0] << ", Min/Max: ";
                
                double minVal, maxVal;
                cv::minMaxLoc(ir_image, &minVal, &maxVal);
                std::cout << minVal << "/" << maxVal << std::endl;
                
                // Save first frame for inspection
                static bool saved = false;
                if (!saved) {
                    cv::imwrite("debug_ir_frame.png", ir_image);
                    std::cout << "[DEBUG] Saved debug_ir_frame.png for inspection" << std::endl;
                    saved = true;
                }
            }
            
            // Get timestamp (Aurora uses timestamp member)
            timestamp = ir_frame->timestamp / 1000000.0; // Convert microseconds to seconds
            
            if (debug_this_frame) {
                std::cout << "[DEBUG] Aurora IR frame - Size: " << ir_image.size() 
                         << ", Type: " << ir_image.type() << std::endl;
            }
            
            return !ir_image.empty();

        } catch (const std::exception& e) {
            std::cerr << "[ERROR] Aurora frame error: " << e.what() << std::endl;
            return false;
        }
    }
    
private:
    std::shared_ptr<Device> device_;
    Stream* stream_;
    cv::Mat irK_;
};

void GenerateMapPointConverter() {
    std::cout << "[MESH] Generating MONOCULAR POINT CLOUD GENERATOR..." << std::endl;
    std::string reconstruction_folder = "Monocular_Reconstruction_Data";
    std::string folder = "Monocular_Reconstruction_Data/";
    
    std::ofstream script(folder + "monocular_point_cloud_generator.py");
    script << "#!/usr/bin/env python3\n";
    script << "\"\"\"\n";
    script << "MONOCULAR SLAM POINT CLOUD GENERATOR\n";
    script << "Processes ORB-SLAM3 monocular map points for 3D visualization\n";
    script << "Optimized for Aurora 930 IR camera data\n";
    script << "\"\"\"\n\n";
    
    script << "import numpy as np\nimport open3d as o3d\nimport os\nimport time\nimport psutil\n\n";
    
    // Memory monitoring
    script << "def print_memory_status(step=''):\n";
    script << "    mem = psutil.virtual_memory()\n";
    script << "    print(f'💾 Memory: {mem.used / (1024**3):.1f}/{mem.total / (1024**3):.1f}GB ({mem.percent:.1f}%) - {step}')\n\n";
    
    // Load ORB-SLAM points
    script << "def load_orb_slam_map_points(filename):\n";
    script << "    print(f'📊 Loading ORB-SLAM3 monocular map points from {filename}...')\n";
    script << "    if not os.path.exists(filename): print(f'❌ File not found: {filename}'); return None\n";
    script << "    try:\n";
    script << "        coords = np.genfromtxt(filename, delimiter=',', skip_header=1)\n";
    script << "        if coords.size == 0: print('❌ No data in file!'); return None\n";
    script << "        if coords.ndim == 1: coords = coords.reshape(1, -1)\n";
    script << "        points = coords[:, :3]\n";
    script << "        print(f'✅ Loaded {len(points):,} ORB-SLAM3 monocular map points')\n";
    script << "        print(f'📏 Point cloud spans:')\n";
    script << "        print(f'   X: {np.min(points[:, 0]):.3f} to {np.max(points[:, 0]):.3f} m')\n";
    script << "        print(f'   Y: {np.min(points[:, 1]):.3f} to {np.max(points[:, 1]):.3f} m')\n";
    script << "        print(f'   Z: {np.min(points[:, 2]):.3f} to {np.max(points[:, 2]):.3f} m')\n";
    script << "        return points\n";
    script << "    except Exception as e: print(f'❌ Error loading map points: {e}'); return None\n\n";
    
    script << "def main():\n";
    script << "    print('🔍 === MONOCULAR SLAM POINT CLOUD GENERATOR ===')\n";
    script << "    print('📸 Processing Aurora 930 IR camera monocular SLAM data')\n";
    script << "    print('⚡ Generating sparse 3D reconstruction from feature points\\\\n')\n";
    script << "    \n";
    script << "    # Load and process monocular map points\n";
    script << "    points = load_orb_slam_map_points('map_points.txt')\n";
    script << "    if points is not None:\n";
    script << "        pcd = o3d.geometry.PointCloud()\n";
    script << "        pcd.points = o3d.utility.Vector3dVector(points)\n";
    script << "        \n";
    script << "        # Color the points for better visualization\n";
    script << "        colors = np.zeros_like(points)\n";
    script << "        colors[:, 0] = 0.8  # Red channel\n";
    script << "        colors[:, 1] = 0.2  # Green channel  \n";
    script << "        colors[:, 2] = 0.1  # Blue channel\n";
    script << "        pcd.colors = o3d.utility.Vector3dVector(colors)\n";
    script << "        \n";
    script << "        # Save point cloud\n";
    script << "        o3d.io.write_point_cloud('monocular_point_cloud.ply', pcd)\n";
    script << "        print('✅ Saved point cloud as monocular_point_cloud.ply')\n";
    script << "        \n";
    script << "        # Visualize if possible\n";
    script << "        try:\n";
    script << "            o3d.visualization.draw_geometries([pcd], window_name='Aurora 930 Monocular SLAM')\n";
    script << "        except:\n";
    script << "            print('🖥️ No display available for visualization')\n";
    script << "    \n";
    script << "    print('🎉 Aurora 930 monocular point cloud generation complete!')\n\n";
    
    script << "if __name__ == '__main__': main()\n";
    
    script.close();
    chmod((folder + "monocular_point_cloud_generator.py").c_str(), 0755);
    
    std::cout << "[MESH] ✅ Created MONOCULAR POINT CLOUD GENERATOR!" << std::endl;
    std::cout << "[MESH] 🚀 Run: cd Monocular_Reconstruction_Data && python3 monocular_point_cloud_generator.py" << std::endl;
}

int main(int argc, char **argv) {
    std::cout << "\n[DEBUG] =============== ORB-SLAM3 MONOCULAR WITH AURORA 930 IR ===============" << std::endl;

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
    
    if (argc != 3) {
        cerr << "Usage: ./mono_aurora930 vocabulary settings" << endl;
        cerr << "Example: ./mono_aurora930 Vocabulary/ORBvoc.txt Examples/Monocular/Aurora930_IR_Mono.yaml" << endl;
        return 1;
    }

    std::cout << "[DEBUG] Vocabulary: " << argv[1] << std::endl;
    std::cout << "[DEBUG] Settings: " << argv[2] << std::endl;

    // Create reconstruction folder
    std::string reconstruction_folder = "Monocular_Reconstruction_Data";
    std::string folder = "Monocular_Reconstruction_Data/";

    // Variables to accumulate all good maps
    std::set<std::string> good_map_points;
    
    if (mkdir(reconstruction_folder.c_str(), 0777) == 0) {
        std::cout << "[DEBUG] Created folder: " << reconstruction_folder << std::endl;
    } else {
        std::cout << "[DEBUG] Using existing folder: " << reconstruction_folder << std::endl;
    }

    AuroraCapture capture;
    if (!capture.initialize()) {
        cerr << "[ERROR] Failed to initialize Aurora camera!" << endl;
        return -1;
    }

    // Initialize SLAM
    std::cout << "\n[DEBUG] Initializing ORB-SLAM3 Monocular..." << std::endl;
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);
    float imageScale = SLAM.GetImageScale();
    std::cout << "[DEBUG] ORB-SLAM3 image scale factor: " << imageScale << std::endl;

    if (!capture.start()) {
        cerr << "[ERROR] Failed to start Aurora camera!" << endl;
        return -1;
    }

    cout << "\n[DEBUG] ============= MONOCULAR SLAM STARTED =============" << endl;
    cout << "[INFO] Controls: 'q'=quit, 'm'=save map" << endl;
    
    // Setup terminal for real-time key input
    setupTerminal();

    cv::Mat im;
    double timestamp = 0.0;
    int frame_count = 0;
    int tracking_ok_count = 0;
    
    auto start_time = std::chrono::steady_clock::now();
    
    while (!g_resource_manager.shouldExit()) {
        // Get frame with exception handling
        bool frame_success = false;
        try {
            frame_success = capture.getFrame(im, timestamp);
        } catch (const std::exception& e) {
            std::cout << "[ERROR] Exception in getFrame: " << e.what() << std::endl;
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
        }

        // Track with SLAM - add exception handling
        int state = -1;
        double ttrack = 0.0;
        try {
            auto track_start = std::chrono::steady_clock::now();
            // Add this before SLAM.TrackMonocular(im, timestamp);
            if (frame_count % 30 == 0) {
                // Test ORB feature detection on this frame
                cv::Ptr<cv::ORB> orb = cv::ORB::create(1000, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 15);
                std::vector<cv::KeyPoint> keypoints;
                cv::Mat descriptors;
                orb->detectAndCompute(im, cv::Mat(), keypoints, descriptors);
                std::cout << "[DEBUG] ORB detected " << keypoints.size() << " features in current frame" << std::endl;
            }

            // Add this before SLAM.TrackMonocular(im, timestamp);
            if (frame_count % 30 == 0) {
                // Test ORB feature detection on this frame
                cv::Ptr<cv::ORB> orb = cv::ORB::create(1000, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 15);
                std::vector<cv::KeyPoint> keypoints;
                cv::Mat descriptors;
                orb->detectAndCompute(im, cv::Mat(), keypoints, descriptors);
                std::cout << "[DEBUG] ORB detected " << keypoints.size() << " features in current frame" << std::endl;
            }

            // ADD THE MOTION DETECTION HERE:
            static cv::Mat prev_frame;
            if (!prev_frame.empty() && frame_count % 30 == 0) {
                cv::Mat diff;
                cv::absdiff(im, prev_frame, diff);
                cv::Scalar mean_diff = cv::mean(diff);
                std::cout << "[DEBUG] Frame motion: " << mean_diff[0] << std::endl;
            }
            prev_frame = im.clone();

            SLAM.TrackMonocular(im, timestamp);
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
        string suffix = "_monocular_run";
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
    std::cout << "\n[MONOCULAR RECONSTRUCTION] You now have:" << std::endl;
    std::cout << "✅ ORB-SLAM3 map points: map_points.txt" << std::endl;
    std::cout << "✅ Camera trajectory: KeyFrameTrajectory_monocular_run.txt" << std::endl;
    std::cout << "✅ Point cloud generator: monocular_point_cloud_generator.py" << std::endl;
    std::cout << "🚀 Sparse 3D reconstruction from Aurora 930 IR monocular SLAM!" << std::endl;

    return 0;
}