/**
 * ORB-SLAM3 Stereo example for Orbbec Gemini 335 camera
 * This implementation uses the left and right IR cameras for stereo SLAM
 */

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <iomanip>
#include <cmath>
#include <sys/select.h>
#include <unistd.h>
#include <fstream>
#include <set>

#include <opencv2/opencv.hpp>
#include <System.h>

// Orbbec SDK includes
#include "libobsensor/ObSensor.hpp"

#include "libobsensor/hpp/Utils.hpp"  // for PointCloudHelper
#include <iostream>

using namespace std;

#include <signal.h>
#include <csignal>
#include <atomic>
#include <unistd.h>
#include <sys/select.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/stat.h>

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
    term.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &term);
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
}

void restoreTerminal() {
    struct termios term;
    tcgetattr(STDIN_FILENO, &term);
    term.c_lflag |= (ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &term);
    fcntl(STDIN_FILENO, F_SETFL, 0);
}

// Point cloud variables for PLY export
auto pointCloud = std::make_shared<ob::PointCloudFilter>();
auto align = std::make_shared<ob::Align>(OB_STREAM_COLOR);

class OrbbecStereoCapture {
public:
    OrbbecStereoCapture() 
        : pipeline_(nullptr), config_(nullptr), 
          frame_ready_(false), frame_count_(0) {
        std::cout << "Orbbec Gemini 335 - Stereo Mode" << std::endl;
    }
    
    ~OrbbecStereoCapture() {
        stop();
    }

    bool initialize() {
        try {
            context_ = std::make_shared<ob::Context>();
            auto deviceList = context_->queryDeviceList();
            if (deviceList->getCount() == 0) {
                std::cerr << "No Orbbec device found!" << std::endl;
                return false;
            }
            
            device_ = deviceList->getDevice(0);
            std::cout << "Device: " << device_->getDeviceInfo()->name() << std::endl;
            std::cout << "Serial: " << device_->getDeviceInfo()->serialNumber() << std::endl;
            
            pipeline_ = std::make_shared<ob::Pipeline>(device_);
            config_ = std::make_shared<ob::Config>();
            
            // Check if device supports stereo IR streams
            auto sensorList = device_->getSensorList();
            bool hasLeftIR = false, hasRightIR = false;
            
            for (uint32_t i = 0; i < sensorList->getCount(); i++) {
                auto sensorType = sensorList->getSensorType(i);
                if (sensorType == OB_SENSOR_IR_LEFT) {
                    hasLeftIR = true;
                    std::cout << "Found left IR sensor" << std::endl;
                }
                if (sensorType == OB_SENSOR_IR_RIGHT) {
                    hasRightIR = true;
                    std::cout << "Found right IR sensor" << std::endl;
                }
            }
            
            if (!hasLeftIR || !hasRightIR) {
                std::cerr << "Device does not support stereo IR streams!" << std::endl;
                std::cerr << "Left IR: " << (hasLeftIR ? "YES" : "NO") << std::endl;
                std::cerr << "Right IR: " << (hasRightIR ? "YES" : "NO") << std::endl;
                return false;
            }
            
            // Configure IR streams for stereo at 90 FPS
            auto leftIRProfiles = pipeline_->getStreamProfileList(OB_SENSOR_IR_LEFT);
            auto rightIRProfiles = pipeline_->getStreamProfileList(OB_SENSOR_IR_RIGHT);

            if (leftIRProfiles->getCount() == 0 || rightIRProfiles->getCount() == 0) {
                std::cerr << "IR stereo streams not available!" << std::endl;
                return false;
            }

            // Find profiles that support 90 FPS first, then fallback to 60, then 30
            std::shared_ptr<ob::VideoStreamProfile> leftIRProfile = nullptr;
            std::shared_ptr<ob::VideoStreamProfile> rightIRProfile = nullptr;
            
            // Try 90 FPS first
            for (int i = 0; i < leftIRProfiles->getCount(); i++) {
                auto profile = leftIRProfiles->getProfile(i)->as<ob::VideoStreamProfile>();
                if (profile->fps() == 90) {
                    leftIRProfile = profile;
                    std::cout << "Found left IR profile: " << profile->width() << "x" << profile->height() 
                             << " at " << profile->fps() << " FPS" << std::endl;
                    break;
                }
            }
            
            for (int i = 0; i < rightIRProfiles->getCount(); i++) {
                auto profile = rightIRProfiles->getProfile(i)->as<ob::VideoStreamProfile>();
                if (profile->fps() == 90) {
                    rightIRProfile = profile;
                    std::cout << "Found right IR profile: " << profile->width() << "x" << profile->height() 
                             << " at " << profile->fps() << " FPS" << std::endl;
                    break;
                }
            }
            
            // Final fallback to 30 FPS
            if (!leftIRProfile) {
                for (int i = 0; i < leftIRProfiles->getCount(); i++) {
                    auto profile = leftIRProfiles->getProfile(i)->as<ob::VideoStreamProfile>();
                    if (profile->fps() == 30) {
                        leftIRProfile = profile;
                        std::cout << "Final fallback to left IR profile: " << profile->width() << "x" << profile->height() 
                                 << " at " << profile->fps() << " FPS" << std::endl;
                        break;
                    }
                }
            }
            
            if (!rightIRProfile) {
                for (int i = 0; i < rightIRProfiles->getCount(); i++) {
                    auto profile = rightIRProfiles->getProfile(i)->as<ob::VideoStreamProfile>();
                    if (profile->fps() == 30) {
                        rightIRProfile = profile;
                        std::cout << "Final fallback to right IR profile: " << profile->width() << "x" << profile->height() 
                                 << " at " << profile->fps() << " FPS" << std::endl;
                        break;
                    }
                }
            }
            
            // Use first available if all FPS attempts failed
            if (!leftIRProfile) {
                leftIRProfile = leftIRProfiles->getProfile(0)->as<ob::VideoStreamProfile>();
                std::cout << "Using first available left IR profile: " << leftIRProfile->width() << "x" << leftIRProfile->height() 
                         << " at " << leftIRProfile->fps() << " FPS" << std::endl;
            }
            
            if (!rightIRProfile) {
                rightIRProfile = rightIRProfiles->getProfile(0)->as<ob::VideoStreamProfile>();
                std::cout << "Using first available right IR profile: " << rightIRProfile->width() << "x" << rightIRProfile->height() 
                         << " at " << rightIRProfile->fps() << " FPS" << std::endl;
            }

            config_->enableStream(leftIRProfile);
            config_->enableStream(rightIRProfile);
            
            actual_fps_ = leftIRProfile->fps();
            std::cout << "IR stereo streams configured at " << actual_fps_ << " FPS" << std::endl;
            
            return true;
            
        } catch (const ob::Error& e) {
            std::cerr << "OrbbecSDK error in initialize: " << e.getMessage() << std::endl;
            return false;
        } catch (const std::exception& e) {
            std::cerr << "Standard error in initialize: " << e.what() << std::endl;
            return false;
        }
    }

    bool start() {
        try {
            if (!pipeline_ || !config_) {
                std::cerr << "Pipeline not initialized!" << std::endl;
                return false;
            }
            
            pipeline_->start(config_);
            std::cout << "Camera pipeline started successfully" << std::endl;
            
            // Wait for pipeline to stabilize
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            return true;
            
        } catch (const ob::Error& e) {
            std::cerr << "OrbbecSDK error in start: " << e.getMessage() << std::endl;
            return false;
        } catch (const std::exception& e) {
            std::cerr << "Standard error in start: " << e.what() << std::endl;
            return false;
        }
    }

    void stop() {
        try {
            if (pipeline_) {
                pipeline_->stop();
                std::cout << "Camera pipeline stopped" << std::endl;
            }
        } catch (const ob::Error& e) {
            std::cerr << "OrbbecSDK error in stop: " << e.getMessage() << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "Standard error in stop: " << e.what() << std::endl;
        }
    }

    bool getFrames(cv::Mat& leftImage, cv::Mat& rightImage, double& timestamp) {
        try {
            auto start_time = std::chrono::high_resolution_clock::now();
            
            auto frameSet = pipeline_->waitForFrames(100); // 100ms timeout
            if (frameSet == nullptr) {
                return false;
            }
            
            auto leftFrame = frameSet->getFrame(OB_FRAME_IR_LEFT);
            auto rightFrame = frameSet->getFrame(OB_FRAME_IR_RIGHT);
            
            if (!leftFrame || !rightFrame) {
                return false;
            }
            
            // Get timestamp (use left frame timestamp)
            timestamp = leftFrame->timeStamp() / 1000000.0; // Convert from microseconds to seconds
            
            // Convert frames to OpenCV Mat
            auto leftVideoFrame = leftFrame->as<ob::VideoFrame>();
            auto rightVideoFrame = rightFrame->as<ob::VideoFrame>();
            
            int leftWidth = leftVideoFrame->width();
            int leftHeight = leftVideoFrame->height();
            int rightWidth = rightVideoFrame->width();
            int rightHeight = rightVideoFrame->height();
            
            if (leftWidth != rightWidth || leftHeight != rightHeight) {
                std::cerr << "Left and right frame dimensions don't match!" << std::endl;
                return false;
            }
            
            // Y8 format for IR frames (8-bit grayscale)
            leftImage = cv::Mat(leftHeight, leftWidth, CV_8UC1, (void*)leftVideoFrame->data()).clone();
            rightImage = cv::Mat(rightHeight, rightWidth, CV_8UC1, (void*)rightVideoFrame->data()).clone();
            
            frame_count_++;
            
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            
            if (frame_count_ % 100 == 0) {
                std::cout << "Frame " << frame_count_ << " processed in " << duration.count() << "ms" << std::endl;
            }
            
            return true;
            
        } catch (const ob::Error& e) {
            std::cerr << "OrbbecSDK error in getFrames: " << e.getMessage() << std::endl;
            return false;
        } catch (const std::exception& e) {
            std::cerr << "Standard error in getFrames: " << e.what() << std::endl;
            return false;
        }
    }
    
    int getFrameCount() const { return frame_count_; }
    int getActualFPS() const { return actual_fps_; }

private:
    std::shared_ptr<ob::Context> context_;
    std::shared_ptr<ob::Device> device_;
    std::shared_ptr<ob::Pipeline> pipeline_;
    std::shared_ptr<ob::Config> config_;
    
    bool frame_ready_;
    int frame_count_;
    int actual_fps_;
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
    
    // Analyze geometry
    script << "def analyze_tube_geometry(points):\n";
    script << "    print('🔍 Analyzing tube geometry...')\n";
    script << "    min_coords, max_coords = np.min(points, axis=0), np.max(points, axis=0)\n";
    script << "    dimensions = max_coords - min_coords\n";
    script << "    print(f'📐 Tube dimensions:')\n";
    script << "    print(f'   Width (X):  {dimensions[0]:.3f} m')\n";
    script << "    print(f'   Height (Y): {dimensions[1]:.3f} m')\n";
    script << "    print(f'   Length (Z): {dimensions[2]:.3f} m')\n";
    script << "    volume = np.prod(dimensions)\n";
    script << "    density = len(points) / volume if volume > 0 else 0\n";
    script << "    print(f'📊 Point density: {density:.1f} points/m³')\n";
    script << "    is_sparse = len(points) < 1000 or density < 100\n";
    script << "    if is_sparse: print('⚠️ Sparse point cloud detected - using geometric reconstruction')\n";
    script << "    else: print('✅ Good point density - using standard reconstruction')\n";
    script << "    return {'dimensions': dimensions, 'density': density, 'is_sparse': is_sparse, 'center': (min_coords + max_coords) / 2, 'bbox_min': min_coords, 'bbox_max': max_coords}\n\n";
    
    // Optimized normals
    script << "def tube_optimized_normals(pcd, geometry_info):\n";
    script << "    print('📐 Tube-optimized normal estimation...')\n";
    script << "    point_count = len(pcd.points)\n";
    script << "    if geometry_info['is_sparse']:\n";
    script << "        print('⚡ Sparse tube - using large radius for normals')\n";
    script << "        max_dim = np.max(geometry_info['dimensions'])\n";
    script << "        radius = max(0.05, max_dim / 20)\n";
    script << "        max_nn = min(30, max(10, point_count // 10))\n";
    script << "        search_param = o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn)\n";
    script << "        print(f'   Using radius: {radius:.3f}m, max_nn: {max_nn}')\n";
    script << "    else:\n";
    script << "        print('✨ Dense tube - using standard normals')\n";
    script << "        search_param = o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=20)\n";
    script << "    pcd.estimate_normals(search_param=search_param)\n";
    script << "    try: pcd.orient_normals_consistent_tangent_plane(100); print('✅ Normals oriented consistently')\n";
    script << "    except: print('⚠️ Could not orient normals consistently')\n";
    script << "    return pcd\n\n";
    
    // Alpha shapes with fine detail preservation
    script << "def create_tube_mesh_alpha_shapes(pcd, geometry_info):\n";
    script << "    print('🔨 Creating tube mesh with Alpha Shapes...')\n";
    script << "    min_dim = np.min(geometry_info['dimensions'])\n";
    script << "    base_alpha = min_dim / 15\n";
    script << "    point_density = len(pcd.points) / (geometry_info['dimensions'][0] * geometry_info['dimensions'][1])\n";
    script << "    density_factor = max(0.5, min(2.0, point_density / 1000))\n";
    script << "    alpha_values = [\n";
    script << "        0.0005 / density_factor,  # Very fine detail\n";
    script << "        0.001 / density_factor,   # Fine detail\n";
    script << "        0.002 / density_factor,   # Medium detail\n";
    script << "        0.004 / density_factor,   # Coarser detail\n";
    script << "        0.008 / density_factor    # Backup\n";
    script << "    ]\n";
    script << "    print(f'🎯 Frame-preserving alphas: {[f\"{a*1000:.1f}mm\" for a in alpha_values]}')\n";
    script << "    print(f'🔍 Testing alpha values: {[f\"{a:.4f}\" for a in alpha_values]}')\n";
    script << "    best_mesh, best_triangle_count, best_alpha = None, 0, 0\n";
    script << "    for alpha in alpha_values:\n";
    script << "        try:\n";
    script << "            print(f'   Testing α = {alpha:.4f}...')\n";
    script << "            mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)\n";
    script << "            triangle_count = len(mesh.triangles)\n";
    script << "            print(f'      Result: {triangle_count:,} triangles')\n";
    script << "            if 50 < triangle_count < 200000 and triangle_count > best_triangle_count:\n";
    script << "                if best_mesh is not None: del best_mesh\n";
    script << "                best_mesh, best_triangle_count, best_alpha = mesh, triangle_count, alpha\n";
    script << "            else: del mesh\n";
    script << "        except Exception as e: print(f'      Failed: {e}')\n";
    script << "    if best_mesh is not None:\n";
    script << "        print(f'✅ Best Alpha Shapes result: α={best_alpha:.4f}, {best_triangle_count:,} triangles')\n";
    script << "        return best_mesh\n";
    script << "    else: print('❌ No suitable alpha shapes mesh found'); return None\n\n";
    
    // Ball pivoting with fine detail preservation
    script << "def create_tube_mesh_ball_pivoting(pcd, geometry_info):\n";
    script << "    print('🔨 Creating tube mesh with Ball Pivoting...')\n";
    script << "    try:\n";
    script << "        distances = pcd.compute_nearest_neighbor_distance()\n";
    script << "        avg_dist = np.mean(distances)\n";
    script << "        print(f'📊 Average point distance: {avg_dist:.4f}m')\n";
    script << "        radii = [\n";
    script << "            0.0005,  # 0.5mm - very fine detail\n";
    script << "            0.001,   # 1mm - fine detail\n";
    script << "            0.002,   # 2mm - medium detail\n";
    script << "            0.003,   # 3mm - structural detail\n";
    script << "            0.005,   # 5mm - larger structures\n";
    script << "            0.008,   # 8mm - backup\n";
    script << "            0.012    # 12mm - emergency\n";
    script << "        ]\n";
    script << "        print(f'🎯 Frame-preserving radii: {[f\"{r*1000:.0f}mm\" for r in radii]}')\n";
    script << "        print(f'🔍 Using radii: {[f\"{r:.4f}\" for r in radii]}')\n";
    script << "        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, o3d.utility.DoubleVector(radii))\n";
    script << "        triangle_count = len(mesh.triangles)\n";
    script << "        print(f'✅ Ball Pivoting result: {triangle_count:,} triangles')\n";
    script << "        return mesh if triangle_count > 50 else None\n";
    script << "    except Exception as e: print(f'❌ Ball Pivoting failed: {e}'); return None\n\n";
    
    // Poisson
    script << "def create_tube_mesh_poisson(pcd, geometry_info):\n";
    script << "    print('🔨 Creating tube mesh with Poisson...')\n";
    script << "    try:\n";
    script << "        tube_area = geometry_info['dimensions'][0] * geometry_info['dimensions'][1]\n";
    script << "        point_density = len(pcd.points) / tube_area\n";
    script << "        base_depth = 8\n";
    script << "        if point_density > 2000: depth = base_depth + 2\n";
    script << "        elif point_density > 1000: depth = base_depth + 1\n";
    script << "        else: depth = base_depth\n";
    script << "        if 0.25 <= tube_area <= 0.35: depth += 1\n";
    script << "        print(f'🎯 Optimized depth: {depth} (density: {point_density:.0f} pts/m²)')\n";
    script << "        if tube_area >= 0.25: depth += 1; print(f'📏 Medium tube detected - increased depth to {depth}')\n";
    script << "        mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=depth, width=0, scale=1.1, linear_fit=False)\n";
    script << "        bbox = pcd.get_axis_aligned_bounding_box()\n";
    script << "        mesh = mesh.crop(bbox)\n";
    script << "        triangle_count = len(mesh.triangles)\n";
    script << "        print(f'✅ Poisson result: {triangle_count:,} triangles')\n";
    script << "        return mesh if triangle_count > 100 else None\n";
    script << "    except Exception as e: print(f'❌ Poisson failed: {e}'); return None\n\n";
    
    // Smoothing function
    script << "def smooth_tube_mesh(mesh, iterations=1):\n";
    script << "    \"\"\"Apply very gentle smoothing\"\"\"\n";
    script << "    print(f\"🎨 Gentle smoothing with {iterations} iterations...\")\n";
    script << "    if mesh is None: return None\n";
    script << "    \n";
    script << "    # Try Taubin smoothing which preserves volume better\n";
    script << "    mesh_smooth = mesh.filter_smooth_taubin(\n";
    script << "        number_of_iterations=iterations,\n";
    script << "        lambda_filter=0.3,\n";
    script << "        mu=-0.31  # Fixed parameter name from mu_filter to mu\n";
    script << "    )\n";
    script << "    print(f\"✅ Mesh smoothed with Taubin filter: {len(mesh_smooth.triangles):,}\")\n";
    script << "    return mesh_smooth\n\n";
    
    // Try all methods
    script << "def try_all_tube_methods(pcd, geometry_info):\n";
    script << "    print('\\\\n🔄 Testing refined tube meshing...')\n";
    script << "    \n";
    script << "    dimensions = geometry_info['dimensions']\n";
    script << "    avg_dim = np.mean(dimensions)\n";
    script << "    point_count = len(pcd.points)\n";
    script << "    \n";
    script << "    # More detailed alpha progression since general shape is working\n";
    script << "    alpha_values = [\n";
    script << "        avg_dim / 15,   # Start where it was working\n";
    script << "        avg_dim / 25,   # More detailed\n";
    script << "        avg_dim / 40,   # Even more detailed\n";
    script << "        avg_dim / 60,   # Fine detail\n";
    script << "        avg_dim / 80,   # Very fine detail\n";
    script << "        avg_dim / 120,  # Maximum detail\n";
    script << "    ]\n";
    script << "    \n";
    script << "    print(f'🎯 Refined alphas: {[f\"{a:.3f}m\" for a in alpha_values]}')\n";
    script << "    \n";
    script << "    best_mesh = None\n";
    script << "    best_score = 0\n";
    script << "    \n";
    script << "    for i, alpha in enumerate(alpha_values):\n";
    script << "        try:\n";
    script << "            print(f'   Level {i+1}: α = {alpha:.3f}m...')\n";
    script << "            mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)\n";
    script << "            triangle_count = len(mesh.triangles)\n";
    script << "            \n";
    script << "            # Adjust scoring to favor more detail now that basic shape works\n";
    script << "            if 50 < triangle_count < 100000:  # Allow more triangles\n";
    script << "                # Favor higher detail levels more aggressively\n";
    script << "                detail_bonus = 1.0 + (i * 0.2)  # Bigger bonus for detail\n";
    script << "                score = triangle_count * detail_bonus\n";
    script << "                \n";
    script << "                print(f'      Result: {triangle_count:,} triangles, score: {score:.0f}')\n";
    script << "                \n";
    script << "                if score > best_score:\n";
    script << "                    if best_mesh is not None:\n";
    script << "                        del best_mesh\n";
    script << "                    best_mesh = mesh\n";
    script << "                    best_score = score\n";
    script << "                else:\n";
    script << "                    del mesh\n";
    script << "            else:\n";
    script << "                print(f'      Result: {triangle_count:,} triangles (outside range)')\n";
    script << "                del mesh\n";
    script << "                \n";
    script << "        except Exception as e:\n";
    script << "            print(f'      Failed: {e}')\n";
    script << "    \n";
    script << "    if best_mesh is not None:\n";
    script << "        print(f'✅ Refined result: {len(best_mesh.triangles):,} triangles')\n";
    script << "        return best_mesh\n";
    script << "    \n";
    script << "    return None\n\n";
    
    // Clean mesh
    script << "def clean_tube_mesh(mesh):\n";
    script << "    if mesh is None: return None\n";
    script << "    print('🧹 Minimal cleaning to preserve detail...')\n";
    script << "    original_triangles = len(mesh.triangles)\n";
    script << "    \n";
    script << "    # Only remove the most obvious bad triangles\n";
    script << "    mesh.remove_degenerate_triangles()\n";
    script << "    # Skip other cleaning that might remove detail\n";
    script << "    \n";
    script << "    cleaned_triangles = len(mesh.triangles)\n";
    script << "    removed = original_triangles - cleaned_triangles\n";
    script << "    print(f'🧹 Minimal clean: removed {removed:,} degenerate triangles only')\n";
    script << "    print(f'📊 Preserved mesh: {len(mesh.vertices):,} vertices, {cleaned_triangles:,} triangles')\n";
    script << "    \n";
    script << "    return mesh\n\n";
    
    // Save results
    script << "def save_tube_results(mesh, pcd, geometry_info):\n";
    script << "    print('💾 Saving frame-preserving reconstruction results...')\n";
    script << "    print_memory_status('before saving')\n";
    script << "    try: o3d.io.write_point_cloud('tube_pointcloud.ply', pcd); print('✅ Tube point cloud: tube_pointcloud.ply')\n";
    script << "    except Exception as e: print(f'❌ Point cloud save failed: {e}')\n";
    script << "    if mesh is not None:\n";
    script << "        try:\n";
    script << "            mesh.compute_vertex_normals(); mesh.compute_triangle_normals()\n";
    script << "            o3d.io.write_triangle_mesh('tube_mesh.stl', mesh); print('✅ Frame-preserving tube mesh STL: tube_mesh.stl')\n";
    script << "            o3d.io.write_triangle_mesh('tube_mesh.ply', mesh); print('✅ Frame-preserving tube mesh PLY: tube_mesh.ply')\n";
    script << "            o3d.io.write_triangle_mesh('tube_mesh.obj', mesh); print('✅ Frame-preserving tube mesh OBJ: tube_mesh.obj')\n";
    script << "            print(f'\\\\n📊 Frame-preserving mesh statistics:')\n";
    script << "            print(f'   Vertices: {len(mesh.vertices):,}')\n";
    script << "            print(f'   Triangles: {len(mesh.triangles):,}')\n";
    script << "        except Exception as e: print(f'❌ Mesh save failed: {e}')\n";
    script << "    try:\n";
    script << "        with open('tube_analysis.txt', 'w') as f:\n";
    script << "            f.write('# Frame-Preserving Tube Geometry Analysis\\\\n')\n";
    script << "            f.write(f'Dimensions: {geometry_info[\\\"dimensions\\\"]}\\\\n')\n";
    script << "            f.write(f'Point density: {geometry_info[\\\"density\\\"]:.1f} points/m³\\\\n')\n";
    script << "            f.write(f'Is sparse: {geometry_info[\\\"is_sparse\\\"]}\\\\n')\n";
    script << "            f.write(f'Center: {geometry_info[\\\"center\\\"]}\\\\n')\n";
    script << "        print('✅ Frame analysis: tube_analysis.txt')\n";
    script << "    except Exception as e: print(f'❌ Analysis save failed: {e}')\n";
    script << "    print_memory_status('after saving')\n\n";
    
    // Main function
    script << "def main():\n";
    script << "    print('🏗️ === FRAME-PRESERVING TUBE MESH GENERATOR ===')\n";
    script << "    print('🔧 Optimized for metal tubes with visible internal frames')\n";
    script << "    print('⚡ Preserves structural details and sharp edges\\\\n')\n";
    script << "    mem = psutil.virtual_memory()\n";
    script << "    print(f'💻 System: {mem.total / (1024**3):.1f}GB RAM available')\n";
    script << "    total_start = time.time()\n";
    script << "    map_files = ['ref_map_points.txt', 'map_points_COMBINED.txt']\n";
    script << "    points, source_file = None, ''\n";
    script << "    for filename in map_files:\n";
    script << "        if os.path.exists(filename):\n";
    script << "            points = load_orb_slam_map_points(filename)\n";
    script << "            if points is not None: source_file = filename; break\n";
    script << "    if points is None:\n";
    script << "        print('❌ No valid ORB-SLAM3 map point files found!')\n";
    script << "        print('💡 Make sure you have \\\"map_points.txt\\\" or \\\"ref_map_points.txt\\\"')\n";
    script << "        return\n";
    script << "    print(f'📍 Using map points from: {source_file}')\n";
    script << "    if len(points) < 20:\n";
    script << "        print('⚠️ Too few points for frame reconstruction')\n";
    script << "        print('💡 Need at least 20 points for frame-preserving reconstruction')\n";
    script << "        return\n";
    script << "    geometry_info = analyze_tube_geometry(points)\n";
    script << "    print(f'\\\\n🔄 Creating frame-preserving point cloud from {len(points):,} points...')\n";
    script << "    pcd = o3d.geometry.PointCloud()\n";
    script << "    pcd.points = o3d.utility.Vector3dVector(points)\n";
    script << "    colors = np.tile([0.8, 0.8, 0.9], (len(points), 1))\n";
    script << "    pcd.colors = o3d.utility.Vector3dVector(colors)\n";
    script << "    pcd = tube_optimized_normals(pcd, geometry_info)\n";
    script << "    mesh = try_all_tube_methods(pcd, geometry_info)\n";
    script << "    if mesh is not None: mesh = clean_tube_mesh(mesh)\n";
    script << "    if mesh is not None: mesh = smooth_tube_mesh(mesh, iterations=2)\n";
    script << "    save_tube_results(mesh, pcd, geometry_info)\n";
    script << "    total_time = time.time() - total_start\n";
    script << "    print(f'\\\\n🎉 Frame-preserving reconstruction complete in {total_time:.1f}s!')\n";
    script << "    if mesh is not None:\n";
    script << "        print(f'\\\\n🎯 For SolidWorks (with frame details):')\n";
    script << "        print(f'   📄 Use: tube_mesh.stl')\n";
    script << "        print(f'   🔧 Preserves internal frame structure')\n";
    script << "        print(f'   📐 Maintains both outer geometry and inner details')\n";
    script << "    else:\n";
    script << "        print(f'\\\\n📊 Point cloud only: tube_pointcloud.ply')\n";
    script << "        print(f'💡 Try different scanning angles to capture more frame details')\n";
    script << "    print(f'\\\\n💡 Frame-preserving scanning tips:')\n";
    script << "    print(f'   🔦 Use consistent lighting to avoid shadows in frames')\n";
    script << "    print(f'   📏 Scan from multiple angles to see through frame gaps')\n";
    script << "    print(f'   🎯 Keep steady distance to maintain frame detail resolution')\n";
    script << "    print(f'   ⏰ Move slowly to capture fine frame structures')\n\n";
    script << "if __name__ == '__main__': main()\n";
    
    script.close();
    chmod((folder + "tube_mesh_generator.py").c_str(), 0755);
    
    std::cout << "[MESH] ✅ Created ENHANCED FRAME-PRESERVING TUBE MESH GENERATOR!" << std::endl;
    std::cout << "[MESH] 🔧 Optimized to preserve internal frame structure" << std::endl;
    std::cout << "[MESH] 📐 Uses finer detail settings to capture visible frames" << std::endl;
    std::cout << "[MESH] 🚀 Run: cd 3D_Reconstruction_Data && python3 tube_mesh_generator.py" << std::endl;
}

int main(int argc, char **argv) {
    std::cout << "\n[DEBUG] =============== ORB-SLAM3 STEREO WITH MAP EXPORT ===============" << std::endl;

    // Register signal handlers
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    cv::setBreakOnError(false);

    if (argc != 3) {
        cerr << endl << "Usage: ./stereo_orbbec_gemini335 path_to_vocabulary path_to_settings" << endl;
        return 1;
    }

    cout << endl << "-------" << endl;
    cout << "ORB-SLAM3 Stereo Mode with Orbbec Gemini 335 - Metal Tube Optimized" << endl;
    cout << "-------" << endl;

    // Create reconstruction folder
    std::string reconstruction_folder = "3D_Reconstruction_Data";
    std::string folder = "3D_Reconstruction_Data/";

    if (mkdir(reconstruction_folder.c_str(), 0777) == 0) {
        std::cout << "[DEBUG] Created folder: " << reconstruction_folder << std::endl;
    } else {
        std::cout << "[DEBUG] Using existing folder: " << reconstruction_folder << std::endl;
    }

    std::set<std::string> good_map_points;

    // Initialize Orbbec camera
    OrbbecStereoCapture capture;
    if (!capture.initialize()) {
        cerr << "Failed to initialize Orbbec camera!" << endl;
        return -1;
    }

    // Create SLAM system
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, true);
    float imageScale = SLAM.GetImageScale();

    if (!capture.start()) {
        cerr << "Failed to start camera capture!" << endl;
        return -1;
    }

    cout << "\n[DEBUG] ============= STEREO SLAM STARTED =============" << endl;
    cout << "[INFO] Controls: 'q'=quit, 'm'=save map" << endl;
    cout << "Starting SLAM with " << capture.getActualFPS() << " FPS camera..." << endl;
    
    // Setup terminal for real-time key input
    setupTerminal();

    // Main processing loop
    cv::Mat leftImage, rightImage;
    double timestamp;
    int frame_count = 0;
    int tracking_ok_count = 0;
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    while (!g_resource_manager.shouldExit()) {
        if (!capture.getFrames(leftImage, rightImage, timestamp)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        
        if (leftImage.empty() || rightImage.empty()) {
            continue;
        }

        // Scale images if necessary
        if (imageScale != 1.0f) {
            int newWidth = leftImage.cols * imageScale;
            int newHeight = leftImage.rows * imageScale;
            cv::resize(leftImage, leftImage, cv::Size(newWidth, newHeight));
            cv::resize(rightImage, rightImage, cv::Size(newWidth, newHeight));
        }

        // Track with SLAM
        int state = -1;
        double ttrack = 0.0;
        try {
            auto slam_start = std::chrono::high_resolution_clock::now();
            SLAM.TrackStereo(leftImage, rightImage, timestamp);
            auto slam_end = std::chrono::high_resolution_clock::now();
            
            ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(slam_end - slam_start).count();
            frame_count++;

            // Get tracking state
            state = SLAM.GetTrackingState();
            if (state == 2) tracking_ok_count++;
        } catch (const std::exception& e) {
            std::cout << "[ERROR] Exception in SLAM tracking: " << e.what() << std::endl;
            continue;
        }
        
        // Performance monitoring
        if (frame_count % 30 == 0) {
            auto current_time = std::chrono::high_resolution_clock::now();
            auto total_duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time);
            double fps = static_cast<double>(frame_count) / total_duration.count();
            
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

        // Save map points every 100 frames when tracking is good
        if (frame_count % 100 == 0 && state == 2) {
            std::string temp_file = folder + "map_points_frame_" + std::to_string(frame_count) + ".txt";
            SLAM.SavePointCloud(temp_file);
            
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
                
                std::cout << "[DEBUG] Frame " << frame_count << ": saved " 
                        << actual_count << " points, total unique: " 
                        << good_map_points.size() << std::endl;
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

    // Enhanced cleanup and export
    std::cout << "\n[DEBUG] Starting cleanup..." << std::endl;
    
    // Restore terminal settings
    restoreTerminal();
    
    // Final statistics
    auto end_time = std::chrono::high_resolution_clock::now();
    double total_time = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time).count();
    
    std::cout << "\n[DEBUG] ===== FINAL STATISTICS =====" << std::endl;
    std::cout << "[DEBUG] Total frames: " << frame_count << std::endl;
    std::cout << "[DEBUG] Total time: " << std::setprecision(2) << total_time << "s" << std::endl;
    std::cout << "[DEBUG] Average FPS: " << (frame_count / total_time) << std::endl;
    std::cout << "[DEBUG] Tracking success: " << (100.0 * tracking_ok_count / frame_count) << "%" << std::endl;

    // // Define folder path
    // std::string folder = "3D_Reconstruction_Data/";

    // Safe SLAM shutdown and data export
    try {
        std::cout << "[DEBUG] Shutting down SLAM system..." << std::endl;
        SLAM.Shutdown();
        std::cout << "[DEBUG] SLAM shutdown complete" << std::endl;
        
        std::cout << "[DEBUG] Saving trajectories..." << std::endl;
        string suffix = "_stereo_final_run";
        SLAM.SaveKeyFrameTrajectoryTUM(folder + "KeyFrameTrajectory" + suffix + ".txt");
        SLAM.SaveTrajectoryTUM(folder + "CameraTrajectory" + suffix + ".txt");
        std::cout << "[DEBUG] Trajectories saved successfully" << std::endl;
        
        // Save ORB-SLAM3 map points
        std::cout << "[DEBUG] Saving ORB-SLAM3 map points..." << std::endl;
        SLAM.SavePointCloud(folder + "map_points.txt");
        std::cout << "[DEBUG] Map points saved successfully" << std::endl;

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
        
        // Save camera parameters for stereo
        std::cout << "[DEBUG] Saving stereo camera parameters..." << std::endl;
        std::ofstream cam_file(folder + "camera_params.txt");
        cam_file << "# Stereo camera parameters from IR cameras" << std::endl;
        cam_file << "mode: stereo" << std::endl;
        cam_file << "left_fx: 640.0" << std::endl;
        cam_file << "left_fy: 640.0" << std::endl;
        cam_file << "left_cx: 320.0" << std::endl;
        cam_file << "left_cy: 240.0" << std::endl;
        cam_file << "baseline: 0.05" << std::endl;
        cam_file << "width: 640" << std::endl;
        cam_file << "height: 480" << std::endl;
        cam_file.close();
        std::cout << "[DEBUG] Camera parameters saved" << std::endl;
        
        GenerateMapPointConverter();
        
    } catch (const std::exception& e) {
        std::cout << "[ERROR] Shutdown/save error: " << e.what() << std::endl;
    }

    std::cout << "[DEBUG] Program completed successfully!" << std::endl;
    std::cout << "\n[3D RECONSTRUCTION] You now have:" << std::endl;
    std::cout << "✅ ORB-SLAM3 map points: map_points.txt" << std::endl;
    std::cout << "✅ Camera trajectory: KeyFrameTrajectory_stereo_final_run.txt" << std::endl;
    std::cout << "✅ Camera parameters: camera_params.txt" << std::endl;
    std::cout << "✅ Tube mesh generator: tube_mesh_generator.py" << std::endl;
    std::cout << "🚀 Stereo reconstruction optimized for metal tubes!" << std::endl;

    return 0;
}