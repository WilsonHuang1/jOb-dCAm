#ifndef ENHANCED_3D_SCANNER_H
#define ENHANCED_3D_SCANNER_H

#include <memory>
#include <vector>
#include <mutex>
#include <thread>
#include <chrono>
#include <deque>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <Eigen/Dense>
#include <open3d/Open3D.h>

/**
 * Enhanced 3D Scanner with SLAM capabilities
 * Based on CCNY RGBD approach for real-time 3D mapping and pose tracking
 */
class Enhanced3DScanner {
public:
    struct Pose {
        Eigen::Matrix4d transformation;
        std::chrono::steady_clock::time_point timestamp;
        double confidence;
        
        Pose() : transformation(Eigen::Matrix4d::Identity()), confidence(1.0) {
            timestamp = std::chrono::steady_clock::now();
        }
        
        // Get translation vector
        Eigen::Vector3d getTranslation() const {
            return transformation.block<3,1>(0,3);
        }
        
        // Get rotation matrix
        Eigen::Matrix3d getRotation() const {
            return transformation.block<3,3>(0,0);
        }
        
        // Calculate distance from another pose
        double distanceTo(const Pose& other) const {
            return (getTranslation() - other.getTranslation()).norm();
        }
        
        // Calculate angular distance from another pose
        double angularDistanceTo(const Pose& other) const {
            Eigen::Matrix3d diff = getRotation() * other.getRotation().transpose();
            return (diff - Eigen::Matrix3d::Identity()).norm();
        }
    };
    
    struct Keyframe {
        cv::Mat rgb_image;
        cv::Mat depth_image;
        std::shared_ptr<open3d::geometry::PointCloud> point_cloud;
        Pose pose;
        int id;
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        std::chrono::steady_clock::time_point creation_time;
        
        Keyframe() : id(-1) {
            creation_time = std::chrono::steady_clock::now();
        }
    };
    
    struct ScannerStats {
        size_t total_frames_processed;
        size_t keyframes_created;
        size_t map_points;
        double total_distance_traveled;
        double current_confidence;
        std::chrono::steady_clock::time_point start_time;
        
        ScannerStats() : total_frames_processed(0), keyframes_created(0), 
                        map_points(0), total_distance_traveled(0.0), 
                        current_confidence(1.0) {
            start_time = std::chrono::steady_clock::now();
        }
    };

private:
    // SLAM components
    std::deque<Keyframe> keyframes_;
    Pose current_pose_;
    Pose previous_pose_;
    std::shared_ptr<open3d::geometry::PointCloud> accumulated_map_;
    ScannerStats stats_;
    
    // Feature detection and tracking
    cv::Ptr<cv::ORB> feature_detector_;
    cv::Ptr<cv::BFMatcher> matcher_;
    
    // Motion estimation parameters
    double max_translation_between_frames_;
    double max_rotation_between_frames_;
    double keyframe_distance_threshold_;
    double keyframe_angle_threshold_;
    int max_keyframes_;
    
    // ICP parameters for pose refinement
    double icp_max_correspondence_distance_;
    int icp_max_iterations_;
    double icp_convergence_criteria_;
    double icp_fitness_threshold_;
    double icp_rmse_threshold_;
    
    // Point cloud processing parameters
    double voxel_size_downsample_;
    double voxel_size_map_;
    int min_points_for_icp_;
    int max_map_points_;
    
    // Feature matching parameters
    int min_feature_matches_;
    double max_feature_distance_;
    int max_features_;
    
    // Thread safety
    mutable std::mutex pose_mutex_;
    mutable std::mutex map_mutex_;
    mutable std::mutex keyframe_mutex_;
    mutable std::mutex stats_mutex_;
    
    // Camera parameters
    open3d::camera::PinholeCameraIntrinsic intrinsics_;
    
    // Debug and visualization flags
    bool debug_mode_;
    bool save_keyframe_images_;
    std::string output_directory_;

public:
    /**
     * Constructor
     * @param camera_intrinsics Camera intrinsic parameters
     * @param debug_mode Enable debug output and visualization
     */
    Enhanced3DScanner(const open3d::camera::PinholeCameraIntrinsic& camera_intrinsics, 
                     bool debug_mode = false);
    
    /**
     * Destructor
     */
    ~Enhanced3DScanner();
    
    /**
     * Main processing function - call this for each frame
     * @param rgb_frame RGB image
     * @param depth_frame Depth image (in millimeters)
     * @return true if processing was successful
     */
    bool processFrame(const cv::Mat& rgb_frame, const cv::Mat& depth_frame);
    
    /**
     * Get current scanner pose
     * @return Current pose with confidence
     */
    Pose getCurrentPose() const;
    
    /**
     * Get previous scanner pose
     * @return Previous pose
     */
    Pose getPreviousPose() const;
    
    /**
     * Get accumulated 3D map
     * @return Shared pointer to accumulated point cloud
     */
    std::shared_ptr<open3d::geometry::PointCloud> getAccumulatedMap() const;
    
    /**
     * Get copy of accumulated map (thread-safe)
     * @return Copy of accumulated point cloud
     */
    std::shared_ptr<open3d::geometry::PointCloud> getAccumulatedMapCopy() const;
    
    /**
     * Get number of keyframes
     * @return Current keyframe count
     */
    size_t getKeyframeCount() const;
    
    /**
     * Get scanner statistics
     * @return Current scanner statistics
     */
    ScannerStats getStats() const;
    
    /**
     * Save accumulated map to file
     * @param filename Output filename (supports .pcd, .ply, .xyz)
     * @return true if saved successfully
     */
    bool saveMap(const std::string& filename) const;
    
    /**
     * Save keyframes as images
     * @param directory Output directory
     * @return true if saved successfully
     */
    bool saveKeyframes(const std::string& directory) const;
    
    /**
     * Reset scanner state
     */
    void reset();
    
    /**
     * Configure scanner parameters
     */
    void setKeyframeThresholds(double distance_threshold, double angle_threshold);
    void setICPParameters(double max_correspondence_distance, int max_iterations, 
                         double convergence_criteria);
    void setFeatureParameters(int max_features, int min_matches, double max_distance);
    void setMapParameters(double voxel_size, int max_points);
    
    /**
     * Enable/disable debug mode
     */
    void setDebugMode(bool enable);
    
    /**
     * Get transformation between two poses
     */
    static Eigen::Matrix4d getRelativeTransform(const Pose& from, const Pose& to);
    
    /**
     * Print current status
     */
    void printStatus() const;

private:
    // Core processing functions
    std::shared_ptr<open3d::geometry::PointCloud> generatePointCloud(
        const cv::Mat& rgb_frame, const cv::Mat& depth_frame);
    
    Pose estimateMotion(const cv::Mat& rgb_frame, const cv::Mat& depth_frame,
                       std::shared_ptr<open3d::geometry::PointCloud> current_cloud);
    
    Eigen::Matrix4d estimateVisualMotion(const cv::Mat& current_frame, 
                                        const Keyframe& reference_keyframe);
    
    Eigen::Matrix4d estimateICPMotion(
        std::shared_ptr<open3d::geometry::PointCloud> source_cloud,
        std::shared_ptr<open3d::geometry::PointCloud> target_cloud);
    
    bool isMotionValid(const Pose& estimated_pose);
    bool shouldCreateKeyframe();
    void createKeyframe(const cv::Mat& rgb_frame, const cv::Mat& depth_frame,
                       std::shared_ptr<open3d::geometry::PointCloud> point_cloud);
    
    void updateAccumulatedMap(std::shared_ptr<open3d::geometry::PointCloud> current_cloud);
    void updateStatistics();
    
    // Utility functions
    std::shared_ptr<open3d::geometry::PointCloud> preprocessPointCloud(
        std::shared_ptr<open3d::geometry::PointCloud> cloud);
    
    void filterFeatureMatches(const std::vector<cv::DMatch>& matches,
                             std::vector<cv::DMatch>& good_matches);
    
    bool validateTransformation(const Eigen::Matrix4d& transformation);
    
    void debugPrint(const std::string& message) const;
    void saveDebugImage(const cv::Mat& image, const std::string& suffix) const;
};

#endif // ENHANCED_3D_SCANNER_H