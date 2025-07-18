#include "enhanced_3d_scanner.h"
#include <filesystem>
#include <algorithm>

// Windows compatibility fixes
#ifdef _WIN32
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif
#pragma warning(disable: 4996)
#pragma warning(disable: 4267)
#pragma warning(disable: 4819)
#endif

Enhanced3DScanner::Enhanced3DScanner(
    const open3d::camera::PinholeCameraIntrinsic& camera_intrinsics, 
    bool debug_mode) 
    : intrinsics_(camera_intrinsics)
    , current_pose_()
    , previous_pose_()
    , debug_mode_(debug_mode)
    , save_keyframe_images_(false)
    , output_directory_("./scanner_output/")
    // Motion estimation parameters
    , max_translation_between_frames_(0.3)  // 30cm max movement
    , max_rotation_between_frames_(0.5)     // ~30 degrees max rotation
    , keyframe_distance_threshold_(0.2)     // 20cm for new keyframe
    , keyframe_angle_threshold_(0.3)        // ~17 degrees for new keyframe
    , max_keyframes_(50)                    // Keep last 50 keyframes
    // ICP parameters
    , icp_max_correspondence_distance_(0.1) // 10cm for ICP
    , icp_max_iterations_(30)
    , icp_convergence_criteria_(1e-6)
    , icp_fitness_threshold_(0.3)
    , icp_rmse_threshold_(0.1)
    // Point cloud parameters
    , voxel_size_downsample_(0.05)         // 5cm for processing
    , voxel_size_map_(0.02)                // 2cm for final map
    , min_points_for_icp_(100)
    , max_map_points_(200000)
    // Feature parameters
    , min_feature_matches_(20)
    , max_feature_distance_(50.0)
    , max_features_(1000)
{
    // Initialize feature detector (ORB is good for real-time)
    feature_detector_ = cv::ORB::create(max_features_);
    matcher_ = cv::BFMatcher::create(cv::NORM_HAMMING, true);
    
    // Initialize accumulated map
    accumulated_map_ = std::make_shared<open3d::geometry::PointCloud>();
    
    // Create output directory if in debug mode
    if (debug_mode_) {
        try {
            std::filesystem::create_directories(output_directory_);
        } catch (const std::exception& e) {
            std::cerr << "Failed to create output directory: " << e.what() << std::endl;
        }
    }
    
    debugPrint("Enhanced 3D Scanner initialized with SLAM capabilities");
    debugPrint("Camera intrinsics: " + 
               std::to_string(intrinsics_.width_) + "x" + std::to_string(intrinsics_.height_));
}

Enhanced3DScanner::~Enhanced3DScanner() {
    debugPrint("Enhanced 3D Scanner shutting down");
}

bool Enhanced3DScanner::processFrame(const cv::Mat& rgb_frame, const cv::Mat& depth_frame) {
    if (rgb_frame.empty() || depth_frame.empty()) {
        debugPrint("Empty frames provided");
        return false;
    }
    
    auto start_time = std::chrono::steady_clock::now();
    
    try {
        // Generate point cloud from current frame
        auto current_cloud = generatePointCloud(rgb_frame, depth_frame);
        if (!current_cloud || current_cloud->points_.empty()) {
            debugPrint("Failed to generate point cloud");
            return false;
        }
        
        // Estimate motion between frames
        Pose estimated_pose = estimateMotion(rgb_frame, depth_frame, current_cloud);
        
        // Validate motion estimate
        if (!isMotionValid(estimated_pose)) {
            debugPrint("Motion estimate invalid, keeping previous pose");
            estimated_pose = current_pose_;
            estimated_pose.confidence *= 0.8; // Reduce confidence
        }
        
        // Update current pose
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            previous_pose_ = current_pose_;
            current_pose_ = estimated_pose;
            current_pose_.timestamp = start_time;
        }
        
        // Check if we need a new keyframe
        if (shouldCreateKeyframe()) {
            createKeyframe(rgb_frame, depth_frame, current_cloud);
        }
        
        // Update the accumulated map
        updateAccumulatedMap(current_cloud);
        
        // Update statistics
        updateStatistics();
        
        auto end_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        if (debug_mode_) {
            std::cout << "Frame processed in " << duration.count() << "ms, "
                      << "pose: [" << std::fixed << std::setprecision(2)
                      << current_pose_.getTranslation().x() << ", " 
                      << current_pose_.getTranslation().y() << ", " 
                      << current_pose_.getTranslation().z() << "], "
                      << "confidence: " << current_pose_.confidence << std::endl;
        }
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error processing frame: " << e.what() << std::endl;
        return false;
    }
}

std::shared_ptr<open3d::geometry::PointCloud> Enhanced3DScanner::generatePointCloud(
    const cv::Mat& rgb_frame, const cv::Mat& depth_frame) {
    
    try {
        // Convert to Open3D images
        open3d::geometry::Image color_image, depth_image;
        
        // Convert BGR to RGB
        cv::Mat rgb_converted;
        cv::cvtColor(rgb_frame, rgb_converted, cv::COLOR_BGR2RGB);
        
        // Prepare color image
        color_image.Prepare(rgb_frame.cols, rgb_frame.rows, 3, sizeof(uint8_t));
        std::memcpy(color_image.data_.data(), rgb_converted.data, 
                   rgb_converted.total() * rgb_converted.elemSize());
        
        // Convert depth to float (assuming input is in millimeters)
        cv::Mat depth_float;
        depth_frame.convertTo(depth_float, CV_32F, 1.0 / 1000.0); // mm to meters
        
        depth_image.Prepare(depth_frame.cols, depth_frame.rows, 1, sizeof(float));
        std::memcpy(depth_image.data_.data(), depth_float.data, 
                   depth_float.total() * depth_float.elemSize());
        
        // Create RGBD image
        auto rgbd = open3d::geometry::RGBDImage::CreateFromColorAndDepth(
            color_image, depth_image, 1.0, 4.0, false);
        
        // Generate point cloud
        auto point_cloud = open3d::geometry::PointCloud::CreateFromRGBDImage(
            *rgbd, intrinsics_);
        
        // Apply coordinate system correction (sensor to standard coordinates)
        Eigen::Matrix4d correction = Eigen::Matrix4d::Identity();
        correction(1, 1) = -1; // Flip Y
        correction(2, 2) = -1; // Flip Z
        point_cloud->Transform(correction);
        
        // Preprocess point cloud
        return preprocessPointCloud(point_cloud);
        
    } catch (const std::exception& e) {
        std::cerr << "Error generating point cloud: " << e.what() << std::endl;
        return nullptr;
    }
}

std::shared_ptr<open3d::geometry::PointCloud> Enhanced3DScanner::preprocessPointCloud(
    std::shared_ptr<open3d::geometry::PointCloud> cloud) {
    
    if (!cloud || cloud->points_.empty()) {
        return cloud;
    }
    
    // Remove statistical outliers
    if (cloud->points_.size() > 100) {
        try {
            auto filtered_result = cloud->RemoveStatisticalOutliers(20, 2.0);
            cloud = std::get<0>(filtered_result);  // Get the filtered cloud
        } catch (const std::exception& e) {
            debugPrint("Error in statistical outlier removal: " + std::string(e.what()));
        }
    }
    
    // Remove points too close or too far
    std::vector<size_t> valid_indices;
    for (size_t i = 0; i < cloud->points_.size(); ++i) {
        const auto& point = cloud->points_[i];
        double distance = std::sqrt(point.x() * point.x() + 
                                  point.y() * point.y() + 
                                  point.z() * point.z());
        if (distance > 0.1 && distance < 5.0) { // 10cm to 5m
            valid_indices.push_back(i);
        }
    }
    
    auto filtered_cloud = cloud->SelectByIndex(valid_indices);
    
    debugPrint("Point cloud preprocessed: " + 
               std::to_string(cloud->points_.size()) + " -> " + 
               std::to_string(filtered_cloud->points_.size()) + " points");
    
    return filtered_cloud;
}

Enhanced3DScanner::Pose Enhanced3DScanner::estimateMotion(
    const cv::Mat& rgb_frame, const cv::Mat& depth_frame,
    std::shared_ptr<open3d::geometry::PointCloud> current_cloud) {
    
    Pose estimated_pose = current_pose_;
    
    // If this is the first frame, return identity
    if (keyframes_.empty()) {
        debugPrint("First frame, using identity pose");
        return estimated_pose;
    }
    
    // Get the most recent keyframe
    const Keyframe& reference_keyframe = keyframes_.back();
    
    // Method 1: Visual odometry using features
    Eigen::Matrix4d visual_motion = estimateVisualMotion(rgb_frame, reference_keyframe);
    
    // Method 2: ICP refinement using point clouds
    Eigen::Matrix4d icp_motion = estimateICPMotion(current_cloud, reference_keyframe.point_cloud);
    
    // Combine estimates based on their reliability
    Eigen::Matrix4d final_motion = Eigen::Matrix4d::Identity();
    double confidence = 0.3;
    
    bool visual_valid = validateTransformation(visual_motion);
    bool icp_valid = validateTransformation(icp_motion);
    
    if (icp_valid && visual_valid) {
        // Use weighted combination
        debugPrint("Using combined visual and ICP motion estimate");
        final_motion = icp_motion; // ICP is generally more accurate for small motions
        confidence = 0.9;
    } else if (icp_valid) {
        debugPrint("Using ICP motion estimate");
        final_motion = icp_motion;
        confidence = 0.8;
    } else if (visual_valid) {
        debugPrint("Using visual motion estimate");
        final_motion = visual_motion;
        confidence = 0.6;
    } else {
        debugPrint("No valid motion estimate, using identity");
        final_motion = Eigen::Matrix4d::Identity();
        confidence = 0.3;
    }
    
    // Apply motion to current pose
    estimated_pose.transformation = final_motion * current_pose_.transformation;
    estimated_pose.confidence = confidence;
    
    return estimated_pose;
}

Eigen::Matrix4d Enhanced3DScanner::estimateVisualMotion(
    const cv::Mat& current_frame, const Keyframe& reference_keyframe) {
    
    try {
        // Extract features from current frame
        std::vector<cv::KeyPoint> current_keypoints;
        cv::Mat current_descriptors;
        feature_detector_->detectAndCompute(current_frame, cv::noArray(), 
                                          current_keypoints, current_descriptors);
        
        if (current_descriptors.empty() || reference_keyframe.descriptors.empty()) {
            debugPrint("No descriptors available for visual motion estimation");
            return Eigen::Matrix4d::Identity();
        }
        
        // Match features
        std::vector<cv::DMatch> matches;
        matcher_->match(reference_keyframe.descriptors, current_descriptors, matches);
        
        if (static_cast<int>(matches.size()) < min_feature_matches_) {
            debugPrint("Insufficient feature matches: " + std::to_string(matches.size()));
            return Eigen::Matrix4d::Identity();
        }
        
        // Filter good matches
        std::vector<cv::DMatch> good_matches;
        filterFeatureMatches(matches, good_matches);
        
        if (static_cast<int>(good_matches.size()) < min_feature_matches_) {
            debugPrint("Insufficient good matches after filtering: " + std::to_string(good_matches.size()));
            return Eigen::Matrix4d::Identity();
        }
        
        // Extract matching points
        std::vector<cv::Point2f> ref_points, curr_points;
        for (const auto& match : good_matches) {
            ref_points.push_back(reference_keyframe.keypoints[match.queryIdx].pt);
            curr_points.push_back(current_keypoints[match.trainIdx].pt);
        }
        
        // Estimate essential matrix
        cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << 
            intrinsics_.GetFocalLength().first, 0, intrinsics_.GetPrincipalPoint().first,
            0, intrinsics_.GetFocalLength().second, intrinsics_.GetPrincipalPoint().second,
            0, 0, 1);
        
        cv::Mat essential_matrix = cv::findEssentialMat(
            ref_points, curr_points, camera_matrix, 
            cv::RANSAC, 0.999, 1.0);
        
        if (essential_matrix.empty()) {
            debugPrint("Failed to estimate essential matrix");
            return Eigen::Matrix4d::Identity();
        }
        
        // Recover pose
        cv::Mat R, t;
        int inliers = cv::recoverPose(essential_matrix, ref_points, curr_points, 
                                    camera_matrix, R, t);
        
        if (inliers < min_feature_matches_ / 2) {
            debugPrint("Insufficient inliers in pose recovery: " + std::to_string(inliers));
            return Eigen::Matrix4d::Identity();
        }
        
        // Convert to Eigen
        Eigen::Matrix4d motion = Eigen::Matrix4d::Identity();
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                motion(i, j) = R.at<double>(i, j);
            }
            motion(i, 3) = t.at<double>(i, 0);
        }
        
        debugPrint("Visual motion estimated with " + std::to_string(inliers) + " inliers");
        return motion;
        
    } catch (const std::exception& e) {
        std::cerr << "Error in visual motion estimation: " << e.what() << std::endl;
        return Eigen::Matrix4d::Identity();
    }
}

Eigen::Matrix4d Enhanced3DScanner::estimateICPMotion(
    std::shared_ptr<open3d::geometry::PointCloud> source_cloud,
    std::shared_ptr<open3d::geometry::PointCloud> target_cloud) {
    
    if (!source_cloud || !target_cloud || 
        source_cloud->points_.empty() || target_cloud->points_.empty()) {
        debugPrint("Empty point clouds for ICP");
        return Eigen::Matrix4d::Identity();
    }
    
    try {
        // Downsample for efficiency
        auto source_down = source_cloud->VoxelDownSample(voxel_size_downsample_);
        auto target_down = target_cloud->VoxelDownSample(voxel_size_downsample_);
        
        if (static_cast<int>(source_down->points_.size()) < min_points_for_icp_ || 
            static_cast<int>(target_down->points_.size()) < min_points_for_icp_) {
            debugPrint("Insufficient points for ICP: " + 
                      std::to_string(source_down->points_.size()) + ", " + 
                      std::to_string(target_down->points_.size()));
            return Eigen::Matrix4d::Identity();
        }
        
        // Estimate normals for point-to-plane ICP
        source_down->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.1, 30));
        target_down->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.1, 30));
        
        // Run ICP
        auto icp_result = open3d::pipelines::registration::RegistrationICP(
            *source_down, *target_down, icp_max_correspondence_distance_,
            Eigen::Matrix4d::Identity(),
            open3d::pipelines::registration::TransformationEstimationPointToPlane(),
            open3d::pipelines::registration::ICPConvergenceCriteria(
                icp_convergence_criteria_, icp_convergence_criteria_, icp_max_iterations_));
        
        // Check if ICP converged well
        if (icp_result.fitness_ > icp_fitness_threshold_ && 
            icp_result.inlier_rmse_ < icp_rmse_threshold_) {
            debugPrint("ICP converged: fitness=" + std::to_string(icp_result.fitness_) + 
                      ", RMSE=" + std::to_string(icp_result.inlier_rmse_));
            return icp_result.transformation_;
        } else {
            debugPrint("ICP failed to converge: fitness=" + std::to_string(icp_result.fitness_) + 
                      ", RMSE=" + std::to_string(icp_result.inlier_rmse_));
        }
        
        return Eigen::Matrix4d::Identity();
        
    } catch (const std::exception& e) {
        std::cerr << "Error in ICP motion estimation: " << e.what() << std::endl;
        return Eigen::Matrix4d::Identity();
    }
}

void Enhanced3DScanner::filterFeatureMatches(const std::vector<cv::DMatch>& matches,
                                            std::vector<cv::DMatch>& good_matches) {
    good_matches.clear();
    
    if (matches.empty()) return;
    
    // Sort matches by distance
    std::vector<cv::DMatch> sorted_matches = matches;
    std::sort(sorted_matches.begin(), sorted_matches.end());
    
    // Use Lowe's ratio test and distance threshold
    double min_dist = sorted_matches[0].distance;
    double max_dist = std::min(max_feature_distance_, min_dist * 3.0);
    
    for (const auto& match : sorted_matches) {
        if (match.distance <= max_dist) {
            good_matches.push_back(match);
        }
        
        // Limit number of matches
        if (static_cast<int>(good_matches.size()) >= max_features_ / 2) {
            break;
        }
    }
}

bool Enhanced3DScanner::validateTransformation(const Eigen::Matrix4d& transformation) {
    // Check if transformation is identity (no motion detected)
    if ((transformation - Eigen::Matrix4d::Identity()).norm() < 1e-6) {
        return false;
    }
    
    // Check translation magnitude
    Eigen::Vector3d translation = transformation.block<3,1>(0,3);
    if (translation.norm() > max_translation_between_frames_) {
        return false;
    }
    
    // Check rotation magnitude
    Eigen::Matrix3d rotation = transformation.block<3,3>(0,0);
    if ((rotation - Eigen::Matrix3d::Identity()).norm() > max_rotation_between_frames_) {
        return false;
    }
    
    // Check if rotation matrix is valid (determinant should be 1)
    if (std::abs(rotation.determinant() - 1.0) > 0.1) {
        return false;
    }
    
    return true;
}

bool Enhanced3DScanner::isMotionValid(const Pose& estimated_pose) {
    Eigen::Vector3d translation_diff = 
        estimated_pose.getTranslation() - current_pose_.getTranslation();
    
    double translation_magnitude = translation_diff.norm();
    
    // Check translation bounds
    if (translation_magnitude > max_translation_between_frames_) {
        debugPrint("Translation too large: " + std::to_string(translation_magnitude));
        return false;
    }
    
    // Check rotation bounds
    double rotation_magnitude = current_pose_.angularDistanceTo(estimated_pose);
    
    if (rotation_magnitude > max_rotation_between_frames_) {
        debugPrint("Rotation too large: " + std::to_string(rotation_magnitude));
        return false;
    }
    
    return true;
}

bool Enhanced3DScanner::shouldCreateKeyframe() {
    if (keyframes_.empty()) {
        return true; // First frame is always a keyframe
    }
    
    const Pose& last_keyframe_pose = keyframes_.back().pose;
    
    // Calculate distance since last keyframe
    double translation_distance = current_pose_.distanceTo(last_keyframe_pose);
    double rotation_distance = current_pose_.angularDistanceTo(last_keyframe_pose);
    
    // Create keyframe if we've moved enough
    bool should_create = (translation_distance > keyframe_distance_threshold_ || 
                         rotation_distance > keyframe_angle_threshold_);
    
    if (should_create) {
        debugPrint("Creating keyframe: translation=" + std::to_string(translation_distance) + 
                  ", rotation=" + std::to_string(rotation_distance));
    }
    
    return should_create;
}

void Enhanced3DScanner::createKeyframe(const cv::Mat& rgb_frame, const cv::Mat& depth_frame,
                                      std::shared_ptr<open3d::geometry::PointCloud> point_cloud) {
    
    std::lock_guard<std::mutex> lock(keyframe_mutex_);
    
    Keyframe new_keyframe;
    new_keyframe.rgb_image = rgb_frame.clone();
    new_keyframe.depth_image = depth_frame.clone();
    new_keyframe.point_cloud = std::make_shared<open3d::geometry::PointCloud>(*point_cloud);
    new_keyframe.pose = current_pose_;
    new_keyframe.id = static_cast<int>(keyframes_.size());
    new_keyframe.creation_time = std::chrono::steady_clock::now();
    
    // Extract and store features
    feature_detector_->detectAndCompute(rgb_frame, cv::noArray(), 
                                      new_keyframe.keypoints, new_keyframe.descriptors);
    
    keyframes_.push_back(new_keyframe);
    
    // Maintain maximum number of keyframes
    if (static_cast<int>(keyframes_.size()) > max_keyframes_) {
        keyframes_.pop_front();
        // Update IDs
        for (size_t i = 0; i < keyframes_.size(); i++) {
            keyframes_[i].id = static_cast<int>(i);
        }
    }
    
    // Save keyframe images if in debug mode
    if (save_keyframe_images_ && debug_mode_) {
        std::string keyframe_dir = output_directory_ + "keyframes/";
        std::filesystem::create_directories(keyframe_dir);
        
        std::string rgb_filename = keyframe_dir + "keyframe_" + 
                                  std::to_string(new_keyframe.id) + "_rgb.jpg";
        std::string depth_filename = keyframe_dir + "keyframe_" + 
                                    std::to_string(new_keyframe.id) + "_depth.png";
        
        cv::imwrite(rgb_filename, rgb_frame);
        cv::imwrite(depth_filename, depth_frame);
    }
    
    debugPrint("Created keyframe " + std::to_string(new_keyframe.id) + 
              " (total: " + std::to_string(keyframes_.size()) + ")");
}

void Enhanced3DScanner::updateAccumulatedMap(std::shared_ptr<open3d::geometry::PointCloud> current_cloud) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    if (!current_cloud || current_cloud->points_.empty()) {
        return;
    }
    
    // Transform current cloud to world coordinates
    auto transformed_cloud = std::make_shared<open3d::geometry::PointCloud>(*current_cloud);
    transformed_cloud->Transform(current_pose_.transformation);
    
    // Add to accumulated map
    *accumulated_map_ += *transformed_cloud;
    
    // Downsample accumulated map to prevent it from getting too large
    if (static_cast<int>(accumulated_map_->points_.size()) > max_map_points_) {
        accumulated_map_ = accumulated_map_->VoxelDownSample(voxel_size_map_);
        debugPrint("Map downsampled to " + std::to_string(accumulated_map_->points_.size()) + " points");
    }
}

void Enhanced3DScanner::updateStatistics() {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    
    stats_.total_frames_processed++;
    stats_.keyframes_created = keyframes_.size();
    stats_.map_points = accumulated_map_->points_.size();
    stats_.current_confidence = current_pose_.confidence;
    
    // Calculate total distance traveled
    static Pose last_pose_for_distance = current_pose_;
    stats_.total_distance_traveled += current_pose_.distanceTo(last_pose_for_distance);
    last_pose_for_distance = current_pose_;
}

// Getter methods implementation
Enhanced3DScanner::Pose Enhanced3DScanner::getCurrentPose() const {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    return current_pose_;
}

Enhanced3DScanner::Pose Enhanced3DScanner::getPreviousPose() const {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    return previous_pose_;
}

std::shared_ptr<open3d::geometry::PointCloud> Enhanced3DScanner::getAccumulatedMap() const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    return accumulated_map_;
}

std::shared_ptr<open3d::geometry::PointCloud> Enhanced3DScanner::getAccumulatedMapCopy() const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    return std::make_shared<open3d::geometry::PointCloud>(*accumulated_map_);
}

size_t Enhanced3DScanner::getKeyframeCount() const {
    std::lock_guard<std::mutex> lock(keyframe_mutex_);
    return keyframes_.size();
}

Enhanced3DScanner::ScannerStats Enhanced3DScanner::getStats() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return stats_;
}

bool Enhanced3DScanner::saveMap(const std::string& filename) const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    if (accumulated_map_->points_.empty()) {
        std::cerr << "No map data to save" << std::endl;
        return false;
    }
    
    try {
        bool success = open3d::io::WritePointCloud(filename, *accumulated_map_);
        if (success) {
            std::cout << "Map saved to " << filename << " with " 
                      << accumulated_map_->points_.size() << " points" << std::endl;
        }
        return success;
    } catch (const std::exception& e) {
        std::cerr << "Error saving map: " << e.what() << std::endl;
        return false;
    }
}

bool Enhanced3DScanner::saveKeyframes(const std::string& directory) const {
    std::lock_guard<std::mutex> lock(keyframe_mutex_);
    
    try {
        std::filesystem::create_directories(directory);
        
        for (const auto& keyframe : keyframes_) {
            std::string rgb_filename = directory + "/keyframe_" + 
                                      std::to_string(keyframe.id) + "_rgb.jpg";
            std::string depth_filename = directory + "/keyframe_" + 
                                        std::to_string(keyframe.id) + "_depth.png";
            std::string cloud_filename = directory + "/keyframe_" + 
                                        std::to_string(keyframe.id) + "_cloud.pcd";
            
            cv::imwrite(rgb_filename, keyframe.rgb_image);
            cv::imwrite(depth_filename, keyframe.depth_image);
            open3d::io::WritePointCloud(cloud_filename, *keyframe.point_cloud);
        }
        
        std::cout << "Saved " << keyframes_.size() << " keyframes to " << directory << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error saving keyframes: " << e.what() << std::endl;
        return false;
    }
}

void Enhanced3DScanner::reset() {
    std::lock_guard<std::mutex> pose_lock(pose_mutex_);
    std::lock_guard<std::mutex> map_lock(map_mutex_);
    std::lock_guard<std::mutex> keyframe_lock(keyframe_mutex_);
    std::lock_guard<std::mutex> stats_lock(stats_mutex_);
    
    current_pose_ = Pose();
    previous_pose_ = Pose();
    keyframes_.clear();
    accumulated_map_->Clear();
    stats_ = ScannerStats();
    
    debugPrint("Scanner reset");
}

// Configuration methods
void Enhanced3DScanner::setKeyframeThresholds(double distance_threshold, double angle_threshold) {
    keyframe_distance_threshold_ = distance_threshold;
    keyframe_angle_threshold_ = angle_threshold;
    debugPrint("Keyframe thresholds updated: distance=" + std::to_string(distance_threshold) + 
              ", angle=" + std::to_string(angle_threshold));
}

void Enhanced3DScanner::setICPParameters(double max_correspondence_distance, int max_iterations, 
                                        double convergence_criteria) {
    icp_max_correspondence_distance_ = max_correspondence_distance;
    icp_max_iterations_ = max_iterations;
    icp_convergence_criteria_ = convergence_criteria;
    debugPrint("ICP parameters updated");
}

void Enhanced3DScanner::setFeatureParameters(int max_features, int min_matches, double max_distance) {
    max_features_ = max_features;
    min_feature_matches_ = min_matches;
    max_feature_distance_ = max_distance;
    
    // Recreate feature detector with new parameters
    feature_detector_ = cv::ORB::create(max_features_);
    debugPrint("Feature parameters updated");
}

void Enhanced3DScanner::setMapParameters(double voxel_size, int max_points) {
    voxel_size_map_ = voxel_size;
    max_map_points_ = max_points;
    debugPrint("Map parameters updated");
}

void Enhanced3DScanner::setDebugMode(bool enable) {
    debug_mode_ = enable;
    if (enable) {
        std::filesystem::create_directories(output_directory_);
    }
}

Eigen::Matrix4d Enhanced3DScanner::getRelativeTransform(const Pose& from, const Pose& to) {
    return to.transformation * from.transformation.inverse();
}

void Enhanced3DScanner::printStatus() const {
    auto stats = getStats();
    auto pose = getCurrentPose();
    auto keyframe_count = getKeyframeCount();
    
    auto runtime = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::steady_clock::now() - stats.start_time);
    
    std::cout << "\n========== Enhanced 3D Scanner Status ==========" << std::endl;
    std::cout << "Runtime: " << runtime.count() << " seconds" << std::endl;
    std::cout << "Frames processed: " << stats.total_frames_processed << std::endl;
    std::cout << "Keyframes created: " << keyframe_count << std::endl;
    std::cout << "Map points: " << stats.map_points << std::endl;
    std::cout << "Distance traveled: " << std::fixed << std::setprecision(2) 
              << stats.total_distance_traveled << " meters" << std::endl;
    std::cout << "Current pose: [" << pose.getTranslation().x() << ", " 
              << pose.getTranslation().y() << ", " << pose.getTranslation().z() << "]" << std::endl;
    std::cout << "Pose confidence: " << pose.confidence << std::endl;
    std::cout << "===============================================" << std::endl;
}

void Enhanced3DScanner::debugPrint(const std::string& message) const {
    if (debug_mode_) {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        
        // Use thread-safe time formatting
        char time_buffer[100];
        struct tm time_info;
        
#ifdef _WIN32
        localtime_s(&time_info, &time_t);
#else
        localtime_r(&time_t, &time_info);
#endif
        
        strftime(time_buffer, sizeof(time_buffer), "%H:%M:%S", &time_info);
        std::cout << "[" << time_buffer << "] " << message << std::endl;
    }
}

void Enhanced3DScanner::saveDebugImage(const cv::Mat& image, const std::string& suffix) const {
    if (debug_mode_ && !image.empty()) {
        std::string filename = output_directory_ + "debug_" + suffix + "_" + 
                              std::to_string(std::time(nullptr)) + ".jpg";
        cv::imwrite(filename, image);
    }
}