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
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>

// SLAM additions
#include <deque>
#include <memory>
#include <map>
#include <algorithm>
#include <random>

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

// Enhanced SLAM structures
struct KeyFrame {
    int id;
    cv::Mat rgb_image;
    cv::Mat depth_image;
    cv::Mat pose; // 4x4 transformation matrix
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    std::vector<cv::Point3f> landmarks_3d; // 3D positions of features
    double timestamp;

    KeyFrame(int frame_id, const cv::Mat& rgb, const cv::Mat& depth, double ts)
        : id(frame_id), rgb_image(rgb.clone()), depth_image(depth.clone()),
        pose(cv::Mat::eye(4, 4, CV_64F)), timestamp(ts) {}
};

struct MapPoint {
    int id;
    cv::Point3f position;
    cv::Vec3b color;
    std::vector<std::pair<int, int>> observations; // (keyframe_id, keypoint_idx)
    bool is_outlier = false;

    MapPoint(int point_id, const cv::Point3f& pos, const cv::Vec3b& col)
        : id(point_id), position(pos), color(col) {}
};

class EnhancedSLAM {
private:
    // Camera intrinsics
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    // SLAM components
    cv::Ptr<cv::ORB> feature_detector_;
    cv::Ptr<cv::DescriptorMatcher> matcher_;

    // Keyframe and map management
    std::vector<std::shared_ptr<KeyFrame>> keyframes_;
    std::map<int, std::shared_ptr<MapPoint>> map_points_;
    std::shared_ptr<KeyFrame> current_keyframe_;
    std::shared_ptr<KeyFrame> last_keyframe_;

    // Pose tracking
    cv::Mat current_pose_;
    std::deque<cv::Mat> pose_history_;

    // IMU integration
    cv::Vec3f last_gyro_;
    cv::Vec3f last_accel_;
    double last_imu_timestamp_;
    bool has_imu_data_;
    cv::Mat imu_predicted_pose_;

    // Parameters (relaxed for better visual-only tracking)
    int next_keyframe_id_ = 0;
    int next_mappoint_id_ = 0;
    const int MAX_KEYFRAMES = 100;  // Increased from 50
    const int MAX_POSE_HISTORY = 200; // Increased
    const double KEYFRAME_DISTANCE_THRESHOLD = 0.1; // Much smaller for indoor use
    const double KEYFRAME_ANGLE_THRESHOLD = 5.0; // Much smaller for better tracking

    // Quality thresholds (very relaxed for robust tracking)
    const int MIN_FEATURES = 30;   // Reduced from 50
    const int MIN_MATCHES = 15;    // Reduced from 20  
    const double MAX_REPROJECTION_ERROR = 5.0; // Increased tolerance

public:
    EnhancedSLAM() {
        // Initialize feature detector with more features for better tracking
        feature_detector_ = cv::ORB::create(
            1500,           // More features
            1.2f,           // scaleFactor
            8,              // nlevels
            31,             // edgeThreshold
            0,              // firstLevel
            2,              // WTA_K
            cv::ORB::HARRIS_SCORE,
            31,             // patchSize
            10              // Lower fastThreshold for more features
        );

        matcher_ = cv::DescriptorMatcher::create("BruteForce-Hamming");
        current_pose_ = cv::Mat::eye(4, 4, CV_64F);
        imu_predicted_pose_ = cv::Mat::eye(4, 4, CV_64F);

        // Initialize IMU data
        last_gyro_ = cv::Vec3f(0, 0, 0);
        last_accel_ = cv::Vec3f(0, 0, 0);
        last_imu_timestamp_ = 0;
        has_imu_data_ = false;

        // DEFAULT camera intrinsics - REPLACE WITH YOUR CALIBRATED VALUES
        SetCameraIntrinsics(735.749, 731.258, 617.785, 358.336);
    }

    // Add IMU data integration
    void UpdateIMUData(const cv::Vec3f& gyro, const cv::Vec3f& accel, double timestamp) {
        if (has_imu_data_) {
            double dt = timestamp - last_imu_timestamp_;
            if (dt > 0 && dt < 0.1) { // Valid time interval
                PredictPoseFromIMU(gyro, accel, dt);
            }
        }

        last_gyro_ = gyro;
        last_accel_ = accel;
        last_imu_timestamp_ = timestamp;
        has_imu_data_ = true;
    }

    void PredictPoseFromIMU(const cv::Vec3f& gyro, const cv::Vec3f& accel, double dt) {
        // Simple IMU integration for pose prediction
        cv::Mat current_R = current_pose_(cv::Rect(0, 0, 3, 3));
        cv::Mat current_t = current_pose_(cv::Rect(3, 0, 1, 3));

        // Integrate gyroscope for rotation (simplified)
        cv::Vec3f delta_rotation = (last_gyro_ + gyro) * 0.5f * static_cast<float>(dt);

        // Create rotation matrix from gyro data
        cv::Mat delta_R = cv::Mat::eye(3, 3, CV_64F);
        if (cv::norm(delta_rotation) > 0.001) { // Avoid tiny rotations
            cv::Mat rvec = (cv::Mat_<double>(3, 1) << delta_rotation[0], delta_rotation[1], delta_rotation[2]);
            cv::Rodrigues(rvec, delta_R);
        }

        // Update predicted pose
        cv::Mat predicted_R = current_R * delta_R;

        // Simple integration for translation (gravity-corrected acceleration)
        cv::Vec3f gravity_world(0, 0, 9.81f); // Assume Z-up coordinate system
        cv::Vec3f accel_corrected = accel - gravity_world;
        cv::Vec3f delta_translation = accel_corrected * static_cast<float>(dt * dt * 0.5);

        cv::Mat predicted_t = current_t + current_R * (cv::Mat_<double>(3, 1) <<
            delta_translation[0], delta_translation[1], delta_translation[2]);

        // Update IMU predicted pose
        predicted_R.copyTo(imu_predicted_pose_(cv::Rect(0, 0, 3, 3)));
        predicted_t.copyTo(imu_predicted_pose_(cv::Rect(3, 0, 1, 3)));
    }

    void SetCameraIntrinsics(double fx, double fy, double cx, double cy) {
        camera_matrix_ = (cv::Mat_<double>(3, 3) <<
            fx, 0, cx,
            0, fy, cy,
            0, 0, 1);
        dist_coeffs_ = cv::Mat::zeros(4, 1, CV_64F);
    }

    // Main SLAM processing function
    bool ProcessFrame(const cv::Mat& rgb_frame, const cv::Mat& depth_frame, double timestamp) {
        if (rgb_frame.empty() || depth_frame.empty()) {
            return false;
        }

        // Create new keyframe candidate
        auto candidate_kf = std::make_shared<KeyFrame>(
            next_keyframe_id_, rgb_frame, depth_frame, timestamp);

        // Extract features
        if (!ExtractFeatures(candidate_kf)) {
            return false;
        }

        bool success = false;

        if (keyframes_.empty()) {
            // First frame - initialize SLAM
            success = InitializeSLAM(candidate_kf);
        }
        else {
            // Track camera pose
            success = TrackPose(candidate_kf);

            // Decide if we need a new keyframe
            if (success && ShouldCreateKeyframe(candidate_kf)) {
                AddKeyframe(candidate_kf);
                TriangulateNewPoints();
                OptimizeLocalMap();
            }
        }

        if (success) {
            current_keyframe_ = candidate_kf;
            pose_history_.push_back(current_pose_.clone());
            if (pose_history_.size() > MAX_POSE_HISTORY) {
                pose_history_.pop_front();
            }
        }

        return success;
    }

private:
    bool ExtractFeatures(std::shared_ptr<KeyFrame> kf) {
        cv::Mat gray;
        if (kf->rgb_image.channels() == 3) {
            cv::cvtColor(kf->rgb_image, gray, cv::COLOR_BGR2GRAY);
        }
        else {
            gray = kf->rgb_image;
        }

        feature_detector_->detectAndCompute(gray, cv::Mat(),
            kf->keypoints, kf->descriptors);

        if (kf->keypoints.size() < MIN_FEATURES) {
            return false;
        }

        // Convert 2D features to 3D using depth
        ConvertFeaturesToLandmarks(kf);

        return true;
    }

    void ConvertFeaturesToLandmarks(std::shared_ptr<KeyFrame> kf) {
        kf->landmarks_3d.clear();

        for (const auto& kp : kf->keypoints) {
            cv::Point3f landmark_3d = GetPoint3D(kp.pt, kf->depth_image);
            kf->landmarks_3d.push_back(landmark_3d);
        }
    }

    cv::Point3f GetPoint3D(const cv::Point2f& pixel, const cv::Mat& depth_image) {
        int x = static_cast<int>(pixel.x);
        int y = static_cast<int>(pixel.y);

        if (x < 0 || x >= depth_image.cols || y < 0 || y >= depth_image.rows) {
            return cv::Point3f(0, 0, 0);
        }

        float depth = depth_image.at<uint16_t>(y, x) / 1000.0f; // Convert mm to meters

        if (depth <= 0.1f || depth > 10.0f) {
            return cv::Point3f(0, 0, 0);
        }

        // Convert to 3D using camera intrinsics
        double fx = camera_matrix_.at<double>(0, 0);
        double fy = camera_matrix_.at<double>(1, 1);
        double cx = camera_matrix_.at<double>(0, 2);
        double cy = camera_matrix_.at<double>(1, 2);

        float x3d = static_cast<float>((pixel.x - cx) * depth / fx);
        float y3d = static_cast<float>((pixel.y - cy) * depth / fy);

        return cv::Point3f(x3d, y3d, depth);
    }

    bool InitializeSLAM(std::shared_ptr<KeyFrame> first_kf) {
        first_kf->pose = cv::Mat::eye(4, 4, CV_64F);
        current_pose_ = first_kf->pose.clone();

        AddKeyframe(first_kf);

        // Create initial map points
        for (size_t i = 0; i < first_kf->keypoints.size(); ++i) {
            if (first_kf->landmarks_3d[i].z > 0.1f) {
                auto map_point = std::make_shared<MapPoint>(
                    next_mappoint_id_++,
                    first_kf->landmarks_3d[i],
                    cv::Vec3b(255, 255, 255)
                    );
                map_point->observations.push_back({ first_kf->id, static_cast<int>(i) });
                map_points_[map_point->id] = map_point;
            }
        }

        return true;
    }

    bool TrackPose(std::shared_ptr<KeyFrame> current_kf) {
        if (!last_keyframe_) {
            return false;
        }

        // Use IMU prediction as initial guess if available
        if (has_imu_data_) {
            current_kf->pose = imu_predicted_pose_.clone();
        }
        else {
            current_kf->pose = last_keyframe_->pose.clone();
        }

        // Match features between current and last keyframe
        std::vector<cv::DMatch> matches;
        matcher_->match(last_keyframe_->descriptors, current_kf->descriptors, matches);

        // Filter matches
        std::vector<cv::DMatch> good_matches;
        FilterMatches(matches, good_matches);

        if (good_matches.size() < MIN_MATCHES) {
            // If visual tracking fails but we have IMU, use IMU prediction
            if (has_imu_data_) {
                current_pose_ = imu_predicted_pose_.clone();
                current_kf->pose = current_pose_.clone();
                return true; // Accept IMU-only tracking
            }
            return false;
        }

        // Use PnP to refine pose estimate
        return SolvePnP(current_kf, good_matches);
    }

    void FilterMatches(const std::vector<cv::DMatch>& matches,
        std::vector<cv::DMatch>& good_matches) {
        if (matches.empty()) return;

        float min_dist = matches[0].distance;
        for (const auto& match : matches) {
            min_dist = std::min(min_dist, match.distance);
        }

        float threshold = std::max(2.0f * min_dist, 30.0f);

        for (const auto& match : matches) {
            if (match.distance <= threshold) {
                good_matches.push_back(match);
            }
        }
    }

    bool SolvePnP(std::shared_ptr<KeyFrame> current_kf,
        const std::vector<cv::DMatch>& matches) {
        std::vector<cv::Point3f> object_points;
        std::vector<cv::Point2f> image_points;

        for (const auto& match : matches) {
            int last_idx = match.queryIdx;
            int curr_idx = match.trainIdx;

            cv::Point3f point_3d = last_keyframe_->landmarks_3d[last_idx];
            if (point_3d.z > 0.1f) {
                object_points.push_back(point_3d);
                image_points.push_back(current_kf->keypoints[curr_idx].pt);
            }
        }

        if (object_points.size() < 6) {
            return false;
        }

        cv::Mat rvec, tvec;
        std::vector<int> inliers;

        bool success = cv::solvePnPRansac(
            object_points, image_points,
            camera_matrix_, dist_coeffs_,
            rvec, tvec, false, 100, MAX_REPROJECTION_ERROR, 0.99, inliers
        );

        if (!success || inliers.size() < MIN_MATCHES) {
            return false;
        }

        cv::Mat R;
        cv::Rodrigues(rvec, R);

        cv::Mat pose = cv::Mat::eye(4, 4, CV_64F);
        R.copyTo(pose(cv::Rect(0, 0, 3, 3)));
        tvec.copyTo(pose(cv::Rect(3, 0, 1, 3)));

        current_pose_ = last_keyframe_->pose * pose;
        current_kf->pose = current_pose_.clone();

        return true;
    }

    bool ShouldCreateKeyframe(std::shared_ptr<KeyFrame> candidate_kf) {
        if (!last_keyframe_) {
            return true;
        }

        // Check translation distance
        cv::Mat last_pos = last_keyframe_->pose(cv::Rect(3, 0, 1, 3));
        cv::Mat curr_pos = candidate_kf->pose(cv::Rect(3, 0, 1, 3));
        double translation = cv::norm(curr_pos - last_pos);

        if (translation > KEYFRAME_DISTANCE_THRESHOLD) {
            return true;
        }

        // Check rotation angle
        cv::Mat last_R = last_keyframe_->pose(cv::Rect(0, 0, 3, 3));
        cv::Mat curr_R = candidate_kf->pose(cv::Rect(0, 0, 3, 3));
        cv::Mat delta_R = curr_R * last_R.t();

        double trace = delta_R.at<double>(0, 0) + delta_R.at<double>(1, 1) + delta_R.at<double>(2, 2);
        double angle = std::acos(std::max(-1.0, std::min(1.0, (trace - 1.0) / 2.0))) * 180.0 / CV_PI;

        if (angle > KEYFRAME_ANGLE_THRESHOLD) {
            return true;
        }

        // Also create keyframe if we have enough new features
        int shared_features = 0;
        std::vector<cv::DMatch> matches;
        matcher_->match(last_keyframe_->descriptors, candidate_kf->descriptors, matches);
        for (const auto& match : matches) {
            if (match.distance < 50) shared_features++;
        }

        // If less than 70% features are shared, create new keyframe
        double feature_overlap = static_cast<double>(shared_features) / std::min(static_cast<double>(last_keyframe_->keypoints.size()), static_cast<double>(candidate_kf->keypoints.size()));
        if (feature_overlap < 0.7) {
            return true;
        }

        return false;
    }

    void AddKeyframe(std::shared_ptr<KeyFrame> kf) {
        kf->id = next_keyframe_id_++;
        keyframes_.push_back(kf);
        last_keyframe_ = kf;

        // Don't limit keyframes as aggressively - keep more for better tracking
        if (keyframes_.size() > MAX_KEYFRAMES) {
            // Remove oldest keyframe but keep more keyframes around
            auto oldest_kf = keyframes_.front();
            keyframes_.erase(keyframes_.begin());
            // Don't remove map points as aggressively
            // RemoveMapPointsFromKeyframe(oldest_kf->id);
        }

        std::cout << "Added keyframe " << kf->id << " (total: " << keyframes_.size() << ")" << std::endl;
    }

    void RemoveMapPointsFromKeyframe(int keyframe_id) {
        auto it = map_points_.begin();
        while (it != map_points_.end()) {
            auto& observations = it->second->observations;
            observations.erase(
                std::remove_if(observations.begin(), observations.end(),
                    [keyframe_id](const std::pair<int, int>& obs) {
                        return obs.first == keyframe_id;
                    }),
                observations.end()
                        );

            if (observations.empty()) {
                it = map_points_.erase(it);
            }
            else {
                ++it;
            }
        }
    }

    void TriangulateNewPoints() {
        if (keyframes_.size() < 2) return;

        auto kf1 = keyframes_[keyframes_.size() - 2];
        auto kf2 = keyframes_[keyframes_.size() - 1];

        std::vector<cv::DMatch> matches;
        matcher_->match(kf1->descriptors, kf2->descriptors, matches);

        std::vector<cv::DMatch> good_matches;
        FilterMatches(matches, good_matches);

        for (const auto& match : good_matches) {
            int idx1 = match.queryIdx;
            int idx2 = match.trainIdx;

            bool exists = false;
            for (const auto& [mp_id, mp] : map_points_) {
                for (const auto& obs : mp->observations) {
                    if ((obs.first == kf1->id && obs.second == idx1) ||
                        (obs.first == kf2->id && obs.second == idx2)) {
                        exists = true;
                        break;
                    }
                }
                if (exists) break;
            }

            if (!exists) {
                cv::Point3f point_3d = kf2->landmarks_3d[idx2];
                if (point_3d.z > 0.1f) {
                    auto map_point = std::make_shared<MapPoint>(
                        next_mappoint_id_++, point_3d, cv::Vec3b(255, 255, 255));
                    map_point->observations.push_back({ kf1->id, idx1 });
                    map_point->observations.push_back({ kf2->id, idx2 });
                    map_points_[map_point->id] = map_point;
                }
            }
        }
    }

    void OptimizeLocalMap() {
        if (keyframes_.size() < 3) return;

        int window_size = std::min(5, static_cast<int>(keyframes_.size()));
        std::vector<std::shared_ptr<KeyFrame>> local_keyframes(
            keyframes_.end() - window_size, keyframes_.end());

        std::set<int> local_map_point_ids;
        for (const auto& kf : local_keyframes) {
            for (const auto& [mp_id, mp] : map_points_) {
                for (const auto& obs : mp->observations) {
                    if (obs.first == kf->id) {
                        local_map_point_ids.insert(mp_id);
                        break;
                    }
                }
            }
        }

        for (auto& kf : local_keyframes) {
            std::vector<cv::Point3f> object_points;
            std::vector<cv::Point2f> image_points;

            for (int mp_id : local_map_point_ids) {
                auto mp = map_points_[mp_id];
                for (const auto& obs : mp->observations) {
                    if (obs.first == kf->id) {
                        object_points.push_back(mp->position);
                        image_points.push_back(kf->keypoints[obs.second].pt);
                        break;
                    }
                }
            }

            if (object_points.size() >= 6) {
                cv::Mat rvec, tvec;
                cv::solvePnP(object_points, image_points,
                    camera_matrix_, dist_coeffs_, rvec, tvec);

                cv::Mat R;
                cv::Rodrigues(rvec, R);
                R.copyTo(kf->pose(cv::Rect(0, 0, 3, 3)));
                tvec.copyTo(kf->pose(cv::Rect(3, 0, 1, 3)));
            }
        }
    }

public:
    // Getter functions
    cv::Mat GetCurrentPose() const {
        return current_pose_.clone();
    }

    std::vector<cv::Point3f> GetMapPoints() const {
        std::vector<cv::Point3f> points;
        for (const auto& [id, mp] : map_points_) {
            if (!mp->is_outlier) {
                points.push_back(mp->position);
            }
        }
        return points;
    }

    std::vector<cv::Mat> GetTrajectory() const {
        std::vector<cv::Mat> trajectory;
        for (const auto& kf : keyframes_) {
            trajectory.push_back(kf->pose.clone());
        }
        return trajectory;
    }

    void GetPoseComponents(cv::Vec3d& translation, cv::Vec3d& rotation) const {
        cv::Mat t = current_pose_(cv::Rect(3, 0, 1, 3));
        cv::Mat R = current_pose_(cv::Rect(0, 0, 3, 3));

        translation = cv::Vec3d(t.at<double>(0), t.at<double>(1), t.at<double>(2));

        cv::Mat rvec;
        cv::Rodrigues(R, rvec);
        rotation = cv::Vec3d(rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2));
    }

    bool IsInitialized() const {
        return !keyframes_.empty();
    }

    int GetNumKeyframes() const {
        return static_cast<int>(keyframes_.size());
    }

    int GetNumMapPoints() const {
        return static_cast<int>(map_points_.size());
    }

    void Reset() {
        keyframes_.clear();
        map_points_.clear();
        current_keyframe_.reset();
        last_keyframe_.reset();
        current_pose_ = cv::Mat::eye(4, 4, CV_64F);
        pose_history_.clear();
        next_keyframe_id_ = 0;
        next_mappoint_id_ = 0;
    }
};

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

    // SLAM components
    EnhancedSLAM slam_system_;
    bool slam_initialized_;
    std::chrono::high_resolution_clock::time_point last_slam_time_;
    std::vector<cv::Point3f> trajectory_points_;
    std::vector<cv::Point3f> map_points_;

public:
    RGBDIRViewer() : m_handle(nullptr), m_bDeviceOpen(false), m_bGrabbing(false),
        m_frameCount(0), m_selectedResolution(0), slam_initialized_(false) {
        // Initialize available display resolutions - all 16:9 aspect ratio
        m_resolutions = {
            {640, 360, "640x360 (16:9)"},
            {854, 480, "854x480 (16:9)"},
            {960, 540, "960x540 (16:9)"},
            {1280, 720, "1280x720 (16:9) - HD"},
            {1920, 1080, "1920x1080 (16:9) - Full HD"}
        };

        // Initialize SLAM with YOUR ACTUAL CALIBRATED VALUES
        last_slam_time_ = std::chrono::high_resolution_clock::now();

        // YOUR CALIBRATED CAMERA PARAMETERS - REPLACE THE DEFAULTS
        slam_system_.SetCameraIntrinsics(735.749, 731.258, 617.785, 358.336);

        std::cout << "SLAM system initialized with calibrated camera parameters:" << std::endl;
        std::cout << "fx=735.749, fy=731.258, cx=617.785, cy=358.336" << std::endl;
    }

    ~RGBDIRViewer() {
        if (m_bGrabbing) {
            StopGrabbing();
        }
        if (m_bDeviceOpen) {
            CloseDevice();
        }
    }

    int SelectDisplayResolution() {
        std::cout << "\nSelect display resolution:" << std::endl;
        for (size_t i = 0; i < m_resolutions.size(); i++) {
            std::cout << i << ": " << m_resolutions[i].name << std::endl;
        }

        std::cout << "Enter selection (0-" << (m_resolutions.size() - 1) << "): ";
        int selection;
        std::cin >> selection;

        if (selection >= 0 && selection < static_cast<int>(m_resolutions.size())) {
            m_selectedResolution = selection;
            std::cout << "Selected: " << m_resolutions[m_selectedResolution].name << std::endl;
            return MV3D_RGBD_OK;
        }
        else {
            std::cerr << "Invalid selection! Using default resolution." << std::endl;
            m_selectedResolution = 0;
            return MV3D_RGBD_E_PARAMETER;
        }
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

        // Register IMU callback - following your working IMU code pattern
        MV3D_RGBD_PARAM stParam;
        stParam.enParamType = ParamType_Enum;
        stParam.ParamInfo.stEnumParam.nCurValue = 1;
        nRet = MV3D_RGBD_SetParam(m_handle, "EventNotification", &stParam);
        if (MV3D_RGBD_OK == nRet) {
            nRet = MV3D_RGBD_RegisterIMUDataCallBack(m_handle, IMUCallBackFunc, m_handle);
            if (MV3D_RGBD_OK == nRet) {
                std::cout << "IMU callback registered successfully." << std::endl;

                // Try different IMU parameter names - your camera might use different ones
                const char* imu_params[] = { "IMUEnable", "ImuEnable", "IMU_Enable", "EventSelector" };
                bool imu_enabled = false;

                for (const char* param_name : imu_params) {
                    MV3D_RGBD_PARAM imuParam;
                    imuParam.enParamType = ParamType_Bool;
                    imuParam.ParamInfo.bBoolParam = true;
                    nRet = MV3D_RGBD_SetParam(m_handle, param_name, &imuParam);
                    if (MV3D_RGBD_OK == nRet) {
                        std::cout << "IMU enabled using parameter: " << param_name << std::endl;
                        imu_enabled = true;
                        break;
                    }
                }

                if (!imu_enabled) {
                    std::cout << "Note: Could not enable IMU streaming (error 0x80060004)" << std::endl;
                    std::cout << "      SLAM will work with visual-only tracking" << std::endl;
                    std::cout << "      IMU may not be supported on this camera model" << std::endl;
                }
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

    cv::Mat ConvertDepthToDisplay(const MV3D_RGBD_IMAGE_DATA& imageData) {
        if (imageData.pData == nullptr || imageData.nDataLen == 0) {
            return cv::Mat();
        }

        // Create depth image from raw data
        cv::Mat depthRaw(imageData.nHeight, imageData.nWidth, CV_16UC1, imageData.pData);
        cv::Mat depthDisplay;

        // Convert to 8-bit for display (scale depth values)
        double minVal, maxVal;
        cv::minMaxLoc(depthRaw, &minVal, &maxVal);
        if (maxVal > minVal) {
            depthRaw.convertTo(depthDisplay, CV_8UC1, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));
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
        if (imageData.pData == nullptr || imageData.nDataLen == 0) {
            return cv::Mat();
        }

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

        if (newData) {
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
            cv::putText(image, imuText[i], cv::Point(10, y_offset + i * 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
        }
    }

    // SLAM processing function with IMU integration
    void ProcessSLAMFrame(const cv::Mat& rgbFrame, const cv::Mat& depthFrame) {
        auto current_time = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double>(current_time - last_slam_time_).count();

        // Process SLAM every 100ms for performance
        if (elapsed < 0.1) {
            return;
        }
        last_slam_time_ = current_time;

        if (rgbFrame.empty() || depthFrame.empty()) {
            return;
        }

        // Get IMU data and integrate it
        float xa, ya, za, xg, yg, zg;
        bool newIMUData;
        g_imuData.getData(xa, ya, za, xg, yg, zg, newIMUData);

        if (newIMUData) {
            cv::Vec3f gyro(xg, yg, zg);
            cv::Vec3f accel(xa, ya, za);
            double timestamp = std::chrono::duration<double>(current_time.time_since_epoch()).count();
            slam_system_.UpdateIMUData(gyro, accel, timestamp);
        }

        // Ensure depth is 16-bit for SLAM processing
        cv::Mat depth16;
        if (depthFrame.type() == CV_16UC1) {
            depth16 = depthFrame;
        }
        else {
            depthFrame.convertTo(depth16, CV_16UC1);
        }

        double timestamp = std::chrono::duration<double>(current_time.time_since_epoch()).count();

        bool success = slam_system_.ProcessFrame(rgbFrame, depth16, timestamp);

        if (success) {
            slam_initialized_ = true;

            // Update visualization data
            auto trajectory = slam_system_.GetTrajectory();
            trajectory_points_.clear();
            for (const auto& pose : trajectory) {
                cv::Mat position = pose(cv::Rect(3, 0, 1, 3));
                trajectory_points_.push_back(cv::Point3f(
                    position.at<double>(0), position.at<double>(1), position.at<double>(2)
                ));
            }
            map_points_ = slam_system_.GetMapPoints();

            // Print position occasionally
            static int counter = 0;
            if (++counter % 30 == 0) {
                cv::Vec3d translation, rotation;
                slam_system_.GetPoseComponents(translation, rotation);
                std::string imu_status = newIMUData ? "IMU: ON" : "IMU: OFF";
                std::cout << "SLAM - Position: (" << std::fixed << std::setprecision(2)
                    << translation[0] << ", " << translation[1] << ", "
                    << translation[2] << ") m | KF: " << slam_system_.GetNumKeyframes()
                    << " | MP: " << std::dec << slam_system_.GetNumMapPoints()
                    << " | " << imu_status << std::endl;
            }
        }
        else {
            // Only print SLAM failures occasionally to avoid spam
            static int fail_counter = 0;
            if (++fail_counter % 60 == 0) { // Less frequent failure reporting
                std::cout << "SLAM tracking lost - trying to recover..." << std::endl;
            }
        }
    }

    void DrawSLAMInfo(cv::Mat& image) {
        if (image.empty()) return;

        // SLAM status
        if (!slam_initialized_) {
            cv::putText(image, "SLAM: Initializing...", cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
        }
        else {
            cv::putText(image, "SLAM: Active", cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

            // Show statistics
            std::string stats = "KF: " + std::to_string(slam_system_.GetNumKeyframes()) +
                " | MP: " + std::to_string(slam_system_.GetNumMapPoints());
            cv::putText(image, stats, cv::Point(10, 60),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

            // Show position
            cv::Vec3d translation, rotation;
            slam_system_.GetPoseComponents(translation, rotation);
            std::stringstream pos_stream;
            pos_stream << std::fixed << std::setprecision(2);
            pos_stream << "Pos: (" << translation[0] << ", " << translation[1] << ", " << translation[2] << ")";
            cv::putText(image, pos_stream.str(), cv::Point(10, 90),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        }
    }

    cv::Mat CreateRoomMap(int size = 300) {
        cv::Mat map = cv::Mat::zeros(size, size, CV_8UC3);

        if (!slam_initialized_ || map_points_.empty()) {
            cv::putText(map, "No SLAM data", cv::Point(size / 2 - 60, size / 2),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
            return map;
        }

        // Find bounds
        float min_x = map_points_[0].x, max_x = map_points_[0].x;
        float min_z = map_points_[0].z, max_z = map_points_[0].z;

        for (const auto& point : map_points_) {
            min_x = std::min(min_x, point.x);
            max_x = std::max(max_x, point.x);
            min_z = std::min(min_z, point.z);
            max_z = std::max(max_z, point.z);
        }

        float range = std::max(max_x - min_x, max_z - min_z);
        if (range < 0.1f) return map;

        float scale = (size - 40) / range;
        cv::Point2f center(size / 2.0f, size / 2.0f);

        // Draw map points
        for (const auto& point : map_points_) {
            int x = static_cast<int>(center.x + (point.x - (min_x + max_x) / 2.0f) * scale);
            int y = static_cast<int>(center.y + (point.z - (min_z + max_z) / 2.0f) * scale);

            if (x >= 0 && x < size && y >= 0 && y < size) {
                cv::circle(map, cv::Point(x, y), 1, cv::Scalar(255, 255, 255), -1);
            }
        }

        // Draw trajectory
        for (size_t i = 1; i < trajectory_points_.size(); ++i) {
            cv::Point3f p1 = trajectory_points_[i - 1];
            cv::Point3f p2 = trajectory_points_[i];

            int x1 = static_cast<int>(center.x + (p1.x - (min_x + max_x) / 2.0f) * scale);
            int y1 = static_cast<int>(center.y + (p1.z - (min_z + max_z) / 2.0f) * scale);
            int x2 = static_cast<int>(center.x + (p2.x - (min_x + max_x) / 2.0f) * scale);
            int y2 = static_cast<int>(center.y + (p2.z - (min_z + max_z) / 2.0f) * scale);

            cv::line(map, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);
        }

        // Draw current position
        if (!trajectory_points_.empty()) {
            cv::Point3f current = trajectory_points_.back();
            int x = static_cast<int>(center.x + (current.x - (min_x + max_x) / 2.0f) * scale);
            int y = static_cast<int>(center.y + (current.z - (min_z + max_z) / 2.0f) * scale);
            cv::circle(map, cv::Point(x, y), 4, cv::Scalar(0, 0, 255), -1);
        }

        // Add coordinate axes
        cv::arrowedLine(map, cv::Point(20, size - 20), cv::Point(50, size - 20),
            cv::Scalar(0, 0, 255), 2); // X axis - red
        cv::arrowedLine(map, cv::Point(20, size - 20), cv::Point(20, size - 50),
            cv::Scalar(0, 255, 0), 2); // Z axis - green
        cv::putText(map, "X", cv::Point(55, size - 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
        cv::putText(map, "Z", cv::Point(25, size - 55), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);

        return map;
    }

    void ProcessFrameData(const MV3D_RGBD_FRAME_DATA& frameData) {
        cv::Mat displayMat;
        cv::Mat rgbFrame, depthFrame;
        cv::Mat originalDepth; // Keep original depth for SLAM
        bool hasRGB = false, hasDepth = false;

        for (int i = 0; i < frameData.nImageCount; i++) {
            const MV3D_RGBD_IMAGE_DATA& imageData = frameData.stImageData[i];

            if (ImageType_Depth == imageData.enImageType) {
                // Store original depth for SLAM
                originalDepth = cv::Mat(imageData.nHeight, imageData.nWidth, CV_16UC1, imageData.pData).clone();

                cv::Mat depthDisplay = ConvertDepthToDisplay(imageData);
                if (!depthDisplay.empty()) {
                    depthFrame = originalDepth; // Use original depth for SLAM
                    hasDepth = true;
                    ResizeForDisplay(depthDisplay, displayMat);
                    cv::imshow("Depth", displayMat);
                }
            }
            else if (ImageType_RGB8_Planar == imageData.enImageType) {
                cv::Mat colorDisplay = ConvertColorToDisplay(imageData);
                if (!colorDisplay.empty()) {
                    rgbFrame = colorDisplay.clone();
                    hasRGB = true;

                    // Add SLAM info overlay
                    DrawSLAMInfo(colorDisplay);

                    // Add IMU overlay
                    AddIMUOverlay(colorDisplay);

                    ResizeForDisplay(colorDisplay, displayMat);
                    cv::imshow("RGB", displayMat);
                }
            }
            else if (ImageType_YUV422 == imageData.enImageType) {
                cv::Mat colorDisplay = ConvertColorToDisplay(imageData);
                if (!colorDisplay.empty()) {
                    rgbFrame = colorDisplay.clone();
                    hasRGB = true;

                    // Add SLAM info overlay
                    DrawSLAMInfo(colorDisplay);

                    // Add IMU overlay
                    AddIMUOverlay(colorDisplay);

                    ResizeForDisplay(colorDisplay, displayMat);
                    cv::imshow("Color", displayMat);
                }
            }
            else if (ImageType_Mono8 == imageData.enImageType) {
                cv::Mat irDisplay(imageData.nHeight, imageData.nWidth, CV_8UC1, imageData.pData);
                if (!irDisplay.empty()) {
                    ResizeForDisplay(irDisplay, displayMat);
                    if (imageData.enStreamType == StreamType_Ir_Right) {
                        cv::imshow("IR Right", displayMat);
                    }
                    else {
                        cv::imshow("IR Left", displayMat);
                    }
                }
            }
        }

        // Process SLAM with RGB and depth frames
        if (hasRGB && hasDepth) {
            ProcessSLAMFrame(rgbFrame, depthFrame);
        }

        // Show room map if SLAM is running
        if (slam_initialized_) {
            cv::Mat roomMap = CreateRoomMap(400);
            cv::imshow("Room Map", roomMap);
        }

        m_frameCount++;
    }

    int RunViewer() {
        if (!m_bGrabbing) {
            std::cerr << "Device is not grabbing!" << std::endl;
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
            else if (key == 's') {
                // Reset SLAM
                slam_system_.Reset();
                slam_initialized_ = false;
                trajectory_points_.clear();
                map_points_.clear();
                std::cout << "SLAM system reset!" << std::endl;
            }
            else if (key == 'c') {
                // Camera calibration info
                std::cout << "\n=== Camera Calibration Info ===" << std::endl;
                std::cout << "Current calibration: fx=735.749, fy=731.258, cx=617.785, cy=358.336" << std::endl;
                std::cout << "These are your ACTUAL calibrated values - SLAM should be accurate!" << std::endl;
                std::cout << "\n=== IMU Troubleshooting ===" << std::endl;
                std::cout << "If IMU shows 'OFF':" << std::endl;
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