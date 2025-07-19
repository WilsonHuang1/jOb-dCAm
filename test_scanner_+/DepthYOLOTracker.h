#ifndef DEPTH_YOLO_TRACKER_H
#define DEPTH_YOLO_TRACKER_H

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <iomanip>
#include <memory>
#include <map>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <numeric>

// Include Deptrum SDK headers
#include "deptrum/device.h"
#include "deptrum/stream.h"
#include "deptrum/stream_types.h"
#include "deptrum/common_types.h"
#include "functional/base.h"
#include "viewer_helper.hpp"

#ifdef DEVICE_TYPE_AURORA900
#include "deptrum/aurora900_series.h"
#endif
#ifdef DEVICE_TYPE_STELLAR400
#include "deptrum/stellar400_series.h"
#endif
#ifdef DEVICE_TYPE_STELLAR200
#include "deptrum/stellar200_series.h"
#endif

using namespace deptrum;
using namespace deptrum::stream;

// Enhanced YOLO Detection Structure with XYZ tracking capabilities
struct YOLODetection {
    int classId;
    float confidence;
    cv::Rect bbox;
    std::string className;
    float avgDepth;          // Average depth in the bounding box
    float minDepth;          // Minimum depth in the bounding box
    float maxDepth;          // Maximum depth in the bounding box
    bool hasValidDepth;
    
    // Tracking information
    int trackId;             // Unique tracking ID
    cv::Point2f center;      // Center point of detection
    float area;              // Area of bounding box
    int framesSinceLastSeen; // Frames since last detection
    bool isTracked;          // Whether this detection is being tracked
    
    // XYZ 3D position information
    cv::Point3f worldPos;        // 3D world position (X, Y, Z in meters)
    bool hasWorldPos;            // Whether world position calculation succeeded
    cv::Point3f bbox3DCenter;    // 3D center of bounding box
    float distance3D;            // Euclidean distance from camera
    
    // Enhanced depth information
    std::vector<float> depthSamples;  // Multiple depth samples for accuracy
    float depthVariance;         // Variance in depth measurements
    
    YOLODetection() : trackId(-1), framesSinceLastSeen(0), isTracked(false), 
                     hasValidDepth(false), hasWorldPos(false), distance3D(-1.0f), 
                     depthVariance(0.0f) {}
};

// Enhanced tracking structure with 3D capabilities
struct TrackedObject {
    int id;
    cv::Rect lastBbox;
    cv::Point2f lastCenter;
    float lastDepth;
    int framesSinceLastSeen;
    int classId;
    std::string className;
    bool isActive;
    float lastConfidence;
    
    // 3D tracking members
    cv::Point3f lastWorldPos;         // Last known 3D world position
    std::vector<cv::Point3f> worldPosHistory;  // History for smoothing
    bool hasValidWorldPos;            // Whether 3D position is valid
    float velocity3D;                 // 3D velocity magnitude
    cv::Point3f velocity3DVector;     // 3D velocity vector
    
    TrackedObject(int trackId, const YOLODetection& detection) 
        : id(trackId), lastBbox(detection.bbox), lastCenter(detection.center),
          lastDepth(detection.avgDepth), framesSinceLastSeen(0),
          classId(detection.classId), className(detection.className), isActive(true),
          lastConfidence(detection.confidence), hasValidWorldPos(false), velocity3D(0.0f) {}
    
    // Update 3D position with smoothing
    void updateWorldPosition(const cv::Point3f& newPos) {
        if (newPos.z > 0) {
            lastWorldPos = newPos;
            hasValidWorldPos = true;
            
            // Keep history for smoothing (max 10 frames)
            worldPosHistory.push_back(newPos);
            if (worldPosHistory.size() > 10) {
                worldPosHistory.erase(worldPosHistory.begin());
            }
        }
    }
    
    // Get smoothed world position
    cv::Point3f getSmoothedWorldPos() const {
        if (worldPosHistory.empty()) return lastWorldPos;
        
        cv::Point3f sum(0, 0, 0);
        for (const auto& pos : worldPosHistory) {
            sum.x += pos.x;
            sum.y += pos.y;
            sum.z += pos.z;
        }
        
        float count = static_cast<float>(worldPosHistory.size());
        return cv::Point3f(sum.x / count, sum.y / count, sum.z / count);
    }
};

class DepthYOLOTracker {
private:
    // YOLO detector components
    cv::dnn::Net yolo_net;
    std::vector<std::string> classNames;
    float confThreshold;
    float nmsThreshold;
    int inputWidth;
    int inputHeight;
    
    // Depth camera components
    std::shared_ptr<Device> depthDevice;
    std::shared_ptr<ViewerHelper> viewerHelper;
    CameraParam cameraParams;
    
    // Camera intrinsics for depth calculation
    Intrinsic rgbIntrinsic;
    Intrinsic irIntrinsic;
    Extrinsic extrinsic;
    
    // Tracking components
    std::map<int, TrackedObject> trackedObjects;
    int nextTrackId;
    const int maxFramesWithoutDetection = 30; // Remove tracks after 30 frames
    const float maxTrackingDistance = 100.0f; // Maximum distance for tracking association
    
    // Performance monitoring
    std::chrono::high_resolution_clock::time_point lastFpsUpdate;
    int frameCount;
    float currentFps;
    
    // XYZ-related members
    bool enableXYZDisplay;           // Enable XYZ coordinate display
    float maxValidDepth;             // Maximum valid depth in meters
    bool showCoordinateGuide;        // Show coordinate system guide
    bool enableDepthFiltering;       // Enable depth noise filtering
    
    // XYZ calculation parameters
    struct XYZCalculationParams {
        int depthSampleRadius = 3;   // Radius for depth sampling
        float outlierThreshold = 0.5f; // Threshold for outlier detection
        bool useMedianFiltering = true; // Use median filtering for depth
        float minValidDepth = 0.1f;  // Minimum valid depth (meters)
        float maxValidDepth = 8.0f;  // Maximum valid depth (meters)
    } xyzParams;
    
    // Class methods
    void initializeClassNames();
    cv::Mat preprocessYOLO(const cv::Mat& image);
    std::vector<cv::Rect> postprocessYOLO(const cv::Mat& image, const std::vector<cv::Mat>& outputs, 
                                         std::vector<int>& classIds, std::vector<float>& confidences);
    
    // Depth processing methods
    float calculateAverageDepth(const cv::Mat& depthMap, const cv::Rect& bbox);
    void calculateDepthStatistics(const cv::Mat& depthMap, const cv::Rect& bbox, 
                                 float& avgDepth, float& minDepth, float& maxDepth);
    cv::Point3f calculateWorldPosition(const cv::Point2f& imagePoint, float depth);
    
    // Tracking methods
    void updateTracking(std::vector<YOLODetection>& detections);
    float calculateTrackingDistance(const TrackedObject& track, const YOLODetection& detection);
    void assignDetectionToTrack(YOLODetection& detection, TrackedObject& track);
    void createNewTrack(YOLODetection& detection);
    void updateExistingTracks();
    void removeInactiveTracks();
    
    // Standard visualization methods
    void visualizeDetections(cv::Mat& rgbFrame, const std::vector<YOLODetection>& detections);
    void drawTrackingInfo(cv::Mat& frame, const YOLODetection& detection);
    void drawDepthInfo(cv::Mat& frame, const YOLODetection& detection);
    void drawPerformanceInfo(cv::Mat& frame);
    
    // Frame processing methods
    bool extractFrames(const StreamFrames& frames, cv::Mat& rgbFrame, cv::Mat& depthFrame);
    void processYUVFrame(const std::shared_ptr<StreamFrame>& frame, cv::Mat& rgbFrame);
    
    // Enhanced depth processing
    float filterDepthValue(float depth);
    std::vector<float> sampleDepthInRegion(const cv::Mat& depthFrame, const cv::Rect& region);
    float calculateDepthVariance(const std::vector<float>& depthSamples);
    
    // 3D tracking enhancements
    void updateTracking3D(std::vector<YOLODetection>& detections);
    void calculate3DVelocity(TrackedObject& track, const cv::Point3f& newWorldPos);
    cv::Point3f getSmoothed3DPosition(const TrackedObject& track);
    
    // Utility methods for 3D calculations
    float calculateDistance3D(const cv::Point3f& pos1, const cv::Point3f& pos2);
    bool isValid3DPosition(const cv::Point3f& worldPos);
    cv::Point3f projectTo3D(const cv::Point2f& imagePoint, float depth);

public:
    // Constructor
    DepthYOLOTracker(const std::string& modelPath, float confThresh = 0.5, float nmsThresh = 0.4);
    
    // Destructor
    ~DepthYOLOTracker();
    
    // Main interface methods
    bool initialize();
    bool initializeDepthCamera();
    std::vector<YOLODetection> detectAndTrack(const StreamFrames& frames);
    void run();
    
    // Enhanced visualization with XYZ coordinates
    void visualizeDetectionsWithXYZ(cv::Mat& rgbFrame, const std::vector<YOLODetection>& detections);
    void drawXYZCoordinates(cv::Mat& frame, const YOLODetection& detection);
    void drawXYZIndicator(cv::Mat& frame, const cv::Point2f& center, const cv::Point3f& worldPos);
    void drawCoordinateSystemGuide(cv::Mat& frame);
    void drawDepthVisualization(cv::Mat& frame, const cv::Mat& depthFrame, const std::vector<YOLODetection>& detections);
    
    // Enhanced 3D position calculation
    cv::Point3f calculateWorldPositionEnhanced(const cv::Point2f& imagePoint, float depth);
    std::vector<cv::Point3f> calculateBoundingBox3D(const cv::Rect& bbox, const cv::Mat& depthFrame);
    float calculateRobustDepth(const cv::Mat& depthFrame, const cv::Rect& bbox);
    
    // XYZ data management
    void printXYZDetectionInfo(const std::vector<YOLODetection>& detections);
    void saveXYZCoordinatesToCSV(const std::vector<YOLODetection>& detections, const std::string& filename = "");
    void logXYZTrackingData(const std::map<int, TrackedObject>& tracks);
    
    // Configuration methods
    void setConfidenceThreshold(float threshold) { confThreshold = threshold; }
    void setNMSThreshold(float threshold) { nmsThreshold = threshold; }
    void setMaxTrackingFrames(int frames) { const_cast<int&>(maxFramesWithoutDetection) = frames; }
    
    // Configuration methods for XYZ features
    void enableXYZCoordinateDisplay(bool enable) { enableXYZDisplay = enable; }
    void setMaxValidDepth(float maxDepth) { maxValidDepth = maxDepth; xyzParams.maxValidDepth = maxDepth; }
    void setDepthFilteringEnabled(bool enable) { enableDepthFiltering = enable; }
    void setCoordinateGuideVisible(bool show) { showCoordinateGuide = show; }
    void setXYZCalculationParams(const XYZCalculationParams& params) { xyzParams = params; }
    
    // Getter methods
    float getCurrentFPS() const { return currentFps; }
    int getActiveTrackCount() const;
    std::vector<TrackedObject> getActiveTracks() const;
    std::map<int, TrackedObject> getActiveTracksMap() const { return trackedObjects; }
    
    // Getter methods for XYZ data
    bool isXYZDisplayEnabled() const { return enableXYZDisplay; }
    float getMaxValidDepth() const { return maxValidDepth; }
    XYZCalculationParams getXYZParams() const { return xyzParams; }
    
    // Enhanced main loop with XYZ features
    void runWithXYZCoordinates();
    
    // Reset and utility methods
    void resetTracking();
    
    // Utility methods
    void printDetectionInfo(const std::vector<YOLODetection>& detections);
    void saveDetectionResults(const std::vector<YOLODetection>& detections, const std::string& filename);
};

// Enhanced parameters structure for advanced depth detection
struct EnhancedDepthParams {
    float roi_x_ratio = 0.3f;          // ROI width ratio for static detection
    float roi_y_ratio = 0.3f;          // ROI height ratio for static detection
    float depth_filter_threshold = 50.0f;  // Filter out depth values below this
    float depth_max_range = 6000.0f;   // Maximum depth range in mm
    float tracking_confidence_boost = 0.1f; // Confidence boost for tracked objects
    bool enable_depth_filtering = true;
    bool enable_world_coordinates = true;
    bool enable_tracking_smoothing = true;
};

// Detection callback interface for external integration
class DetectionCallback {
public:
    virtual ~DetectionCallback() = default;
    virtual void onDetection(const std::vector<YOLODetection>& detections) = 0;
    virtual void onTrackingUpdate(const std::vector<TrackedObject>& tracks) = 0;
};

// Utility functions
namespace DepthYOLOUtils {
    // Distance calculation utilities
    float calculateEuclideanDistance(const cv::Point2f& p1, const cv::Point2f& p2);
    float calculateBboxOverlap(const cv::Rect& bbox1, const cv::Rect& bbox2);
    
    // Depth utilities
    bool isValidDepthValue(float depth, float minRange = 100.0f, float maxRange = 6000.0f);
    cv::Mat filterDepthMap(const cv::Mat& depthMap, float minDepth, float maxDepth);
    
    // Visualization utilities
    cv::Scalar getClassColor(int classId);
    std::string formatDepthString(float depth);
    std::string formatConfidenceString(float confidence);
    
    // Performance utilities
    void logPerformanceMetrics(float fps, int detectionCount, int trackCount);
    
    // File I/O utilities
    bool saveDetectionLog(const std::vector<YOLODetection>& detections, 
                         const std::string& timestamp);
    bool loadYOLOModel(const std::string& modelPath, cv::dnn::Net& net);
}

#endif // DEPTH_YOLO_TRACKER_H