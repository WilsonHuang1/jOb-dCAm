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

// Enhanced YOLO Detection Structure with tracking capabilities
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
    
    // 3D position (if depth available)
    cv::Point3f worldPos;    // 3D world position
    bool hasWorldPos;
    
    YOLODetection() : trackId(-1), framesSinceLastSeen(0), isTracked(false), 
                     hasValidDepth(false), hasWorldPos(false) {}
};

// Simple tracking structure
struct TrackedObject {
    int id;
    cv::Rect lastBbox;
    cv::Point2f lastCenter;
    float lastDepth;
    int framesSinceLastSeen;
    int classId;
    std::string className;
    bool isActive;
    
    TrackedObject(int trackId, const YOLODetection& detection) 
        : id(trackId), lastBbox(detection.bbox), lastCenter(detection.center),
          lastDepth(detection.avgDepth), framesSinceLastSeen(0),
          classId(detection.classId), className(detection.className), isActive(true) {}
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
    
    // Visualization methods
    void visualizeDetections(cv::Mat& rgbFrame, const std::vector<YOLODetection>& detections);
    void drawTrackingInfo(cv::Mat& frame, const YOLODetection& detection);
    void drawDepthInfo(cv::Mat& frame, const YOLODetection& detection);
    void drawPerformanceInfo(cv::Mat& frame);
    
    // Frame processing methods
    bool extractFrames(const StreamFrames& frames, cv::Mat& rgbFrame, cv::Mat& depthFrame);
    void processYUVFrame(const std::shared_ptr<StreamFrame>& frame, cv::Mat& rgbFrame);

public:
    // Constructor
    DepthYOLOTracker(const std::string& modelPath, float confThresh = 0.5, float nmsThresh = 0.4);
    
    // Destructor
    ~DepthYOLOTracker();
    
    // Main interface methods
    bool initializeDepthCamera();
    std::vector<YOLODetection> detectAndTrack(const StreamFrames& frames);
    void run();
    
    // Configuration methods
    void setConfidenceThreshold(float threshold) { confThreshold = threshold; }
    void setNMSThreshold(float threshold) { nmsThreshold = threshold; }
    void setMaxTrackingFrames(int frames) { const_cast<int&>(maxFramesWithoutDetection) = frames; }
    
    // Getter methods
    float getCurrentFPS() const { return currentFps; }
    int getActiveTrackCount() const;
    std::vector<TrackedObject> getActiveTracks() const;
    
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