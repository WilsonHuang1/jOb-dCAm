#include "DepthYOLOTracker.h"
#include <algorithm>
#include <fstream>
#include <sstream>

// Constructor
DepthYOLOTracker::DepthYOLOTracker(const std::string& modelPath, float confThresh, float nmsThresh)
    : confThreshold(confThresh), nmsThreshold(nmsThresh), inputWidth(640), inputHeight(640),
      nextTrackId(0), frameCount(0), currentFps(0.0f) {
    
    initializeClassNames();
    lastFpsUpdate = std::chrono::high_resolution_clock::now();
    
    // Initialize YOLO
    try {
        yolo_net = cv::dnn::readNetFromONNX(modelPath);
        
        if (cv::cuda::getCudaEnabledDeviceCount() > 0) {
            std::cout << "Using CUDA backend for YOLO" << std::endl;
            yolo_net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
            yolo_net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
        } else {
            std::cout << "Using CPU backend for YOLO" << std::endl;
            yolo_net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
            yolo_net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        }
        
        std::cout << "YOLO model loaded successfully!" << std::endl;
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to load YOLO model: " + std::string(e.what()));
    }
}

// Destructor
DepthYOLOTracker::~DepthYOLOTracker() {
    if (depthDevice) {
        depthDevice->Stop();
        depthDevice->Close();
    }
    cv::destroyAllWindows();
}

void DepthYOLOTracker::initializeClassNames() {
    classNames = {
        "person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
        "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
        "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
        "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
        "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
        "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli",
        "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed",
        "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
        "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
        "teddy bear", "hair drier", "toothbrush"
    };
}

cv::Mat DepthYOLOTracker::preprocessYOLO(const cv::Mat& image) {
    cv::Mat blob;
    cv::dnn::blobFromImage(image, blob, 1.0/255.0, cv::Size(inputWidth, inputHeight), cv::Scalar(0,0,0), true, false);
    return blob;
}

std::vector<cv::Rect> DepthYOLOTracker::postprocessYOLO(const cv::Mat& image, const std::vector<cv::Mat>& outputs,
                                                        std::vector<int>& classIds, std::vector<float>& confidences) {
    std::vector<cv::Rect> boxes;
    float x_factor = image.cols / static_cast<float>(inputWidth);
    float y_factor = image.rows / static_cast<float>(inputHeight);

    const float* data = (float*)outputs[0].data;
    const int dimensions = 85; // 4 + 1 + 80 for COCO
    const int rows = outputs[0].size[1];

    for (int i = 0; i < rows; ++i) {
        float confidence = data[4];
        if (confidence >= confThreshold) {
            float* classes_scores = data + 5;
            cv::Mat scores(1, classNames.size(), CV_32FC1, classes_scores);
            cv::Point class_id;
            double max_class_score;
            minMaxLoc(scores, 0, &max_class_score, 0, &class_id);
            
            if (max_class_score > confThreshold) {
                float cx = data[0];
                float cy = data[1];
                float w = data[2];
                float h = data[3];
                
                int left = int((cx - 0.5 * w) * x_factor);
                int top = int((cy - 0.5 * h) * y_factor);
                int width = int(w * x_factor);
                int height = int(h * y_factor);
                
                boxes.push_back(cv::Rect(left, top, width, height));
                classIds.push_back(class_id.x);
                confidences.push_back(confidence);
            }
        }
        data += dimensions;
    }
    return boxes;
}

void DepthYOLOTracker::calculateDepthStatistics(const cv::Mat& depthMap, const cv::Rect& bbox,
                                               float& avgDepth, float& minDepth, float& maxDepth) {
    avgDepth = minDepth = maxDepth = -1.0f;
    
    if (depthMap.empty() || bbox.area() == 0) return;
    
    cv::Rect safeRect = bbox & cv::Rect(0, 0, depthMap.cols, depthMap.rows);
    if (safeRect.area() == 0) return;
    
    cv::Mat roi = depthMap(safeRect);
    cv::Mat mask = roi > 0; // Only consider non-zero depth values
    
    if (cv::countNonZero(mask) == 0) return;
    
    cv::Scalar meanVal, stdVal;
    cv::meanStdDev(roi, meanVal, stdVal, mask);
    
    double minVal, maxVal;
    cv::minMaxLoc(roi, &minVal, &maxVal, nullptr, nullptr, mask);
    
    avgDepth = static_cast<float>(meanVal[0]);
    minDepth = static_cast<float>(minVal);
    maxDepth = static_cast<float>(maxVal);
}

float DepthYOLOTracker::calculateAverageDepth(const cv::Mat& depthMap, const cv::Rect& bbox) {
    float avgDepth, minDepth, maxDepth;
    calculateDepthStatistics(depthMap, bbox, avgDepth, minDepth, maxDepth);
    return avgDepth;
}

cv::Point3f DepthYOLOTracker::calculateWorldPosition(const cv::Point2f& imagePoint, float depth) {
    if (depth <= 0) return cv::Point3f(-1, -1, -1);
    
    // Convert from image coordinates to world coordinates using camera intrinsics
    float x = (imagePoint.x - cameraParams.cx) * depth / cameraParams.fx;
    float y = (imagePoint.y - cameraParams.cy) * depth / cameraParams.fy;
    float z = depth;
    
    return cv::Point3f(x, y, z);
}

void DepthYOLOTracker::updateTracking(std::vector<YOLODetection>& detections) {
    // Update existing tracks
    updateExistingTracks();
    
    // Associate detections with existing tracks
    std::vector<bool> detectionAssigned(detections.size(), false);
    
    for (auto& [trackId, track] : trackedObjects) {
        if (!track.isActive) continue;
        
        float bestDistance = std::numeric_limits<float>::max();
        int bestDetectionIdx = -1;
        
        for (int i = 0; i < detections.size(); ++i) {
            if (detectionAssigned[i]) continue;
            if (detections[i].classId != track.classId) continue;
            
            float distance = calculateTrackingDistance(track, detections[i]);
            if (distance < maxTrackingDistance && distance < bestDistance) {
                bestDistance = distance;
                bestDetectionIdx = i;
            }
        }
        
        if (bestDetectionIdx >= 0) {
            assignDetectionToTrack(detections[bestDetectionIdx], track);
            detectionAssigned[bestDetectionIdx] = true;
        }
    }
    
    // Create new tracks for unassigned detections
    for (int i = 0; i < detections.size(); ++i) {
        if (!detectionAssigned[i]) {
            createNewTrack(detections[i]);
        }
    }
    
    // Remove inactive tracks
    removeInactiveTracks();
}

float DepthYOLOTracker::calculateTrackingDistance(const TrackedObject& track, const YOLODetection& detection) {
    // Calculate distance based on center point and depth (if available)
    float centerDistance = DepthYOLOUtils::calculateEuclideanDistance(track.lastCenter, detection.center);
    
    if (detection.hasValidDepth && track.lastDepth > 0) {
        float depthDistance = std::abs(detection.avgDepth - track.lastDepth) / 100.0f; // Scale depth difference
        return centerDistance + depthDistance;
    }
    
    return centerDistance;
}

void DepthYOLOTracker::assignDetectionToTrack(YOLODetection& detection, TrackedObject& track) {
    detection.trackId = track.id;
    detection.isTracked = true;
    
    track.lastBbox = detection.bbox;
    track.lastCenter = detection.center;
    track.lastDepth = detection.avgDepth;
    track.framesSinceLastSeen = 0;
}

void DepthYOLOTracker::createNewTrack(YOLODetection& detection) {
    detection.trackId = nextTrackId;
    detection.isTracked = true;
    
    trackedObjects[nextTrackId] = TrackedObject(nextTrackId, detection);
    nextTrackId++;
}

void DepthYOLOTracker::updateExistingTracks() {
    for (auto& [trackId, track] : trackedObjects) {
        track.framesSinceLastSeen++;
    }
}

void DepthYOLOTracker::removeInactiveTracks() {
    auto it = trackedObjects.begin();
    while (it != trackedObjects.end()) {
        if (it->second.framesSinceLastSeen > maxFramesWithoutDetection) {
            it = trackedObjects.erase(it);
        } else {
            ++it;
        }
    }
}

bool DepthYOLOTracker::initializeDepthCamera() {
    try {
        // Register device hotplug callback (optional)
        DeviceManager::GetInstance()->RegisterDeviceConnectedCallback();
        
        // Get device list
        std::vector<DeviceInformation> deviceList;
        int ret = DeviceManager::GetInstance()->GetDeviceList(deviceList);
        if (ret != 0 || deviceList.empty()) {
            std::cerr << "No depth camera devices found!" << std::endl;
            return false;
        }
        
        // Create device (use first available)
        depthDevice = DeviceManager::GetInstance()->CreateDevice(deviceList[0]);
        if (!depthDevice) {
            std::cerr << "Failed to create depth device!" << std::endl;
            return false;
        }
        
        // Open device
        ret = depthDevice->Open();
        if (ret != 0) {
            std::cerr << "Failed to open depth device!" << std::endl;
            return false;
        }
        
        // Get device name and setup
        std::string deviceName = depthDevice->GetDeviceName();
        std::cout << "Connected to depth camera: " << deviceName << std::endl;
        
        // Get supported frame modes
        std::vector<std::tuple<FrameMode, FrameMode, FrameMode>> deviceResolutionVec;
        depthDevice->GetSupportedFrameMode(deviceResolutionVec);
        
        if (!deviceResolutionVec.empty()) {
            // Use first available mode
            FrameMode irMode = std::get<0>(deviceResolutionVec[0]);
            FrameMode rgbMode = std::get<1>(deviceResolutionVec[0]);
            FrameMode depthMode = std::get<2>(deviceResolutionVec[0]);
            
            ret = depthDevice->SetMode(irMode, rgbMode, depthMode);
            if (ret != 0) {
                std::cerr << "Failed to set camera modes!" << std::endl;
                return false;
            }
        }
        
        // Get camera parameters
        depthDevice->GetCameraParameters(irIntrinsic, rgbIntrinsic, extrinsic);
        
        cameraParams.cx = irIntrinsic.principal_point[0];
        cameraParams.cy = irIntrinsic.principal_point[1];
        cameraParams.fx = irIntrinsic.focal_length[0];
        cameraParams.fy = irIntrinsic.focal_length[1];
        
        // Initialize viewer helper
        viewerHelper = std::make_shared<ViewerHelper>(deviceName);
        
        std::cout << "Depth camera initialized successfully!" << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error initializing depth camera: " << e.what() << std::endl;
        return false;
    }
}

bool DepthYOLOTracker::extractFrames(const StreamFrames& frames, cv::Mat& rgbFrame, cv::Mat& depthFrame) {
    bool hasRGB = false, hasDepth = false;
    
    for (int i = 0; i < frames.count; i++) {
        auto frame = frames.frame_ptr[i];
        
        if (frame->frame_type == kRgbFrame) {
            processYUVFrame(frame, rgbFrame);
            hasRGB = !rgbFrame.empty();
        }
        else if (frame->frame_type == kDepthFrame) {
            if (frame->cols > 0 && frame->rows > 0) {
                depthFrame = cv::Mat(frame->rows, frame->cols, CV_16UC1, frame->data.get());
                hasDepth = true;
            }
        }
    }
    
    return hasRGB;
}

void DepthYOLOTracker::processYUVFrame(const std::shared_ptr<StreamFrame>& frame, cv::Mat& rgbFrame) {
    if (frame->cols > 0 && frame->rows > 0) {
        cv::Mat yuvMat(frame->rows * 1.5f, frame->cols, CV_8UC1, frame->data.get());
        rgbFrame = cv::Mat(frame->rows, frame->cols, CV_8UC3);
        cv::cvtColor(yuvMat, rgbFrame, cv::COLOR_YUV2BGR_NV12);
    }
}

std::vector<YOLODetection> DepthYOLOTracker::detectAndTrack(const StreamFrames& frames) {
    std::vector<YOLODetection> detections;
    
    if (frames.count == 0) return detections;
    
    cv::Mat rgbFrame, depthFrame;
    bool hasRGB = extractFrames(frames, rgbFrame, depthFrame);
    bool hasDepth = !depthFrame.empty();
    
    if (!hasRGB) {
        std::cerr << "No RGB frame available!" << std::endl;
        return detections;
    }
    
    // Run YOLO detection on RGB frame
    cv::Mat blob = preprocessYOLO(rgbFrame);
    yolo_net.setInput(blob);
    
    std::vector<cv::Mat> outputs;
    yolo_net.forward(outputs, yolo_net.getUnconnectedOutLayersNames());
    
    std::vector<int> classIds;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes = postprocessYOLO(rgbFrame, outputs, classIds, confidences);
    
    // Apply Non-Maximum Suppression
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
    
    // Create detection results with depth information
    for (int idx : indices) {
        YOLODetection detection;
        detection.classId = classIds[idx];
        detection.confidence = confidences[idx];
        detection.bbox = boxes[idx];
        detection.className = classNames[detection.classId];
        detection.center = cv::Point2f(detection.bbox.x + detection.bbox.width/2.0f,
                                      detection.bbox.y + detection.bbox.height/2.0f);
        detection.area = detection.bbox.area();
        
        // Calculate depth information if available
        if (hasDepth) {
            calculateDepthStatistics(depthFrame, detection.bbox, 
                                   detection.avgDepth, detection.minDepth, detection.maxDepth);
            detection.hasValidDepth = (detection.avgDepth > 0);
            
            if (detection.hasValidDepth) {
                detection.worldPos = calculateWorldPosition(detection.center, detection.avgDepth);
                detection.hasWorldPos = (detection.worldPos.z > 0);
            }
        } else {
            detection.avgDepth = detection.minDepth = detection.maxDepth = -1.0f;
            detection.hasValidDepth = false;
            detection.hasWorldPos = false;
        }
        
        detections.push_back(detection);
    }
    
    // Update tracking
    updateTracking(detections);
    
    return detections;
}

void DepthYOLOTracker::visualizeDetections(cv::Mat& rgbFrame, const std::vector<YOLODetection>& detections) {
    for (const auto& detection : detections) {
        drawTrackingInfo(rgbFrame, detection);
        drawDepthInfo(rgbFrame, detection);
    }
    
    drawPerformanceInfo(rgbFrame);
}

void DepthYOLOTracker::drawTrackingInfo(cv::Mat& frame, const YOLODetection& detection) {
    // Choose color based on tracking status
    cv::Scalar color = detection.isTracked ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
    if (detection.hasValidDepth) {
        color = cv::Scalar(0, 255, 255); // Yellow for depth + tracking
    }
    
    // Draw bounding box
    cv::rectangle(frame, detection.bbox, color, 2);
    
    // Draw tracking ID if tracked
    if (detection.isTracked) {
        std::string trackText = "ID:" + std::to_string(detection.trackId);
        cv::putText(frame, trackText, 
                   cv::Point(detection.bbox.x, detection.bbox.y - 30),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
    }
    
    // Draw center point
    cv::circle(frame, detection.center, 3, color, -1);
}

void DepthYOLOTracker::drawDepthInfo(cv::Mat& frame, const YOLODetection& detection) {
    // Prepare label text
    std::string label = detection.className + " " + 
                       DepthYOLOUtils::formatConfidenceString(detection.confidence);
    
    if (detection.hasValidDepth) {
        label += " " + DepthYOLOUtils::formatDepthString(detection.avgDepth);
    }
    
    // Draw label background
    int baseline;
    cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
    cv::Point labelPos(detection.bbox.x, detection.bbox.y - 10);
    
    cv::Scalar color = detection.hasValidDepth ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
    cv::rectangle(frame, 
                 cv::Point(labelPos.x, labelPos.y - labelSize.height), 
                 cv::Point(labelPos.x + labelSize.width, labelPos.y + baseline),
                 color, cv::FILLED);
    
    // Draw label text
    cv::putText(frame, label, labelPos, cv::FONT_HERSHEY_SIMPLEX, 0.5, 
               cv::Scalar(255, 255, 255), 1);
}

void DepthYOLOTracker::drawPerformanceInfo(cv::Mat& frame) {
    // Calculate FPS
    frameCount++;
    auto currentTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastFpsUpdate);
    
    if (duration.count() >= 1000) { // Update FPS every second
        currentFps = frameCount * 1000.0f / duration.count();
        frameCount = 0;
        lastFpsUpdate = currentTime;
    }
    
    // Draw FPS and tracking info
    std::string fpsText = "FPS: " + std::to_string(static_cast<int>(currentFps));
    std::string trackText = "Tracks: " + std::to_string(getActiveTrackCount());
    
    cv::putText(frame, fpsText, cv::Point(10, 30), 
               cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
    cv::putText(frame, trackText, cv::Point(10, 70), 
               cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
}

int DepthYOLOTracker::getActiveTrackCount() const {
    int count = 0;
    for (const auto& [id, track] : trackedObjects) {
        if (track.isActive && track.framesSinceLastSeen < maxFramesWithoutDetection) {
            count++;
        }
    }
    return count;
}

std::vector<TrackedObject> DepthYOLOTracker::getActiveTracks() const {
    std::vector<TrackedObject> activeTracks;
    for (const auto& [id, track] : trackedObjects) {
        if (track.isActive && track.framesSinceLastSeen < maxFramesWithoutDetection) {
            activeTracks.push_back(track);
        }
    }
    return activeTracks;
}

void DepthYOLOTracker::printDetectionInfo(const std::vector<YOLODetection>& detections) {
    for (const auto& detection : detections) {
        std::cout << "Detected: " << detection.className 
                 << " (ID: " << detection.trackId << ")"
                 << " (conf: " << std::fixed << std::setprecision(2) << detection.confidence << ")";
        if (detection.hasValidDepth) {
            std::cout << " at " << static_cast<int>(detection.avgDepth) << "mm";
            if (detection.hasWorldPos) {
                std::cout << " pos: (" << static_cast<int>(detection.worldPos.x) 
                         << "," << static_cast<int>(detection.worldPos.y) 
                         << "," << static_cast<int>(detection.worldPos.z) << ")";
            }
        }
        std::cout << std::endl;
    }
}

void DepthYOLOTracker::run() {
    if (!initializeDepthCamera()) {
        std::cerr << "Failed to initialize depth camera!" << std::endl;
        return;
    }
    
    std::cout << "Starting enhanced depth camera with YOLO tracking..." << std::endl;
    std::cout << "Press 'q' to quit, 's' to save detection log" << std::endl;
    
    while (true) {
        StreamFrames frames;
        int ret = depthDevice->GetFrames(frames, 2000);
        
        if (ret == 0 && frames.count > 0) {
            // Detect objects and get depth information
            auto detections = detectAndTrack(frames);
            
            // Get RGB frame for visualization
            cv::Mat displayFrame, depthFrame;
            extractFrames(frames, displayFrame, depthFrame);
            
            if (!displayFrame.empty()) {
                // Visualize detections
                visualizeDetections(displayFrame, detections);
                
                // Print detection information
                if (!detections.empty()) {
                    printDetectionInfo(detections);
                }
                
                // Display the frame
                cv::imshow("Depth Camera + YOLO Tracking", displayFrame);
            }
            
            // Also show depth visualization using ViewerHelper
            viewerHelper->ShowFrame(frames, cameraParams);
        }
        
        // Check for quit key
        char key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 27) { // 'q' or ESC key
            break;
        }
        else if (key == 's') { // Save detection log
            auto activeTracks = getActiveTracks();
            std::cout << "Saving detection log with " << activeTracks.size() << " active tracks" << std::endl;
            // Implementation for saving would go here
        }
    }
    
    std::cout << "Shutting down..." << std::endl;
}

// Utility namespace implementations
namespace DepthYOLOUtils {
    
    float calculateEuclideanDistance(const cv::Point2f& p1, const cv::Point2f& p2) {
        float dx = p1.x - p2.x;
        float dy = p1.y - p2.y;
        return std::sqrt(dx * dx + dy * dy);
    }
    
    float calculateBboxOverlap(const cv::Rect& bbox1, const cv::Rect& bbox2) {
        cv::Rect intersection = bbox1 & bbox2;
        cv::Rect unionRect = bbox1 | bbox2;
        
        if (intersection.area() == 0 || unionRect.area() == 0) {
            return 0.0f;
        }
        
        return static_cast<float>(intersection.area()) / static_cast<float>(unionRect.area());
    }
    
    bool isValidDepthValue(float depth, float minRange, float maxRange) {
        return depth > minRange && depth < maxRange && depth != std::numeric_limits<float>::infinity();
    }
    
    cv::Mat filterDepthMap(const cv::Mat& depthMap, float minDepth, float maxDepth) {
        cv::Mat filtered;
        depthMap.copyTo(filtered);
        
        cv::Mat mask = (depthMap >= minDepth) & (depthMap <= maxDepth) & (depthMap > 0);
        filtered.setTo(0, ~mask);
        
        return filtered;
    }
    
    cv::Scalar getClassColor(int classId) {
        // Generate consistent colors for different classes
        static std::vector<cv::Scalar> colors = {
            cv::Scalar(255, 0, 0),     // Red
            cv::Scalar(0, 255, 0),     // Green
            cv::Scalar(0, 0, 255),     // Blue
            cv::Scalar(255, 255, 0),   // Cyan
            cv::Scalar(255, 0, 255),   // Magenta
            cv::Scalar(0, 255, 255),   // Yellow
            cv::Scalar(128, 0, 128),   // Purple
            cv::Scalar(255, 165, 0),   // Orange
            cv::Scalar(0, 128, 128),   // Teal
            cv::Scalar(128, 128, 0)    // Olive
        };
        
        return colors[classId % colors.size()];
    }
    
    std::string formatDepthString(float depth) {
        if (depth <= 0) return "N/A";
        
        if (depth < 1000) {
            return std::to_string(static_cast<int>(depth)) + "mm";
        } else {
            return std::to_string(static_cast<int>(depth / 10) / 100.0f) + "m";
        }
    }
    
    std::string formatConfidenceString(float confidence) {
        return std::to_string(static_cast<int>(confidence * 100)) + "%";
    }
    
    void logPerformanceMetrics(float fps, int detectionCount, int trackCount) {
        static auto startTime = std::chrono::high_resolution_clock::now();
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime);
        
        std::cout << "[" << duration.count() << "s] FPS: " << fps 
                  << " | Detections: " << detectionCount 
                  << " | Active Tracks: " << trackCount << std::endl;
    }
    
    bool saveDetectionLog(const std::vector<YOLODetection>& detections, const std::string& timestamp) {
        std::string filename = "detection_log_" + timestamp + ".csv";
        std::ofstream file(filename);
        
        if (!file.is_open()) {
            std::cerr << "Failed to open file for writing: " << filename << std::endl;
            return false;
        }
        
        // Write CSV header
        file << "timestamp,track_id,class_name,confidence,bbox_x,bbox_y,bbox_w,bbox_h,";
        file << "center_x,center_y,avg_depth,min_depth,max_depth,world_x,world_y,world_z\n";
        
        // Write detection data
        for (const auto& detection : detections) {
            file << timestamp << ","
                 << detection.trackId << ","
                 << detection.className << ","
                 << detection.confidence << ","
                 << detection.bbox.x << ","
                 << detection.bbox.y << ","
                 << detection.bbox.width << ","
                 << detection.bbox.height << ","
                 << detection.center.x << ","
                 << detection.center.y << ","
                 << detection.avgDepth << ","
                 << detection.minDepth << ","
                 << detection.maxDepth << ","
                 << detection.worldPos.x << ","
                 << detection.worldPos.y << ","
                 << detection.worldPos.z << "\n";
        }
        
        file.close();
        std::cout << "Detection log saved to: " << filename << std::endl;
        return true;
    }
    
    bool loadYOLOModel(const std::string& modelPath, cv::dnn::Net& net) {
        try {
            net = cv::dnn::readNetFromONNX(modelPath);
            
            // Set backend and target
            if (cv::cuda::getCudaEnabledDeviceCount() > 0) {
                std::cout << "Using CUDA backend" << std::endl;
                net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
                net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
            } else {
                std::cout << "Using CPU backend" << std::endl;
                net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
                net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
            }
            
            return true;
        } catch (const std::exception& e) {
            std::cerr << "Failed to load YOLO model: " << e.what() << std::endl;
            return false;
        }
    }
}

// Main function with enhanced argument parsing
int main(int argc, char* argv[]) {
    std::string modelPath = "yolov8n.onnx";  // Default model path
    float confThreshold = 0.5f;
    float nmsThreshold = 0.4f;
    
    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        
        if (arg == "--model" && i + 1 < argc) {
            modelPath = argv[++i];
        }
        else if (arg == "--conf" && i + 1 < argc) {
            confThreshold = std::stof(argv[++i]);
        }
        else if (arg == "--nms" && i + 1 < argc) {
            nmsThreshold = std::stof(argv[++i]);
        }
        else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0] << " [options]\n"
                      << "Options:\n"
                      << "  --model <path>    Path to YOLO ONNX model (default: yolov8n.onnx)\n"
                      << "  --conf <value>    Confidence threshold (default: 0.5)\n"
                      << "  --nms <value>     NMS threshold (default: 0.4)\n"
                      << "  --help, -h        Show this help message\n";
            return 0;
        }
    }
    
    std::cout << "Configuration:\n"
              << "  Model: " << modelPath << "\n"
              << "  Confidence threshold: " << confThreshold << "\n"
              << "  NMS threshold: " << nmsThreshold << "\n";
    
    try {
        // Create and run the integrated tracker
        DepthYOLOTracker tracker(modelPath, confThreshold, nmsThreshold);
        tracker.run();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}