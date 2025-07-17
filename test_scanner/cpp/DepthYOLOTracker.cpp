#include "DepthYOLOTracker.h"

// Constructor with XYZ initialization
DepthYOLOTracker::DepthYOLOTracker(const std::string& modelPath, float confThresh, float nmsThresh)
    : confThreshold(confThresh), nmsThreshold(nmsThresh), nextTrackId(1), frameCount(0), currentFps(0.0f),
      enableXYZDisplay(true), maxValidDepth(8.0f), showCoordinateGuide(true), enableDepthFiltering(true),
      inputWidth(640), inputHeight(640) {
    
    // Initialize class names
    initializeClassNames();
    
    // Load YOLO model
    try {
        yolo_net = cv::dnn::readNetFromONNX(modelPath);
        
        // Try CUDA first, fall back to CPU
        if (cv::cuda::getCudaEnabledDeviceCount() > 0) {
            std::cout << "Using CUDA for YOLO inference" << std::endl;
            yolo_net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
            yolo_net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
        } else {
            std::cout << "Using CPU for YOLO inference" << std::endl;
            yolo_net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
            yolo_net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        }
        
        std::cout << "YOLO model loaded successfully from: " << modelPath << std::endl;
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to load YOLO model: " + std::string(e.what()));
    }
    
    // Initialize timing
    lastFpsUpdate = std::chrono::high_resolution_clock::now();
}

// Destructor
DepthYOLOTracker::~DepthYOLOTracker() {
    if (depthDevice) {
        // Cleanup will be handled by smart pointers
    }
}

// Initialize class names
void DepthYOLOTracker::initializeClassNames() {
    classNames = {
        "person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
        "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
        "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
        "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
        "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
        "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
        "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake",
        "chair", "sofa", "pottedplant", "bed", "diningtable", "toilet", "tvmonitor",
        "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven",
        "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
        "teddy bear", "hair drier", "toothbrush"
    };
}

// Initialize depth camera
bool DepthYOLOTracker::initializeDepthCamera() {
    std::cout << "Initializing depth camera..." << std::endl;
    
    // Register device hotplug callback
    DeviceManager::GetInstance()->RegisterDeviceConnectedCallback();

    // Query connected devices
    std::vector<DeviceInformation> device_list;
    int ret = DeviceManager::GetInstance()->GetDeviceList(device_list);
    if (ret != 0 || device_list.empty()) {
        std::cerr << "No depth cameras found" << std::endl;
        return false;
    }
    
    std::cout << "Found " << device_list.size() << " depth camera(s)" << std::endl;
    
    // Create device
    depthDevice = DeviceManager::GetInstance()->CreateDevice(device_list[0]);
    if (!depthDevice) {
        std::cerr << "Failed to create device" << std::endl;
        return false;
    }

    ret = depthDevice->Open();
    if (ret != 0) {
        std::cerr << "Failed to open device" << std::endl;
        return false;
    }

    // Get device name and setup viewer
    std::string device_name = depthDevice->GetDeviceName();
    std::cout << "Device name: " << device_name << std::endl;
    viewerHelper = std::make_shared<ViewerHelper>(device_name);

    // Get camera parameters
    depthDevice->GetCameraParameters(irIntrinsic, rgbIntrinsic, extrinsic);
    
    cameraParams.cx = irIntrinsic.principal_point[0];
    cameraParams.cy = irIntrinsic.principal_point[1];
    cameraParams.fx = irIntrinsic.focal_length[0];
    cameraParams.fy = irIntrinsic.focal_length[1];
    
    std::cout << "Camera parameters - fx: " << cameraParams.fx 
              << ", fy: " << cameraParams.fy 
              << ", cx: " << cameraParams.cx 
              << ", cy: " << cameraParams.cy << std::endl;

    return true;
}

// Initialize
bool DepthYOLOTracker::initialize() {
    return initializeDepthCamera();
}

// Preprocess image for YOLO
cv::Mat DepthYOLOTracker::preprocessYOLO(const cv::Mat& image) {
    cv::Mat blob;
    cv::dnn::blobFromImage(image, blob, 1.0/255.0, cv::Size(inputWidth, inputHeight), cv::Scalar(0,0,0), true, false);
    return blob;
}

// Postprocess YOLO outputs
std::vector<cv::Rect> DepthYOLOTracker::postprocessYOLO(const cv::Mat& image, const std::vector<cv::Mat>& outputs, 
                                                        std::vector<int>& classIds, std::vector<float>& confidences) {
    std::vector<cv::Rect> boxes;
    classIds.clear();
    confidences.clear();
    
    if (outputs.empty()) return boxes;
    
    float x_factor = static_cast<float>(image.cols) / inputWidth;
    float y_factor = static_cast<float>(image.rows) / inputHeight;
    
    cv::Mat output = outputs[0];
    const int num_classes = 80;
    const int num_boxes = output.size[2];
    
    // YOLOv8 format: transpose output for easier processing
    cv::Mat output_transposed;
    cv::transpose(output.reshape(1, output.size[1]), output_transposed);
    
    const float* data = output_transposed.ptr<float>();
    
    for (int i = 0; i < num_boxes; ++i) {
        const float* row = data + i * output.size[1];
        
        // Extract bbox coordinates
        float cx = row[0];
        float cy = row[1];
        float w = row[2];
        float h = row[3];
        
        // Find class with highest score
        float max_class_score = 0.0f;
        int best_class_id = -1;
        
        for (int j = 0; j < num_classes; ++j) {
            float class_score = row[4 + j];
            if (class_score > max_class_score) {
                max_class_score = class_score;
                best_class_id = j;
            }
        }
        
        float confidence = max_class_score;
        
        if (confidence >= confThreshold && best_class_id >= 0) {
            // Convert to pixel coordinates
            float x1 = (cx - w * 0.5f) * x_factor;
            float y1 = (cy - h * 0.5f) * y_factor;
            float x2 = (cx + w * 0.5f) * x_factor;
            float y2 = (cy + h * 0.5f) * y_factor;
            
            // Clamp to image bounds
            x1 = std::max(0.0f, std::min(x1, static_cast<float>(image.cols - 1)));
            y1 = std::max(0.0f, std::min(y1, static_cast<float>(image.rows - 1)));
            x2 = std::max(x1 + 1.0f, std::min(x2, static_cast<float>(image.cols)));
            y2 = std::max(y1 + 1.0f, std::min(y2, static_cast<float>(image.rows)));
            
            int bbox_x = static_cast<int>(x1);
            int bbox_y = static_cast<int>(y1);
            int bbox_w = static_cast<int>(x2 - x1);
            int bbox_h = static_cast<int>(y2 - y1);
            
            if (bbox_w > 0 && bbox_h > 0) {
                boxes.push_back(cv::Rect(bbox_x, bbox_y, bbox_w, bbox_h));
                classIds.push_back(best_class_id);
                confidences.push_back(confidence);
            }
        }
    }
    
    return boxes;
}

// Extract frames from StreamFrames
bool DepthYOLOTracker::extractFrames(const StreamFrames& frames, cv::Mat& rgbFrame, cv::Mat& depthFrame) {
    rgbFrame = cv::Mat();
    depthFrame = cv::Mat();
    
    for (int i = 0; i < frames.count; i++) {
        auto frame = frames.frame_ptr[i];
        
        if (frame->frame_type == kRgbFrame && frame->cols > 0 && frame->rows > 0) {
            cv::Mat yuvMat(static_cast<int>(frame->rows * 1.5f), frame->cols, CV_8UC1, frame->data.get());
            rgbFrame = cv::Mat(frame->rows, frame->cols, CV_8UC3);
            cv::cvtColor(yuvMat, rgbFrame, cv::COLOR_YUV2BGR_NV12);
        }
        else if (frame->frame_type == kDepthFrame && frame->cols > 0 && frame->rows > 0) {
            depthFrame = cv::Mat(frame->rows, frame->cols, CV_16UC1, frame->data.get());
        }
    }
    
    return !rgbFrame.empty();
}

// Calculate depth statistics
void DepthYOLOTracker::calculateDepthStatistics(const cv::Mat& depthMap, const cv::Rect& bbox, 
                                               float& avgDepth, float& minDepth, float& maxDepth) {
    avgDepth = minDepth = maxDepth = -1.0f;
    
    if (bbox.x < 0 || bbox.y < 0 || 
        bbox.x + bbox.width >= depthMap.cols || 
        bbox.y + bbox.height >= depthMap.rows) {
        return;
    }
    
    cv::Mat roi = depthMap(bbox);
    cv::Mat mask = roi > 0;
    
    if (cv::countNonZero(mask) == 0) return;
    
    double minVal, maxVal;
    cv::minMaxLoc(roi, &minVal, &maxVal, nullptr, nullptr, mask);
    
    cv::Scalar meanVal = cv::mean(roi, mask);
    
    avgDepth = static_cast<float>(meanVal[0]) / 1000.0f; // Convert mm to meters
    minDepth = static_cast<float>(minVal) / 1000.0f;
    maxDepth = static_cast<float>(maxVal) / 1000.0f;
}

// Calculate world position from image point and depth
cv::Point3f DepthYOLOTracker::calculateWorldPosition(const cv::Point2f& imagePoint, float depth) {
    if (depth <= 0) return cv::Point3f(-1, -1, -1);
    
    // Convert from image coordinates to world coordinates using camera intrinsics
    float x = (imagePoint.x - cameraParams.cx) * depth / cameraParams.fx;
    float y = (imagePoint.y - cameraParams.cy) * depth / cameraParams.fy;
    float z = depth;
    
    return cv::Point3f(x, y, z);
}

// Enhanced world position calculation
cv::Point3f DepthYOLOTracker::calculateWorldPositionEnhanced(const cv::Point2f& imagePoint, float depth) {
    if (depth <= xyzParams.minValidDepth || depth > xyzParams.maxValidDepth) {
        return cv::Point3f(-1, -1, -1);
    }
    
    // Apply depth filtering if enabled
    if (enableDepthFiltering) {
        depth = filterDepthValue(depth);
    }
    
    // Convert from image coordinates to world coordinates using camera intrinsics
    float x = (imagePoint.x - cameraParams.cx) * depth / cameraParams.fx;
    float y = (imagePoint.y - cameraParams.cy) * depth / cameraParams.fy;
    float z = depth;
    
    return cv::Point3f(x, y, z);
}

// Calculate robust depth
float DepthYOLOTracker::calculateRobustDepth(const cv::Mat& depthFrame, const cv::Rect& bbox) {
    if (bbox.x < 0 || bbox.y < 0 || 
        bbox.x + bbox.width >= depthFrame.cols || 
        bbox.y + bbox.height >= depthFrame.rows) {
        return -1.0f;
    }
    
    // Sample multiple points within the bounding box
    std::vector<float> depthSamples = sampleDepthInRegion(depthFrame, bbox);
    
    if (depthSamples.empty()) return -1.0f;
    
    // Use median for robust depth estimation
    if (xyzParams.useMedianFiltering && depthSamples.size() > 1) {
        std::sort(depthSamples.begin(), depthSamples.end());
        size_t middle = depthSamples.size() / 2;
        return depthSamples[middle];
    } else if (!depthSamples.empty()) {
        // Use mean if only one sample or median filtering disabled
        float sum = std::accumulate(depthSamples.begin(), depthSamples.end(), 0.0f);
        return sum / depthSamples.size();
    }
    
    return -1.0f;
}

// Sample depth in region
std::vector<float> DepthYOLOTracker::sampleDepthInRegion(const cv::Mat& depthFrame, const cv::Rect& region) {
    std::vector<float> samples;
    
    // Sample points in a grid pattern within the region
    int stepX = std::max(1, region.width / 4);
    int stepY = std::max(1, region.height / 4);
    
    for (int y = region.y; y < region.y + region.height; y += stepY) {
        for (int x = region.x; x < region.x + region.width; x += stepX) {
            if (x >= 0 && y >= 0 && x < depthFrame.cols && y < depthFrame.rows) {
                float depth = depthFrame.at<uint16_t>(y, x) / 1000.0f; // Convert mm to meters
                
                // Filter valid depth values
                if (depth > xyzParams.minValidDepth && depth < xyzParams.maxValidDepth) {
                    samples.push_back(depth);
                }
            }
        }
    }
    
    return samples;
}

// Main detection and tracking method with XYZ calculation
std::vector<YOLODetection> DepthYOLOTracker::detectAndTrack(const StreamFrames& frames) {
    std::vector<YOLODetection> detections;
    
    // Extract frames
    cv::Mat rgbFrame, depthFrame;
    if (!extractFrames(frames, rgbFrame, depthFrame)) {
        std::cerr << "Failed to extract frames" << std::endl;
        return detections;
    }
    
    if (rgbFrame.empty()) {
        return detections;
    }
    
    // Preprocess image for YOLO
    cv::Mat blob = preprocessYOLO(rgbFrame);
    yolo_net.setInput(blob);
    
    // Run YOLO inference
    std::vector<cv::Mat> outputs;
    yolo_net.forward(outputs, yolo_net.getUnconnectedOutLayersNames());
    
    // Postprocess YOLO outputs
    std::vector<int> classIds;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes = postprocessYOLO(rgbFrame, outputs, classIds, confidences);
    
    // Apply Non-Maximum Suppression
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
    
    // Create detection results with depth and XYZ information
    for (int idx : indices) {
        YOLODetection detection;
        detection.classId = classIds[idx];
        detection.confidence = confidences[idx];
        detection.bbox = boxes[idx];
        detection.className = classIds[idx] < classNames.size() ? classNames[classIds[idx]] : "unknown";
        detection.center = cv::Point2f(detection.bbox.x + detection.bbox.width/2.0f,
                                      detection.bbox.y + detection.bbox.height/2.0f);
        detection.area = detection.bbox.area();
        
        // Calculate depth information if available
        if (!depthFrame.empty()) {
            calculateDepthStatistics(depthFrame, detection.bbox, 
                                   detection.avgDepth, detection.minDepth, detection.maxDepth);
            detection.hasValidDepth = (detection.avgDepth > 0);
            
            if (detection.hasValidDepth) {
                // Calculate robust depth for XYZ calculation
                float robustDepth = calculateRobustDepth(depthFrame, detection.bbox);
                
                if (robustDepth > 0) {
                    // Calculate enhanced world position
                    detection.worldPos = calculateWorldPositionEnhanced(detection.center, robustDepth);
                    detection.hasWorldPos = isValid3DPosition(detection.worldPos);
                    
                    if (detection.hasWorldPos) {
                        // Calculate 3D distance from camera
                        detection.distance3D = std::sqrt(
                            detection.worldPos.x * detection.worldPos.x +
                            detection.worldPos.y * detection.worldPos.y +
                            detection.worldPos.z * detection.worldPos.z
                        );
                        
                        // Sample depth for variance calculation
                        detection.depthSamples = sampleDepthInRegion(depthFrame, detection.bbox);
                        detection.depthVariance = calculateDepthVariance(detection.depthSamples);
                    }
                }
            }
        } else {
            detection.avgDepth = detection.minDepth = detection.maxDepth = -1.0f;
            detection.hasValidDepth = false;
            detection.hasWorldPos = false;
        }
        
        detections.push_back(detection);
    }
    
    // Update tracking with 3D information
    if (enableXYZDisplay) {
        updateTracking3D(detections);
    } else {
        updateTracking(detections);
    }
    
    return detections;
}

// Enhanced visualization with XYZ coordinates
void DepthYOLOTracker::visualizeDetectionsWithXYZ(cv::Mat& rgbFrame, const std::vector<YOLODetection>& detections) {
    // Draw coordinate system guide first
    if (showCoordinateGuide) {
        drawCoordinateSystemGuide(rgbFrame);
    }
    
    // Draw each detection with XYZ information
    for (const auto& detection : detections) {
        // Draw standard tracking info
        drawTrackingInfo(rgbFrame, detection);
        
        // Draw XYZ coordinates if available
        if (enableXYZDisplay && detection.hasWorldPos) {
            drawXYZCoordinates(rgbFrame, detection);
            drawXYZIndicator(rgbFrame, detection.center, detection.worldPos);
        }
        
        // Draw depth info
        drawDepthInfo(rgbFrame, detection);
    }
    
    // Draw performance info
    drawPerformanceInfo(rgbFrame);
}

// Draw XYZ coordinates
void DepthYOLOTracker::drawXYZCoordinates(cv::Mat& frame, const YOLODetection& detection) {
    if (!detection.hasWorldPos) return;
    
    // Colors for XYZ display
    cv::Scalar xColor(0, 0, 255);    // Red for X
    cv::Scalar yColor(0, 255, 0);    // Green for Y
    cv::Scalar zColor(255, 0, 0);    // Blue for Z
    cv::Scalar bgColor(0, 0, 0);     // Black background
    
    // Calculate text position
    int textY = detection.bbox.y - 80;
    if (textY < 50) textY = detection.bbox.y + detection.bbox.height + 80;
    int textX = detection.bbox.x;
    
    // Format coordinates with appropriate precision
    std::ostringstream xStream, yStream, zStream, distStream;
    xStream << std::fixed << std::setprecision(3) << detection.worldPos.x;
    yStream << std::fixed << std::setprecision(3) << detection.worldPos.y;
    zStream << std::fixed << std::setprecision(3) << detection.worldPos.z;
    distStream << std::fixed << std::setprecision(2) << detection.distance3D;
    
    std::string xText = "X: " + xStream.str() + "m";
    std::string yText = "Y: " + yStream.str() + "m";
    std::string zText = "Z: " + zStream.str() + "m";
    std::string distText = "Dist: " + distStream.str() + "m";
    
    // Font settings
    int font = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 0.4;
    int thickness = 1;
    
    // Get text sizes for background rectangles
    cv::Size xSize = cv::getTextSize(xText, font, fontScale, thickness, nullptr);
    cv::Size ySize = cv::getTextSize(yText, font, fontScale, thickness, nullptr);
    cv::Size zSize = cv::getTextSize(zText, font, fontScale, thickness, nullptr);
    cv::Size distSize = cv::getTextSize(distText, font, fontScale, thickness, nullptr);
    
    int maxWidth = std::max({xSize.width, ySize.width, zSize.width, distSize.width});
    
    // Draw background rectangle
    cv::rectangle(frame, 
                  cv::Point(textX - 3, textY - xSize.height - 3),
                  cv::Point(textX + maxWidth + 3, textY + 50),
                  bgColor, -1);
    
    // Draw coordinate text
    cv::putText(frame, xText, cv::Point(textX, textY), font, fontScale, xColor, thickness);
    cv::putText(frame, yText, cv::Point(textX, textY + 12), font, fontScale, yColor, thickness);
    cv::putText(frame, zText, cv::Point(textX, textY + 24), font, fontScale, zColor, thickness);
    cv::putText(frame, distText, cv::Point(textX, textY + 36), font, fontScale, cv::Scalar(255, 255, 255), thickness);
}

// Draw XYZ indicator at object center
void DepthYOLOTracker::drawXYZIndicator(cv::Mat& frame, const cv::Point2f& center, const cv::Point3f& worldPos) {
    int axisLength = 25;
    cv::Point2f centerPt(center.x, center.y);
    
    // X-axis (Red) - horizontal right
    cv::Point2f xEnd(center.x + axisLength, center.y);
    cv::arrowedLine(frame, centerPt, xEnd, cv::Scalar(0, 0, 255), 2, 8, 0, 0.3);
    cv::putText(frame, "X", cv::Point(xEnd.x + 3, xEnd.y + 5), 
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 255), 1);
    
    // Y-axis (Green) - vertical down
    cv::Point2f yEnd(center.x, center.y + axisLength);
    cv::arrowedLine(frame, centerPt, yEnd, cv::Scalar(0, 255, 0), 2, 8, 0, 0.3);
    cv::putText(frame, "Y", cv::Point(yEnd.x + 3, yEnd.y + 5), 
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1);
    
    // Z-axis (Blue) - diagonal (representing depth)
    cv::Point2f zEnd(center.x - axisLength*0.7, center.y - axisLength*0.7);
    cv::arrowedLine(frame, centerPt, zEnd, cv::Scalar(255, 0, 0), 2, 8, 0, 0.3);
    cv::putText(frame, "Z", cv::Point(zEnd.x - 15, zEnd.y - 5), 
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 0, 0), 1);
    
    // Center point
    cv::circle(frame, centerPt, 3, cv::Scalar(255, 255, 255), -1);
}

// Draw coordinate system guide
void DepthYOLOTracker::drawCoordinateSystemGuide(cv::Mat& frame) {
    int guideX = 20;
    int guideY = 50;
    int axisLength = 35;
    
    // Background rectangle
    cv::rectangle(frame, 
                  cv::Point(guideX - 10, guideY - 40),
                  cv::Point(guideX + 140, guideY + 50),
                  cv::Scalar(0, 0, 0), -1);
    cv::rectangle(frame, 
                  cv::Point(guideX - 10, guideY - 40),
                  cv::Point(guideX + 140, guideY + 50),
                  cv::Scalar(255, 255, 255), 1);
    
    // Title
    cv::putText(frame, "3D Coordinates", cv::Point(guideX, guideY - 25), 
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
    
    cv::Point2f origin(guideX, guideY);
    
    // X-axis (Red)
    cv::Point2f xEnd(guideX + axisLength, guideY);
    cv::arrowedLine(frame, origin, xEnd, cv::Scalar(0, 0, 255), 2, 8, 0, 0.3);
    cv::putText(frame, "X (Right)", cv::Point(guideX + 40, guideY + 5), 
                cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 255), 1);
    
    // Y-axis (Green)
    cv::Point2f yEnd(guideX, guideY + axisLength);
    cv::arrowedLine(frame, origin, yEnd, cv::Scalar(0, 255, 0), 2, 8, 0, 0.3);
    cv::putText(frame, "Y (Down)", cv::Point(guideX + 40, guideY + 15), 
                cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 255, 0), 1);
    
    // Z-axis (Blue)
    cv::Point2f zEnd(guideX - axisLength*0.7, guideY - axisLength*0.7);
    cv::arrowedLine(frame, origin, zEnd, cv::Scalar(255, 0, 0), 2, 8, 0, 0.3);
    cv::putText(frame, "Z (Depth)", cv::Point(guideX + 40, guideY + 25), 
                cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 0, 0), 1);
    
    // Origin point
    cv::circle(frame, origin, 2, cv::Scalar(255, 255, 255), -1);
}

// Standard visualization (existing method)
void DepthYOLOTracker::visualizeDetections(cv::Mat& rgbFrame, const std::vector<YOLODetection>& detections) {
    for (const auto& detection : detections) {
        drawTrackingInfo(rgbFrame, detection);
        drawDepthInfo(rgbFrame, detection);
    }
    
    drawPerformanceInfo(rgbFrame);
}

// Draw tracking info
void DepthYOLOTracker::drawTrackingInfo(cv::Mat& frame, const YOLODetection& detection) {
    // Choose color based on tracking status
    cv::Scalar color = detection.isTracked ? 
        cv::Scalar(0, 255, 0) :  // Green for tracked
        cv::Scalar(0, 255, 255); // Yellow for new detection
    
    // Draw bounding box
    cv::rectangle(frame, detection.bbox, color, 2);
    
    // Create label with class, confidence, and tracking info
    std::string label = detection.className + " (" + 
                       std::to_string(static_cast<int>(detection.confidence * 100)) + "%)";
    
    if (detection.isTracked) {
        label += " ID:" + std::to_string(detection.trackId);
    }
    
    // Add depth information to label
    if (detection.hasValidDepth) {
        label += " D:" + std::to_string(detection.avgDepth).substr(0, 4) + "m";
    }
    
    // Draw label background
    int baseline = 0;
    cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
    cv::rectangle(frame, 
                  cv::Point(detection.bbox.x, detection.bbox.y - labelSize.height - 10),
                  cv::Point(detection.bbox.x + labelSize.width, detection.bbox.y),
                  color, -1);
    
    // Draw label text
    cv::putText(frame, label, 
                cv::Point(detection.bbox.x, detection.bbox.y - 5),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    
    // Draw center point
    cv::circle(frame, detection.center, 4, color, -1);
}

// Draw depth info
void DepthYOLOTracker::drawDepthInfo(cv::Mat& frame, const YOLODetection& detection) {
    if (!detection.hasValidDepth) return;
    
    // Draw depth information
    std::string depthText = "Depth: " + std::to_string(static_cast<int>(detection.avgDepth * 1000)) + "mm";
    cv::putText(frame, depthText, 
                cv::Point(detection.bbox.x, detection.bbox.y + detection.bbox.height + 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
}

// Draw performance info
void DepthYOLOTracker::drawPerformanceInfo(cv::Mat& frame) {
    // Update FPS calculation
    auto now = std::chrono::high_resolution_clock::now();
    frameCount++;
    
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastFpsUpdate);
    if (elapsed.count() >= 1000) { // Update every second
        currentFps = frameCount * 1000.0f / elapsed.count();
        frameCount = 0;
        lastFpsUpdate = now;
    }
    
    // Draw FPS and tracking info
    std::string fpsText = "FPS: " + std::to_string(static_cast<int>(currentFps));
    std::string trackText = "Tracks: " + std::to_string(getActiveTrackCount());
    
    cv::putText(frame, fpsText, cv::Point(frame.cols - 120, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
    cv::putText(frame, trackText, cv::Point(frame.cols - 120, 60), 
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
}

// Print XYZ detection information
void DepthYOLOTracker::printXYZDetectionInfo(const std::vector<YOLODetection>& detections) {
    std::cout << "\n=== XYZ Detection Report ===" << std::endl;
    std::cout << std::fixed << std::setprecision(3);
    
    for (size_t i = 0; i < detections.size(); ++i) {
        const auto& detection = detections[i];
        
        std::cout << "[" << i+1 << "] " << detection.className 
                  << " (conf: " << std::setprecision(1) << (detection.confidence * 100) << "%)";
        
        if (detection.isTracked) {
            std::cout << " [Track ID: " << detection.trackId << "]";
        }
        
        if (detection.hasWorldPos) {
            std::cout << "\n    World Position (XYZ): ("
                      << std::setprecision(3) << detection.worldPos.x << ", "
                      << detection.worldPos.y << ", "
                      << detection.worldPos.z << ") meters";
            std::cout << "\n    Distance: " << std::setprecision(2) << detection.distance3D << "m";
        }
        
        if (detection.hasValidDepth) {
            std::cout << "\n    Depth: avg=" << std::setprecision(3) << detection.avgDepth
                      << "m, var=" << std::setprecision(4) << detection.depthVariance;
        }
        
        std::cout << "\n    Image Center: (" << std::setprecision(1) 
                  << detection.center.x << ", " << detection.center.y << ")";
        std::cout << "\n" << std::endl;
    }
}

// Save XYZ coordinates to CSV
void DepthYOLOTracker::saveXYZCoordinatesToCSV(const std::vector<YOLODetection>& detections, const std::string& filename) {
    // Generate filename with timestamp if not provided
    std::string csvFilename = filename;
    if (csvFilename.empty()) {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        auto tm = *std::localtime(&time_t);
        
        std::ostringstream filenameStream;
        filenameStream << "xyz_coordinates_" 
                      << std::put_time(&tm, "%Y%m%d_%H%M%S") 
                      << ".csv";
        csvFilename = filenameStream.str();
    }
    
    std::ofstream file(csvFilename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file for saving: " << csvFilename << std::endl;
        return;
    }
    
    // Write CSV header
    file << "Timestamp,ClassName,Confidence,TrackID,X,Y,Z,Distance,ImageX,ImageY,BboxWidth,BboxHeight,DepthAvg,DepthVar\n";
    
    // Write current detections
    auto currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    
    for (const auto& detection : detections) {
        file << currentTime << ","
             << detection.className << ","
             << std::fixed << std::setprecision(3) << detection.confidence << ","
             << detection.trackId << ",";
        
        if (detection.hasWorldPos) {
            file << detection.worldPos.x << ","
                 << detection.worldPos.y << ","
                 << detection.worldPos.z << ","
                 << detection.distance3D << ",";
        } else {
            file << ",,,,";
        }
        
        file << detection.center.x << ","
             << detection.center.y << ","
             << detection.bbox.width << ","
             << detection.bbox.height << ",";
        
        if (detection.hasValidDepth) {
            file << detection.avgDepth << ","
                 << detection.depthVariance;
        } else {
            file << ",";
        }
        
        file << "\n";
    }
    
    file.close();
    std::cout << "XYZ coordinates saved to: " << csvFilename << std::endl;
    std::cout << "Saved " << detections.size() << " detections" << std::endl;
}

// Enhanced main loop with XYZ features
void DepthYOLOTracker::runWithXYZCoordinates() {
    if (!initialize()) {
        std::cerr << "Failed to initialize DepthYOLOTracker" << std::endl;
        return;
    }
    
    std::cout << "Starting enhanced depth camera with YOLO tracking and XYZ coordinates..." << std::endl;
    std::cout << "Controls:" << std::endl;
    std::cout << "  'q' or ESC - Quit" << std::endl;
    std::cout << "  's' - Save XYZ coordinates to CSV" << std::endl;
    std::cout << "  'r' - Reset tracking" << std::endl;
    std::cout << "  'c' - Toggle coordinate display" << std::endl;
    std::cout << "  'g' - Toggle coordinate guide" << std::endl;
    std::cout << "  'f' - Toggle depth filtering" << std::endl;
    
    // Initialize streaming
    Stream* stream = nullptr;
    std::vector<StreamType> stream_types = {StreamType::kRgbd};
    int ret = depthDevice->CreateStream(stream, stream_types);
    if (ret != 0) {
        std::cerr << "Failed to create stream" << std::endl;
        return;
    }
    
    ret = stream->Start();
    if (ret != 0) {
        std::cerr << "Failed to start stream" << std::endl;
        return;
    }
    
    while (true) {
        StreamFrames frames;
        ret = stream->GetFrames(frames, 2000);
        
        if (ret == 0 && frames.count > 0) {
            // Detect objects with XYZ coordinates
            auto detections = detectAndTrack(frames);
            
            // Get display frame
            cv::Mat displayFrame, depthFrame;
            extractFrames(frames, displayFrame, depthFrame);
            
            if (!displayFrame.empty()) {
                // Use enhanced visualization with XYZ
                visualizeDetectionsWithXYZ(displayFrame, detections);
                
                // Print XYZ info (every 30 frames to avoid spam)
                static int printCounter = 0;
                if (++printCounter % 30 == 0 && !detections.empty()) {
                    printXYZDetectionInfo(detections);
                }
                
                // Display frame
                cv::imshow("YOLO + Depth Camera + XYZ Coordinates", displayFrame);
            }
            
            // Show depth visualization
            viewerHelper->ShowFrame(frames, cameraParams);
        }
        
        // Handle keyboard input
        char key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 27) break;
        else if (key == 's') {
            saveXYZCoordinatesToCSV(detections);
        }
        else if (key == 'r') {
            resetTracking();
        }
        else if (key == 'c') {
            enableXYZDisplay = !enableXYZDisplay;
            std::cout << "XYZ display: " << (enableXYZDisplay ? "ON" : "OFF") << std::endl;
        }
        else if (key == 'g') {
            showCoordinateGuide = !showCoordinateGuide;
            std::cout << "Coordinate guide: " << (showCoordinateGuide ? "ON" : "OFF") << std::endl;
        }
        else if (key == 'f') {
            enableDepthFiltering = !enableDepthFiltering;
            std::cout << "Depth filtering: " << (enableDepthFiltering ? "ON" : "OFF") << std::endl;
        }
    }
    
    // Cleanup
    stream->Stop();
    depthDevice->DestroyStream(stream);
    cv::destroyAllWindows();
    
    std::cout << "Shutting down XYZ tracker..." << std::endl;
}

// Helper methods
float DepthYOLOTracker::filterDepthValue(float depth) {
    if (depth < xyzParams.minValidDepth || depth > xyzParams.maxValidDepth) {
        return -1.0f;
    }
    return depth;
}

bool DepthYOLOTracker::isValid3DPosition(const cv::Point3f& worldPos) {
    return (worldPos.x != -1.0f && worldPos.y != -1.0f && worldPos.z != -1.0f &&
            worldPos.z > xyzParams.minValidDepth && worldPos.z < xyzParams.maxValidDepth);
}

float DepthYOLOTracker::calculateDepthVariance(const std::vector<float>& depthSamples) {
    if (depthSamples.size() < 2) return 0.0f;
    
    float mean = std::accumulate(depthSamples.begin(), depthSamples.end(), 0.0f) / depthSamples.size();
    float variance = 0.0f;
    
    for (float depth : depthSamples) {
        variance += (depth - mean) * (depth - mean);
    }
    
    return variance / (depthSamples.size() - 1);
}

// Standard tracking methods (keep existing implementation)
void DepthYOLOTracker::updateTracking(std::vector<YOLODetection>& detections) {
    // Update existing tracks
    updateExistingTracks();
    
    // Associate detections with existing tracks
    std::vector<bool> detectionAssigned(detections.size(), false);
    
    for (auto& [trackId, track] : trackedObjects) {
        if (!track.isActive) continue;
        
        float bestDistance = std::numeric_limits<float>::max();
        int bestDetectionIdx = -1;
        
        for (size_t i = 0; i < detections.size(); ++i) {
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
    for (size_t i = 0; i < detections.size(); ++i) {
        if (!detectionAssigned[i]) {
            createNewTrack(detections[i]);
        }
    }
    
    // Remove inactive tracks
    removeInactiveTracks();
}

// Enhanced tracking with 3D information
void DepthYOLOTracker::updateTracking3D(std::vector<YOLODetection>& detections) {
    // Update existing tracks
    updateExistingTracks();
    
    // Associate detections with existing tracks
    std::vector<bool> detectionAssigned(detections.size(), false);
    
    for (auto& [trackId, track] : trackedObjects) {
        if (!track.isActive) continue;
        
        float bestDistance = std::numeric_limits<float>::max();
        int bestDetectionIdx = -1;
        
        for (size_t i = 0; i < detections.size(); ++i) {
            if (detectionAssigned[i]) continue;
            if (detections[i].classId != track.classId) continue;
            
            // Use 3D distance if available, otherwise fall back to 2D
            float distance;
            if (detections[i].hasWorldPos && track.hasValidWorldPos) {
                distance = calculateDistance3D(track.lastWorldPos, detections[i].worldPos);
            } else {
                distance = calculateTrackingDistance(track, detections[i]);
            }
            
            if (distance < maxTrackingDistance && distance < bestDistance) {
                bestDistance = distance;
                bestDetectionIdx = i;
            }
        }
        
        if (bestDetectionIdx >= 0) {
            assignDetectionToTrack(detections[bestDetectionIdx], track);
            detectionAssigned[bestDetectionIdx] = true;
            
            // Update 3D position if available
            if (detections[bestDetectionIdx].hasWorldPos) {
                track.updateWorldPosition(detections[bestDetectionIdx].worldPos);
                calculate3DVelocity(track, detections[bestDetectionIdx].worldPos);
            }
        }
    }
    
    // Create new tracks for unassigned detections
    for (size_t i = 0; i < detections.size(); ++i) {
        if (!detectionAssigned[i]) {
            createNewTrack(detections[i]);
        }
    }
    
    // Remove inactive tracks
    removeInactiveTracks();
}

float DepthYOLOTracker::calculateTrackingDistance(const TrackedObject& track, const YOLODetection& detection) {
    // Calculate Euclidean distance between centers
    float dx = track.lastCenter.x - detection.center.x;
    float dy = track.lastCenter.y - detection.center.y;
    return std::sqrt(dx * dx + dy * dy);
}

void DepthYOLOTracker::assignDetectionToTrack(YOLODetection& detection, TrackedObject& track) {
    detection.trackId = track.id;
    detection.isTracked = true;
    
    // Update track information
    track.lastBbox = detection.bbox;
    track.lastCenter = detection.center;
    track.lastDepth = detection.avgDepth;
    track.framesSinceLastSeen = 0;
    track.lastConfidence = detection.confidence;
}

void DepthYOLOTracker::createNewTrack(YOLODetection& detection) {
    TrackedObject newTrack(nextTrackId++, detection);
    
    // Initialize 3D position if available
    if (detection.hasWorldPos) {
        newTrack.updateWorldPosition(detection.worldPos);
    }
    
    trackedObjects[newTrack.id] = newTrack;
    
    detection.trackId = newTrack.id;
    detection.isTracked = true;
}

void DepthYOLOTracker::updateExistingTracks() {
    for (auto& [trackId, track] : trackedObjects) {
        if (track.isActive) {
            track.framesSinceLastSeen++;
            if (track.framesSinceLastSeen > maxFramesWithoutDetection) {
                track.isActive = false;
            }
        }
    }
}

void DepthYOLOTracker::removeInactiveTracks() {
    auto it = trackedObjects.begin();
    while (it != trackedObjects.end()) {
        if (!it->second.isActive) {
            it = trackedObjects.erase(it);
        } else {
            ++it;
        }
    }
}

float DepthYOLOTracker::calculateDistance3D(const cv::Point3f& pos1, const cv::Point3f& pos2) {
    float dx = pos1.x - pos2.x;
    float dy = pos1.y - pos2.y;
    float dz = pos1.z - pos2.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void DepthYOLOTracker::calculate3DVelocity(TrackedObject& track, const cv::Point3f& newWorldPos) {
    if (track.hasValidWorldPos) {
        cv::Point3f velocity = newWorldPos - track.lastWorldPos;
        track.velocity3DVector = velocity;
        track.velocity3D = std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z);
    }
}

int DepthYOLOTracker::getActiveTrackCount() const {
    int count = 0;
    for (const auto& [trackId, track] : trackedObjects) {
        if (track.isActive) count++;
    }
    return count;
}

std::vector<TrackedObject> DepthYOLOTracker::getActiveTracks() const {
    std::vector<TrackedObject> activeTracks;
    for (const auto& [trackId, track] : trackedObjects) {
        if (track.isActive) {
            activeTracks.push_back(track);
        }
    }
    return activeTracks;
}

void DepthYOLOTracker::resetTracking() {
    trackedObjects.clear();
    nextTrackId = 1;
    std::cout << "Tracking reset - all tracks cleared" << std::endl;
}

void DepthYOLOTracker::printDetectionInfo(const std::vector<YOLODetection>& detections) {
    std::cout << "Detected " << detections.size() << " objects:" << std::endl;
    for (const auto& detection : detections) {
        std::cout << "  " << detection.className 
                  << " (conf: " << std::fixed << std::setprecision(1) << (detection.confidence * 100) << "%)";
        if (detection.isTracked) {
            std::cout << " [ID: " << detection.trackId << "]";
        }
        if (detection.hasValidDepth) {
            std::cout << " [Depth: " << std::setprecision(2) << detection.avgDepth << "m]";
        }
        std::cout << std::endl;
    }
}

void DepthYOLOTracker::saveDetectionResults(const std::vector<YOLODetection>& detections, const std::string& filename) {
    saveXYZCoordinatesToCSV(detections, filename);
}

// Standard run method (without XYZ features)
void DepthYOLOTracker::run() {
    if (!initialize()) {
        std::cerr << "Failed to initialize DepthYOLOTracker" << std::endl;
        return;
    }
    
    std::cout << "Starting depth camera with YOLO tracking..." << std::endl;
    std::cout << "Press 'q' to quit, 's' to save detection log" << std::endl;
    
    // Initialize streaming
    Stream* stream = nullptr;
    std::vector<StreamType> stream_types = {StreamType::kRgbd};
    int ret = depthDevice->CreateStream(stream, stream_types);
    if (ret != 0) {
        std::cerr << "Failed to create stream" << std::endl;
        return;
    }
    
    ret = stream->Start();
    if (ret != 0) {
        std::cerr << "Failed to start stream" << std::endl;
        return;
    }
    
    while (true) {
        StreamFrames frames;
        ret = stream->GetFrames(frames, 2000);
        
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
            saveDetectionResults(detections, "");
        }
    }
    
    // Cleanup
    stream->Stop();
    depthDevice->DestroyStream(stream);
    cv::destroyAllWindows();
    
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
        
        return static_cast<float>(intersection.area()) / unionRect.area();
    }
    
    bool isValidDepthValue(float depth, float minRange, float maxRange) {
        return depth > minRange && depth < maxRange;
    }
    
    cv::Mat filterDepthMap(const cv::Mat& depthMap, float minDepth, float maxDepth) {
        cv::Mat filtered;
        cv::Mat mask = (depthMap > minDepth) & (depthMap < maxDepth);
        depthMap.copyTo(filtered, mask);
        return filtered;
    }
    
    cv::Scalar getClassColor(int classId) {
        // Generate consistent colors for different classes
        cv::RNG rng(classId * 12345);
        return cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
    }
    
    std::string formatDepthString(float depth) {
        if (depth <= 0) return "No depth";
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2) << depth << "m";
        return oss.str();
    }
    
    std::string formatConfidenceString(float confidence) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1) << (confidence * 100) << "%";
        return oss.str();
    }
    
    void logPerformanceMetrics(float fps, int detectionCount, int trackCount) {
        std::cout << "Performance - FPS: " << fps 
                  << ", Detections: " << detectionCount 
                  << ", Tracks: " << trackCount << std::endl;
    }
    
    bool saveDetectionLog(const std::vector<YOLODetection>& detections, const std::string& timestamp) {
        std::string filename = "detection_log_" + timestamp + ".txt";
        std::ofstream file(filename);
        
        if (!file.is_open()) {
            return false;
        }
        
        file << "Detection Log - " << timestamp << std::endl;
        file << "=====================================" << std::endl;
        
        for (const auto& detection : detections) {
            file << "Class: " << detection.className 
                 << ", Confidence: " << detection.confidence
                 << ", Bbox: [" << detection.bbox.x << "," << detection.bbox.y 
                 << "," << detection.bbox.width << "x" << detection.bbox.height << "]";
            
            if (detection.hasWorldPos) {
                file << ", XYZ: (" << detection.worldPos.x << "," 
                     << detection.worldPos.y << "," << detection.worldPos.z << ")";
            }
            
            file << std::endl;
        }
        
        file.close();
        return true;
    }
    
    bool loadYOLOModel(const std::string& modelPath, cv::dnn::Net& net) {
        try {
            net = cv::dnn::readNetFromONNX(modelPath);
            return true;
        } catch (const std::exception& e) {
            std::cerr << "Failed to load YOLO model: " << e.what() << std::endl;
            return false;
        }
    }
}