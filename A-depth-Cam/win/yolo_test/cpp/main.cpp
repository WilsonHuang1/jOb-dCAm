#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <iomanip>

class YOLOv8Detector {
private:
    cv::dnn::Net net;
    std::vector<std::string> classNames;
    float confThreshold;
    float nmsThreshold;
    int inputWidth;
    int inputHeight;

    void initializeClassNames() {
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

public:
    YOLOv8Detector(const std::string& modelPath, float confThresh = 0.5, float nmsThresh = 0.4)
        : confThreshold(confThresh), nmsThreshold(nmsThresh), inputWidth(640), inputHeight(640) {

        initializeClassNames();

        // Load the ONNX model
        try {
            net = cv::dnn::readNetFromONNX(modelPath);

            // Set backend and target
            if (cv::cuda::getCudaEnabledDeviceCount() > 0) {
                std::cout << "Using CUDA backend" << std::endl;
                net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
                net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
            }
            else {
                std::cout << "Using CPU backend" << std::endl;
                net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
                net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
            }

            std::cout << "Model loaded successfully!" << std::endl;
        }
        catch (const cv::Exception& e) {
            std::cerr << "Error loading model: " << e.what() << std::endl;
            throw;
        }
    }

    cv::Mat preprocess(const cv::Mat& image) {
        cv::Mat blob;
        cv::dnn::blobFromImage(image, blob, 1.0 / 255.0, cv::Size(inputWidth, inputHeight), cv::Scalar(), true, false, CV_32F);
        return blob;
    }

    std::vector<cv::Rect> postprocess(const cv::Mat& image, const std::vector<cv::Mat>& outputs,
        std::vector<int>& classIds, std::vector<float>& confidences) {
        std::vector<cv::Rect> boxes;
        classIds.clear();
        confidences.clear();

        float xFactor = (float)image.cols / inputWidth;
        float yFactor = (float)image.rows / inputHeight;

        const cv::Mat& output = outputs[0];

        // YOLOv8 output format: [1, 84, 8400]
        // 84 = 4 (bbox coordinates) + 80 (class probabilities)
        // 8400 = number of anchor points

        if (output.dims != 3) {
            std::cerr << "Unexpected output dimensions: " << output.dims << std::endl;
            return boxes;
        }

        const int numClasses = 80;
        const int numDetections = output.size[2];  // 8400
        const int outputRows = output.size[1];     // 84

        // Process each detection
        for (int i = 0; i < numDetections; ++i) {
            // Get pointer to detection data for detection i
            const float* detection = output.ptr<float>(0, 0) + i;

            // Extract bounding box coordinates (first 4 values)
            float centerX = detection[0 * numDetections];
            float centerY = detection[1 * numDetections];
            float width = detection[2 * numDetections];
            float height = detection[3 * numDetections];

            // Find class with maximum confidence (indices 4-83)
            float maxConf = 0;
            int classId = 0;
            for (int j = 0; j < numClasses; ++j) {
                float confidence = detection[(4 + j) * numDetections];
                if (confidence > maxConf) {
                    maxConf = confidence;
                    classId = j;
                }
            }

            // Filter by confidence threshold
            if (maxConf >= confThreshold) {
                // Convert from center coordinates to top-left coordinates
                int left = static_cast<int>((centerX - width / 2) * xFactor);
                int top = static_cast<int>((centerY - height / 2) * yFactor);
                int boxWidth = static_cast<int>(width * xFactor);
                int boxHeight = static_cast<int>(height * yFactor);

                // Ensure coordinates are within image bounds
                left = std::max(0, std::min(left, image.cols - 1));
                top = std::max(0, std::min(top, image.rows - 1));
                boxWidth = std::min(boxWidth, image.cols - left);
                boxHeight = std::min(boxHeight, image.rows - top);

                if (boxWidth > 0 && boxHeight > 0) {
                    boxes.emplace_back(left, top, boxWidth, boxHeight);
                    classIds.push_back(classId);
                    confidences.push_back(maxConf);
                }
            }
        }

        return boxes;
    }

    void drawDetections(cv::Mat& image, const std::vector<cv::Rect>& boxes,
        const std::vector<int>& classIds, const std::vector<float>& confidences,
        const std::vector<int>& indices) {
        for (int i : indices) {
            const cv::Rect& box = boxes[i];
            int classId = classIds[i];
            float confidence = confidences[i];

            // Draw bounding box
            cv::rectangle(image, box, cv::Scalar(255, 0, 255), 3);

            // Prepare label
            std::string label = classNames[classId] + ": " + std::to_string(static_cast<int>(confidence * 100)) + "%";

            // Get label size for background rectangle
            int baseline;
            cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 1, 2, &baseline);

            // Draw background rectangle for text
            cv::Point labelPos(box.x, box.y - labelSize.height - baseline);
            if (labelPos.y < 0) labelPos.y = labelSize.height;

            cv::rectangle(image, labelPos, cv::Point(labelPos.x + labelSize.width, labelPos.y + labelSize.height + baseline),
                cv::Scalar(255, 0, 0), cv::FILLED);

            // Draw label text
            cv::putText(image, label, cv::Point(labelPos.x, labelPos.y + labelSize.height),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);

            // Print detection info
            std::cout << "Detection: " << classNames[classId]
                << " - Confidence: " << std::fixed << std::setprecision(2) << confidence << std::endl;
        }
    }

    void detect(cv::Mat& image) {
        // Preprocess
        cv::Mat blob = preprocess(image);
        net.setInput(blob);

        // Run inference
        std::vector<cv::Mat> outputs;
        net.forward(outputs, net.getUnconnectedOutLayersNames());

        // Postprocess
        std::vector<int> classIds;
        std::vector<float> confidences;
        std::vector<cv::Rect> boxes = postprocess(image, outputs, classIds, confidences);

        // Apply Non-Maximum Suppression
        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);

        // Draw detections
        drawDetections(image, boxes, classIds, confidences, indices);
    }
};

int main(int argc, char* argv[]) {
    std::string modelPath = "yolov8n.onnx";  // Default model path

    // Parse command line arguments
    if (argc > 1) {
        modelPath = argv[1];
    }

    try {
        // Initialize detector
        YOLOv8Detector detector(modelPath);

        // Initialize camera
        cv::VideoCapture cap(0);
        if (!cap.isOpened()) {
            std::cerr << "Error: Cannot open camera" << std::endl;
            return -1;
        }

        // Set camera properties
        cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

        std::cout << "Camera initialized. Press 'q' to quit." << std::endl;

        cv::Mat frame;
        while (true) {
            // Capture frame
            bool success = cap.read(frame);
            if (!success) {
                std::cerr << "Error: Cannot read frame from camera" << std::endl;
                break;
            }

            // Perform detection
            auto start = std::chrono::high_resolution_clock::now();
            detector.detect(frame);
            auto end = std::chrono::high_resolution_clock::now();

            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

            // Display FPS
            std::string fpsText = "FPS: " + std::to_string(1000.0 / duration.count());
            cv::putText(frame, fpsText, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);

            // Display result
            cv::imshow("YOLOv8 Detection", frame);

            // Check for quit key
            char key = cv::waitKey(1) & 0xFF;
            if (key == 'q' || key == 27) { // 'q' or ESC key
                break;
            }
        }

        cap.release();
        cv::destroyAllWindows();

    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}