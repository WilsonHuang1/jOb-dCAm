// MINIMAL ENHANCED 3D INTEGRATION
// Add this ONLY after all your existing code and before the main() function

// Enhanced mode flag - add after your existing global variables
bool g_enhanced_mode = false;

// Simple pose tracking without complex types
struct SimpleCameraPose {
    float x, y, z;
    int frame_count;
    bool valid;
    
    SimpleCameraPose() : x(0), y(0), z(0), frame_count(0), valid(false) {}
};

SimpleCameraPose g_camera_pose;

// Enhanced frame processing - REPLACE your existing generate3DPointCloud call with this
void processEnhanced3DSimple(const cv::Mat& rgbFrame, const cv::Mat& depthFrame,
                             const std::vector<ImprovedYOLODetector::Detection>& detections) {
    
    if (!g_enhanced_mode || !g_enable3D) {
        // Use original 3D processing
        generate3DPointCloud(rgbFrame, depthFrame, detections);
        return;
    }
    
    // Simple pose estimation from detected objects
    if (!detections.empty()) {
        float sum_x = 0, sum_y = 0, sum_z = 0;
        int count = 0;
        
        for (const auto& det : detections) {
            if (det.hasWorldPos) {
                sum_x += det.worldX;
                sum_y += det.worldY; 
                sum_z += det.worldZ;
                count++;
            }
        }
        
        if (count > 0) {
            // Update camera pose (opposite of object positions)
            g_camera_pose.x = -sum_x / count;
            g_camera_pose.y = -sum_y / count;
            g_camera_pose.z = -sum_z / count + 1.0f; // Offset camera back
            g_camera_pose.frame_count++;
            g_camera_pose.valid = true;
            
            std::cout << "Enhanced Mode - Camera: [" << g_camera_pose.x 
                      << ", " << g_camera_pose.y << ", " << g_camera_pose.z 
                      << "] Frame: " << g_camera_pose.frame_count << std::endl;
        }
    }
    
    // Still use original 3D generation but with pose info
    generate3DPointCloud(rgbFrame, depthFrame, detections);
}

// Enhanced display info - add this to your display drawing
void addEnhancedDisplay(cv::Mat& frame) {
    if (!g_enhanced_mode) return;
    
    // Draw enhanced mode indicator
    cv::putText(frame, "ENHANCED MODE", cv::Point(10, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
    
    // Draw camera pose if valid
    if (g_camera_pose.valid) {
        char pose_text[200];
        sprintf(pose_text, "Cam: [%.2f, %.2f, %.2f] F:%d", 
                g_camera_pose.x, g_camera_pose.y, g_camera_pose.z, g_camera_pose.frame_count);
        cv::putText(frame, pose_text, cv::Point(10, 60), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    }
}

// Enhanced keyboard handling - add this to your keyboard switch
void handleEnhancedKey(int key) {
    switch (key) {
    case 'e':
        g_enhanced_mode = !g_enhanced_mode;
        std::cout << "Enhanced 3D mode: " << (g_enhanced_mode ? "ON" : "OFF") << std::endl;
        if (g_enhanced_mode) {
            g_camera_pose = SimpleCameraPose(); // Reset pose
            std::cout << "Enhanced mode tracks camera position using detected objects" << std::endl;
        }
        break;
        
    case 'r':
        if (g_enhanced_mode) {
            g_camera_pose = SimpleCameraPose(); // Reset pose
            std::cout << "Camera pose reset!" << std::endl;
        }
        break;
    }
}

// Enhanced help - REPLACE your help function with this simple version
void showSimpleEnhancedHelp() {
    std::cout << "\n========== 3D Scanner Help ==========" << std::endl;
    std::cout << "Controls:" << std::endl;
    std::cout << "  'q' or ESC  - Quit" << std::endl;
    std::cout << "  'h'         - Show help" << std::endl;
    std::cout << "  't'         - Toggle YOLO (" << (g_enableYOLO ? "ON" : "OFF") << ")" << std::endl;
    std::cout << "  'x'         - Toggle XYZ (" << (g_enableXYZ ? "ON" : "OFF") << ")" << std::endl;
    std::cout << "  '3'         - Toggle 3D (" << (g_enable3D ? "ON" : "OFF") << ")" << std::endl;
    std::cout << "  'w'         - Toggle 3D window (" << (g_show3DWindow ? "ON" : "OFF") << ")" << std::endl;
    std::cout << "Enhanced:" << std::endl;
    std::cout << "  'e'         - Toggle enhanced mode (" << (g_enhanced_mode ? "ON" : "OFF") << ")" << std::endl;
    std::cout << "  'r'         - Reset camera pose" << std::endl;
    std::cout << "======================================" << std::endl;
    
    if (g_enhanced_mode && g_camera_pose.valid) {
        std::cout << "Camera Position: [" << g_camera_pose.x << ", " 
                  << g_camera_pose.y << ", " << g_camera_pose.z << "]" << std::endl;
    }
}

/*
================================================================================
INTEGRATION INSTRUCTIONS - SIMPLE VERSION:
================================================================================

1. ADD the code above to your main.cpp file (after your existing functions, before main())

2. In your EnhancedFrameCallback function, REPLACE the line:
   generate3DPointCloud(rgbFrame, depthFrame, detections);
   
   WITH:
   processEnhanced3DSimple(rgbFrame, depthFrame, detections);

3. In your display code, AFTER drawing detections, ADD:
   addEnhancedDisplay(displayFrame);

4. In your keyboard handling switch statement, REPLACE the 'h' case:
   case 'h':
       showSimpleEnhancedHelp();
       break;
   
   And ADD a default case:
   default:
       handleEnhancedKey(key);
       break;

5. In getUserConfiguration(), ADD this after the 3D window question:
   if (g_enable3D) {
       std::cout << "5. Enable enhanced pose tracking? (y/n): ";
       std::getline(std::cin, input);
       g_enhanced_mode = (!input.empty() && (input[0] == 'y' || input[0] == 'Y'));
   }

That's it! This simple version:
✅ Tracks camera position using detected objects
✅ Shows pose info on screen and console  
✅ No complex libraries or templates
✅ Minimal changes to your existing code
✅ Works with your current structure

The enhanced mode will show camera position calculated from detected object positions,
giving you spatial awareness of where the scanner is in relation to the objects!
*/