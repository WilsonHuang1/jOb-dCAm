import cv2
import numpy as np

# Open the camera (try index 0, 1, etc., if needed; or use Deptrum's device path)
cap = cv2.VideoCapture(1)  # Adjust index if multiple cameras

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Assuming your camera outputs depth as a grayscale image (adapt based on format)
    # Convert to depth map (e.g., normalize for visualization)
    depth_map = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Example; adjust for your cam
    depth_colormap = cv2.applyColorMap(depth_map, cv2.COLORMAP_JET)  # Colorize for better view

    cv2.imshow('Depth Map', depth_colormap)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit
        break

cap.release()
cv2.destroyAllWindows()