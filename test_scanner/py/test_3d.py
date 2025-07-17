import cv2
import numpy as np
import open3d as o3d

# Camera intrinsics (replace with your actual values from Deptrum Scope "Device Info" or calibration)
# Example for 640x480 resolution; get accurate fx/fy/cx/cy from your camera's specs
width, height = 640, 480  # Adjust to your camera's resolution (check Deptrum Scope settings)
fx, fy = 525.0, 525.0     # Focal lengths
cx, cy = width / 2 - 0.5, height / 2 - 0.5  # Principal points (centered)
intrinsic = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)

# Open the camera (try index 0, 1, etc.; Deptrum might be on a higher index if other cams are connected)
cap = cv2.VideoCapture(0)  # Change to 1, 2, etc., if needed

# Set camera properties if possible (e.g., resolution; depends on driver)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture frame. Check camera connection/index.")
        break

    # Print frame shape for debugging
    print(f"Captured frame shape: {frame.shape}, dtype: {frame.dtype}")

    # Convert to grayscale (assuming camera outputs BGR/RGB; adjust if it's already depth)
    depth_raw = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Scale to float32 depth in meters (adjust divisor based on your camera's units, e.g., /1000 for mm to m)
    depth = depth_raw.astype(np.float32) / 1000.0

    # Reshape to [H, W, 1] for Open3D
    depth = np.expand_dims(depth, axis=2)  # Now [H, W, 1]

    # Create a 3-channel placeholder color image (black; replace with actual RGB if available)
    # If your camera provides RGB, use cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) instead
    color = np.zeros((height, width, 3), dtype=np.uint8)  # Shape [H, W, 3], uint8

    # Create Open3D images
    color_image = o3d.geometry.Image(color)
    depth_image = o3d.geometry.Image(depth)

    # Create RGBD image
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_image,
        depth_image,
        depth_scale=1.0,  # Adjust if needed (already scaled to meters)
        depth_trunc=4.0,  # Truncate depths >4m to avoid noise
        convert_rgb_to_intensity=False
    )

    # Generate point cloud
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)

    # Visualize (flips it for correct orientation)
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    o3d.visualization.draw_geometries([pcd])

    # Optional: Save point cloud to file
    # o3d.io.write_point_cloud("output.ply", pcd)

    if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit
        break

cap.release()
cv2.destroyAllWindows()