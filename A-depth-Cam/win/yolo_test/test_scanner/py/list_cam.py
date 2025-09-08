import cv2

def list_cameras(max_index=10):
    available = []
    for i in range(max_index):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            print(f"Index {i}: Opened (resolution: {width}x{height})")
            available.append(i)
            cap.release()
        else:
            print(f"Index {i}: Not available")
    return available

if __name__ == "__main__":
    list_cameras()