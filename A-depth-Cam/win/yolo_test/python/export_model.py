from ultralytics import YOLO

model = YOLO("yolov8n.pt")

# Export the model to ONNX format
model.export(format="onnx")