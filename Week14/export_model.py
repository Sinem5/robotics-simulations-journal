from ultralytics import YOLO
print("Loading model...")
model = YOLO('/home/sinem/new_ws/yolo11n.pt')
print("Exporting to TensorRT (.engine)...")
model.export(format='engine', imgsz=640)
print("Export completed.")