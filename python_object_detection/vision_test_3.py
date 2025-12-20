import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO

object_ID = 39 # bottle ID for testing, can be changed to other objects

frame_width = 640
frame_center = frame_width // 2
align_error = 10

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline_profile = pipeline.start(config)
align = rs.align(rs.stream.color)

# run yolo on CPU (default) for testing
model = YOLO('yolov8n.pt')
model.fuse()

try:
    while True:
        # grab a frame from the camera
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue
        img = np.asanyarray(color_frame.get_data())
        pos_objs = model(img)[0]
        for box in pos_objs.boxes:
            cls_ID = int(box.cls[0])
            temp = box.xyxy[0].tolist()
            x1 = int(temp[0])
            y1 = int(temp[1])
            x2 = int(temp[2])
            y2 = int(temp[3])
            cx = (x1 + x2) // 2 # horizontal center of object bounding box
            cy = (y1 + y2) // 2 # vertical center

            depth = depth_frame.get_distance(cx, cy)

            # vary box color based on detected/centered
            box_color = (0, 255, 0)
            if cls_ID == object_ID:
                error = cx - frame_center # how far off center
                if abs(error) <= align_error:
                    box_color = (255, 0, 255)
                else:
                    box_color = (255, 0, 0)
            cv2.rectangle (img, (x1, y1), (x2, y2), box_color, 2)
            cv2.putText(img, f"{model.names[cls_ID]} {depth:.2f} m", (x1, y1+10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)
        cv2.imshow('realsense feed', img)
        cv2.waitKey(1)
finally:
    pipeline.stop()