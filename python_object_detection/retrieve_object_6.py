import time
import math
import cv2
import numpy as np
import pyrealsense2 as rs
import serial
from ultralytics import YOLO  # YOLOv8

# object IDs
object_ID = 47 # apple in this case, number can be changed to grab other objects
human_ID = 0


camera_angle = 20 # camera angled down 20 degrees

# everything in px
frame_width = 640
frame_center = frame_width // 2
align_error = 10

serial_port = '/dev/ttyACM0'
baud_rate = 115200

# commands to send to arduino
def turn(ser, direction: str, secs: float):
    mes = f"turn {direction} {secs}\n"
    ser.write(mes.encode('utf-8'))
    while True:
        line = ser.readline().decode('utf-8').strip()
        if line == "Done":
            return


def forward(ser, distance_m: float):
    mes = f"travel forward {distance_m:.2f}\n"
    ser.write(mes.encode('utf-8'))
    while True:
        line = ser.readline().decode('utf-8').strip()
        if line == "Done":
            return
        
def close_CR(ser):
    ser.write("spin \n".encode('utf-8'))
    while True:
        line = ser.readline().decode('utf-8').strip()
        if line == "Done":
            return

def open_CR(ser):
    ser.write("spinback \n".encode('utf-8'))
    while True:
        line = ser.readline().decode('utf-8').strip()
        if line == "Done":
            return

def stop_CR(ser):
    ser.write("stop\n".encode('utf-8'))
    while True:
        line = ser.readline().decode('utf-8').strip()
        if line == "Done":
            return

def set_MG(ser, degs: int):
    mes = (f"mg996r {degs}\n")
    ser.write(mes.encode('utf-8'))
    while True:
        line = ser.readline().decode('utf-8').strip()
        if line == "Done":
            return


# start communication with arduino
try:
    ser = serial.Serial(serial_port, baud_rate, timeout=0.1)
    time.sleep(2)
    ser.reset_input_buffer()
except serial.SerialException as e:
    print(f"serial broke: {e}")
    exit(1)

# object detection
model = YOLO('yolov8n.pt')
model.fuse()
model.to('cuda') # use orin nano's gpu

# start realsense
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, frame_width, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, frame_width, 480, rs.format.bgr8, 30)
align = rs.align(rs.stream.color)
pipeline.start(config)

# object detection and approach
has_object = False
first = 0 # first time finding an object
p1_done = False # approach part 1

try:
    while True:
        # grab frames
        frames = pipeline.wait_for_frames()
        aligned = align.process(frames)
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()
        if not color_frame or not depth_frame:
            continue

        img = np.asanyarray(color_frame.get_data())
        results = model(img)[0]

        target_ID = human_ID if has_object else object_ID

        found = False
        target_x = None
        target_depth = 0.0

        # go through bounding boxes
        for box in results.boxes:
            if int(box.cls[0].item()) != target_ID:
                continue
            temp = box.xyxy[0].tolist()
            x1 = int(temp[0])
            y1 = int(temp[1])
            x2 = int(temp[2])
            y2 = int(temp[3])
            cx = (x1 + x2) // 2 # horizontal center of object bounding box
            cy = (y1 + y2) // 2 # vertical center
            depth = depth_frame.get_distance(cx, cy) # distance to object center
            found = True
            target_x = cx
            target_depth = depth
            break

        if not found:
            if first > 0: # prevent robot from easily "losing" an object
                first -= 1
            else:
                turn(ser, "right", 0.2)  # sweep for object
        else:
            first = 3
            error = target_x - frame_center # how far bounding box center is from frame center

            # check if centered
            if abs(error) <= align_error:
                # find horizontal ground distance
                horiz_dist = target_depth * math.cos(math.radians(camera_angle))

                stop_offset = 0.20 if has_object else 0.07
                first_stop = 0.25
                if not has_object:
                    if not p1_done:
                        first_leg = max(0.0, horiz_dist - first_stop)
                        if first_leg > 0.0:
                            forward(ser, first_leg)
                            p1_done = True
                        else:
                            p1_done = True
                    else:
                        remaining = max(0.0, horiz_dist - stop_offset)
                        if remaining > 0.0:
                            forward(ser, remaining)
                        #grab the object
                        time.sleep(1)
                        close_CR(ser) #close the claw
                        time.sleep(2)
                        set_MG(ser, 120) # raise the arm
                        has_object = True
                        first = 0
                        p1_done = False

                else:
                    # search/bring object to human
                    remaining = max(0.0, horiz_dist - stop_offset)
                    if remaining > 0.0:
                        forward(ser, remaining)

                    # drop object to human
                    stop_CR(ser)
                    set_MG(ser, 100) # lower arm
                    open_CR(ser) # open claw
                    time.sleep(2)
                    stop_CR(ser)
                    break

            else:
                # if object is within the frame, make tiny turns to center the robot on it
                direction = "right" if error > 0 else "left"
                mag = abs(error)
                secs = 0.1
                if mag <= 30:
                    secs = 0.01
                elif mag <= 300:
                    secs = 0.05
                turn(ser, direction, secs)

        time.sleep(0.2) # wait between turns

finally:
    pipeline.stop()
    ser.close()