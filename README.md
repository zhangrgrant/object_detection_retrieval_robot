# Object Detection and Retrieval Robot

- Mobile robot that uses object detection and depth sensing to approach objects, pick them up, and bring them to a person.
- Uses GPU accelerated YOLOv8 for object detection, Intel RealSense depth camera for depth and color frame data, and Arduino for motor control.

## Hardware

- Intel RealSense depth camera
- Drivetrain with motors and encoders
- Arduino board for motor control
- Continuous rotation claw and MG996R servo arm

## Software

- Python 3.9+
- Arduino IDE
- OpenCV
- Ultralytics YOLOv8
- pyrealsense2
- PySerial