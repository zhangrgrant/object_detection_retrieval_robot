# Object Detection and Retrieval Robot

- Mobile robot that uses object detection and depth sensing to approach objects, pick them up, and bring them to a person.
- Uses GPU accelerated YOLOv8 on the Jetson Orin Nano for object detection, Intel RealSense depth camera for depth and color frame data, and Arduino for motor control.

## Repository Structure
- This repository is separated into python code for object detection and retrieval and Arduino code for controlling the robot's motors.

### arduino/robot_motor_control/
- Contains robot_motor_control.ino, the Arduino code that controls the robot’s motors, encoders, and servos.

### python_object_detection/
- Contains Python files for object detection, navigation, and decision making

- vision-test-3.py - tests object detection and camera centering on objects

- retrieve-object-6.py – Uses object detection to approach and retrieve an object through communication with the Arduino

### Hardware

- Intel RealSense depth camera
- Drivetrain with motors and encoders
- Arduino board for motor control
- Continuous rotation claw and MG996R servo arm

### Software

- Python 3.9+
- Arduino IDE
- OpenCV
- Ultralytics YOLOv8
- pyrealsense2
- PySerial
