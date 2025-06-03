# Drone Gesture Control

Control a drone using hand gestures with MediaPipe, ROS2, and PX4.

## Gestures

- **Open hand** - Move forward 4m
- **Index finger up** - Move backward 4m

## Requirements

```bash
pip install opencv-python mediapipe ultralytics torch numpy
sudo apt install ros-humble-px4-msgs
```

## Setup

1. Update camera IP in the Python files:
   ```python
   cap = cv2.VideoCapture("http://YOUR_IP:8080/video")
   ```

2. Start ROS2 controller:
   ```bash
   ros2 run your_package forward_flight_controller
   ```

3. Run gesture detection:
   ```bash
   python ROS2_Hand_Movement.py
   ```

## Usage

- Arm drone and set to HOLD mode
- Hold gestures for 1 second to trigger
- 3.3 second cooldown between commands

## Files

- `Object_Detection.py` - YOLO object detection
- `ROS2_Hand_Movement.py` - Hand gesture recognition
- `forward_flight_controller.py` - ROS2 flight controller

## Safety

Always test in open areas away from people and obstacles.