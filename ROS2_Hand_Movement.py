import cv2
import mediapipe as mp
import numpy as np
import time
import subprocess

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

cap = cv2.VideoCapture("http://192.168.0.112:8080/video") 

gesture_label = ""
gesture_label_time = 0
DISPLAY_DURATION = 1.0
HOLD_DURATION = 1.0  # in seconds

# Cooldowns
last_forward_time = 0
last_backward_time = 0
COOLDOWN = 3.3

forward_start_time = None
backward_start_time = None



def is_finger_strictly_extended(lm, tip_id, pip_id):
    tip = lm[tip_id]
    pip = lm[pip_id]
    return tip.y < pip.y

def all_fingers_extended_strict(lm):
    return (
        is_finger_strictly_extended(lm, 4, 3) and
        is_finger_strictly_extended(lm, 8, 6) and
        is_finger_strictly_extended(lm, 12, 10) and
        is_finger_strictly_extended(lm, 16, 14) and
        is_finger_strictly_extended(lm, 20, 18)
    )

def only_index_extended_strict(lm):
    return (
        is_finger_strictly_extended(lm, 8, 6) and
        not is_finger_strictly_extended(lm, 12, 10) and
        not is_finger_strictly_extended(lm, 16, 14) and
        not is_finger_strictly_extended(lm, 20, 18)
    )

def get_index_direction(lm):
    tip = np.array([lm[8].x, lm[8].y, lm[8].z])
    mcp = np.array([lm[5].x, lm[5].y, lm[5].z])
    direction = tip - mcp
    norm = np.linalg.norm(direction)
    if norm == 0:
        return None
    return direction / norm

with mp_hands.Hands(min_detection_confidence=0.8, min_tracking_confidence=0.8) as hands:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = hands.process(rgb)
        current_time = time.time()

        if result.multi_hand_landmarks:
            for hand_landmarks in result.multi_hand_landmarks:
                lm = hand_landmarks.landmark
                mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                # MOVE FORWARD: Open hand 
                if all_fingers_extended_strict(lm):
                    if forward_start_time is None:
                        forward_start_time = current_time
                    elif (current_time - forward_start_time >= HOLD_DURATION and
                          current_time - last_forward_time >= COOLDOWN):
                        gesture_label = "Move Forward"
                        gesture_label_time = current_time
                        last_forward_time = current_time
                        forward_start_time = None
                        subprocess.Popen(
                            'ros2 topic pub -1 /drone_command/distance std_msgs/msg/Float32 "data: 4.0"',
                            shell=True
                        )
                else:
                    forward_start_time = None

                # MOVE BACKWARD: Index 
                if only_index_extended_strict(lm):
                    direction = get_index_direction(lm)
                    if direction is not None and -direction[1] > 0.75: 
                        if backward_start_time is None:
                            backward_start_time = current_time
                        elif (current_time - backward_start_time >= HOLD_DURATION and
                              current_time - last_backward_time >= COOLDOWN):
                            gesture_label = "Move Backward"
                            gesture_label_time = current_time
                            last_backward_time = current_time
                            backward_start_time = None
                            subprocess.Popen(
                                'ros2 topic pub -1 /drone_command/distance std_msgs/msg/Float32 "data: -4.0"',
                                shell=True
                            )
                    else:
                        backward_start_time = None
                else:
                    backward_start_time = None

        # Display label
        if current_time - gesture_label_time < DISPLAY_DURATION:
            cv2.putText(frame, f"Gesture: {gesture_label}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2, cv2.LINE_AA)

        cv2.imshow("Hand Gesture Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()