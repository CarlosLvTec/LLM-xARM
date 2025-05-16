"""
Robot Arm + Color Detection with Button GUI (v2)
--------------------------------------------------
Features:
- GUI buttons in the OpenCV window to select which color to detect.
- Press 'q' to pick and return object to initial position.
- Press 'w', 'a', 's', 'd' to move object up, left, down, or right after picking.
- Uses corner calibration to convert screen coordinates to real-world positions.
- NEW: Adjust camera parameters like contrast, brightness, exposure, etc.
"""

import cv2
import numpy as np
from xarm.wrapper import XArmAPI
import time

# Initial robot setup
arm = XArmAPI('192.168.1.173', baud_checkset=False)
arm.motion_enable(True)
arm.set_mode(0)
arm.set_state(0)
time.sleep(1)

# Initial pick-up home position
INIT_POS = [100, 0, 160]

# Color HSV ranges
detected_color = 'red'
COLOR_RANGES = {
    'red': [([0, 120, 70], [10, 255, 255]), ([170, 120, 70], [180, 255, 255])],
    'green': [([36, 25, 25], [86, 255, 255])],
    'blue': [([94, 80, 2], [126, 255, 255])],
    'yellow': [([20, 100, 100], [30, 255, 255])],
    'cyan': [([78, 100, 100], [100, 255, 255])],
    'magenta': [([125, 100, 100], [150, 255, 255])],
    'orange': [([10, 100, 20], [25, 255, 255])]
}

# Screen corners mapped to real world coordinates
corner_map = {
    (629, 464): [324.8, 225.3, 200],
    (622, 15): [10.5, 220.4, 200],
    (13, 17): [11.9, -205.9, 200],
    (16, 467): [326.9, -203.8, 200]
}

# Interpolating camera pixel to real world space
def pixel_to_world(px, py):
    top_mid = np.mean([corner_map[(622, 15)], corner_map[(13, 17)]], axis=0)
    bottom_mid = np.mean([corner_map[(629, 464)], corner_map[(16, 467)]], axis=0)
    left_mid = np.mean([corner_map[(622, 15)], corner_map[(629, 464)]], axis=0)
    right_mid = np.mean([corner_map[(13, 17)], corner_map[(16, 467)]], axis=0)
    width = 640
    height = 480
    x_world = np.interp(px, [0, width], [left_mid[0], right_mid[0]])
    y_world = np.interp(py, [0, height], [top_mid[1], bottom_mid[1]])
    return [x_world, y_world, 200]

# GUI Button area definitions (x1, y1, x2, y2)
buttons = {
    'red': (10, 10, 70, 40),
    'green': (80, 10, 140, 40),
    'blue': (150, 10, 210, 40),
    'yellow': (220, 10, 280, 40),
    'cyan': (290, 10, 350, 40),
    'magenta': (360, 10, 430, 40),
    'orange': (440, 10, 510, 40),
}

# Move robot to initial home position
def go_home():
    arm.set_position(*INIT_POS, wait=True)

def pick_and_place(center):
    world = pixel_to_world(center[0], center[1])
    above = world.copy()
    above[2] = 200
    down = world.copy()
    down[2] = 87
    arm.set_position(*above, wait=True)
    arm.set_position(*down, wait=True)
    arm.set_suction_cup(True, wait=False)
    time.sleep(0.5)
    arm.set_position(*above, wait=True)
    return world

def place(world):
    above = world.copy()
    above[2] = 200
    down = world.copy()
    down[2] = 85
    arm.set_position(*above, wait=True)
    arm.set_position(*down, wait=True)
    arm.set_suction_cup(False, wait=False)
    time.sleep(0.5)
    arm.set_position(*above, wait=True)

def offset_place(base, direction):
    offsets = {
        'w': (0, 30),
        's': (0, -30),
        'a': (-30, 0),
        'd': (30, 0)
    }
    dx, dy = offsets[direction]
    target = base.copy()
    target[0] += dx
    target[1] += dy
    place(target)

# Adjust camera settings
def adjust_camera_settings(cap, contrast=0.5, brightness=0.5, exposure=-4, saturation=0.5, gain=0):
    cap.set(cv2.CAP_PROP_CONTRAST, contrast)
    cap.set(cv2.CAP_PROP_BRIGHTNESS, brightness)
    cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
    cap.set(cv2.CAP_PROP_SATURATION, saturation)
    cap.set(cv2.CAP_PROP_GAIN, gain)

# GUI callback for mouse click
def mouse_callback(event, x, y, flags, param):
    global detected_color
    if event == cv2.EVENT_LBUTTONDOWN:
        for name, (x1, y1, x2, y2) in buttons.items():
            if x1 <= x <= x2 and y1 <= y <= y2:
                detected_color = name
                print(f"Selected color: {detected_color}")

# Main loop
cap = cv2.VideoCapture(3)
adjust_camera_settings(cap)
cv2.namedWindow("Color Detection")
cv2.setMouseCallback("Color Detection", mouse_callback)
go_home()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    masks = [cv2.inRange(hsv, np.array(rng[0]), np.array(rng[1])) for rng in COLOR_RANGES[detected_color]]
    mask = masks[0]
    for extra in masks[1:]:
        mask += extra

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    largest = None
    largest_area = 0

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 500 and area > largest_area:
            largest = cnt
            largest_area = area

    center = None
    if largest is not None:
        x, y, w, h = cv2.boundingRect(largest)
        center = (int(x + w / 2), int(y + h / 2))
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(frame, center, 5, (255, 0, 0), -1)
        cv2.putText(frame, f"{center}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    for name, (x1, y1, x2, y2) in buttons.items():
        color = (0, 255, 255) if name == detected_color else (50, 50, 50)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, -1)
        cv2.putText(frame, name, (x1 + 5, y2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

    cv2.imshow("Color Detection", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q') and center:
        obj_pos = pick_and_place(center)
        place(INIT_POS)
    elif key in [ord('w'), ord('a'), ord('s'), ord('d')] and center:
        obj_pos = pick_and_place(center)
        offset_place(obj_pos, chr(key))
    elif key == 27:
        break

cap.release()
cv2.destroyAllWindows()
go_home()
arm.disconnect()
