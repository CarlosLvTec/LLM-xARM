import cv2
import numpy as np

# --- Setup camera ---
cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

# --- Define the color range to highlight ---
# You can change this to green, blue, etc.
#RED
low_color = np.array([0, 120, 70])   # Red range in HSV
high_color = np.array([180, 255, 255])
#BLUE
'''low_color = np.array([94, 80, 2])   # Blue range in HSV
high_color = np.array([126, 255, 255])'''

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create mask for selected color
    mask = cv2.inRange(hsv, low_color, high_color)
    mask_inv = cv2.bitwise_not(mask)

    # Convert entire image to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray_bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    # Extract color region
    color_region = cv2.bitwise_and(frame, frame, mask=mask)

    # Extract grayscale region (everything else)
    background = cv2.bitwise_and(gray_bgr, gray_bgr, mask=mask_inv)

    # Combine both parts
    highlighted = cv2.add(color_region, background)

    cv2.imshow("Highlighted Color Only", highlighted)

    key = cv2.waitKey(1)
    if key == 27:  # ESC
        break

cap.release()
cv2.destroyAllWindows()
