import cv2
import numpy as np
from ultralytics import YOLO

# Define checkerboard dimensions (number of internal corners)
CHECKERBOARD = (6, 9)  # 6 rows x 9 columns of internal corners
SQUARE_SIZE = 0.02  # 2 cm in meters

# Termination criteria for corner refinement
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Create object points (3D real-world coordinates)
objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[1], 0:CHECKERBOARD[0]].T.reshape(-1, 2)
objp *= SQUARE_SIZE  # Scale by 2 cm

# Arrays to store object points and image points
objpoints = []  # 3D point in real-world
imgpoints = []  # 2D points in image plane

# Initialize camera
cap = cv2.VideoCapture(0)

# Camera calibration placeholders
calibrated = False
camera_matrix = None
dist_coeffs = None

# Run until calibration and detection
while True:
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Find the checkerboard corners
    ret_corners, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret_corners:
        objpoints.append(objp)
        imgpoints.append(corners)

        # Refine the corner locations
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)

        # Draw corners
        cv2.drawChessboardCorners(frame, CHECKERBOARD, corners2, ret_corners)

        # If we collected enough views, calibrate the camera
        if len(objpoints) >= 10 and not calibrated:
            ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
                objpoints, imgpoints, gray.shape[::-1], None, None)
            calibrated = True
            print("Camera calibrated.")
            print("Camera matrix:\n", camera_matrix)
            print("Distortion coefficients:\n", dist_coeffs)

    # Show the frame
    cv2.imshow('Checkerboard Detection', frame)

    key = cv2.waitKey(1)
    if key == 27:  # ESC to exit
        break

cap.release()
cv2.destroyAllWindows()
