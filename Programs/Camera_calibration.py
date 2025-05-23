import cv2
import numpy as np

# Checkerboard dimensions (number of INNER corners per row and column)
CHECKERBOARD = (22, 17)  # You must adjust this to match your pattern

# Real-world square size in cm
SQUARE_SIZE = 2.0

# Prepare object points (3D points in real-world space)
# For example, (0,0,0), (2,0,0), ..., assuming flat Z=0
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

# Arrays to store 3D points (real world) and 2D points (image plane)
objpoints = []  # 3D point in real world
imgpoints = []  # 2D point in image plane

# Open webcam
cap = cv2.VideoCapture(0)

# Collect calibration data
print("Show the checkerboard pattern to the webcam.")
collected = 0
MAX_SAMPLES = 20

while collected < MAX_SAMPLES:
    ret, frame = cap.read()
    if not ret:
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Try to find the checkerboard corners
    ret_corners, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret_corners:
        # Refine corner location for better accuracy
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1),
                                    criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        imgpoints.append(corners2)
        objpoints.append(objp)
        collected += 1
        print(f"Collected {collected}/{MAX_SAMPLES}")
        cv2.drawChessboardCorners(frame, CHECKERBOARD, corners2, ret_corners)

    # Show current frame
    cv2.imshow('Calibration Capture', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyWindow('Calibration Capture')

# Perform calibration
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

print("Calibration complete.")
print("Camera matrix:\n", camera_matrix)
print("Distortion coefficients:\n", dist_coeffs)

# Start real-time undistortion
print("Press 'q' to quit live feed.")
while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Undistort the frame using calibration results
    undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs)

    # Show side-by-side comparison
    combined = np.hstack((frame, undistorted))
    cv2.imshow('Original (Left) vs Undistorted (Right)', combined)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
