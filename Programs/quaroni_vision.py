import cv2
import numpy as np

def adjust_camera_settings(cap, contrast=0.5, brightness=0.5, exposure=-4, saturation=0.5, gain=0):
    cap.set(cv2.CAP_PROP_CONTRAST, contrast)
    cap.set(cv2.CAP_PROP_BRIGHTNESS, brightness)
    cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
    cap.set(cv2.CAP_PROP_SATURATION, saturation)
    cap.set(cv2.CAP_PROP_GAIN, gain)

def get_layout(frame):
  '''
  Codigo para obtener el layout de nuestro espacio de trabajo
  Número de objetos y sus características
  Acomodados de izquierda a derecha
  
  n: int -> número de objetos
  c: lista de strings con características
  pos: lista de tuplas de posiciones
  '''
  # Convert to HSV
  hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

  # Define HSV ranges for red, green, and blue
  color_ranges = {
    'red': [([0, 120, 70], [10, 255, 255]), ([170, 120, 70], [180, 255, 255])],
  #  'green': [([36, 25, 25], [86, 255, 255])],
  #  'blue': [([94, 80, 2], [126, 255, 255])]
  }

  object_positions = []
  object_colors = []

  for color, ranges in color_ranges.items():
      mask = None
      for lower, upper in ranges:
          lower = np.array(lower)
          upper = np.array(upper)
          current_mask = cv2.inRange(hsv, lower, upper)
          mask = current_mask if mask is None else cv2.bitwise_or(mask, current_mask)

      # Find contours in the mask
      contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
      for cnt in contours:
          M = cv2.moments(cnt)
          if M["m00"] != 0:
              cx = int(M["m10"] / M["m00"])
              cy = int(M["m01"] / M["m00"])
              object_positions.append((cx, cy))
              object_colors.append(color)

              cv2.circle(frame, (cx, cy), 8, (0, 0, 255), -1)

  # Sort by x-coordinate (left to right)
  sorted_objects = sorted(zip(object_positions, object_colors), key=lambda x: x[0][0])
  positions_sorted, colors_sorted = zip(*sorted_objects) if sorted_objects else ([], [])

  """cv2.imshow("Detected Objects", frame)
  cv2.waitKey(0)  # Wait until a key is pressed
  cv2.destroyAllWindows()"""

  return frame, list(positions_sorted), list(colors_sorted)


def live_camera_with_key_capture(cam_index=0):
    cap = cv2.VideoCapture(cam_index)
    if not cap.isOpened():
        print(f"Error: Could not open camera {cam_index}")
        return

    print("Press SPACE to capture and detect. Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to read frame.")
            break

        # Show live feed
        cv2.imshow("Live Camera", frame)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q') or key == 27:  # q or Esc to quit
            break
        elif key == ord('c'):
            camera_cal(frame)
        elif key == 32:  # Spacebar to capture and detect
            '''processed_frame, circles = detect_blue_circles_in_frame(frame.copy())
            print("Blue Circles:", circles)
            cv2.imshow("Detected Blue Circles", processed_frame)'''
            
            processed_frame, positions, colors = get_layout(frame.copy())
            print("Positions:", positions)
            print("Colors:", colors)

            # Show detection result
            cv2.imshow("Detected Objects", processed_frame)
            cv2.waitKey(0)
            cv2.destroyWindow("Detected Objects")

    cap.release()
    cv2.destroyAllWindows()


def translate_to_mm(pos):
    pos1 = (319,408)
    pos2 = (325,151)
    dif = pos1[1]-pos2[1]
    mm = 190/dif
    pos_mm = (pos[1]*mm, (pos[0]-1280/2)*mm)
    return pos_mm

def camera_cal(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Invert image: black areas become white for thresholding
    inverted = 255 - gray

    # Threshold the image: only keep black/dark areas
    _, thresh = cv2.threshold(inverted, 60, 255, cv2.THRESH_BINARY)

    # Find contours
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Filter contours for squares
    squares = []
    for cnt in contours:
        # Approximate shape
        approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
        area = cv2.contourArea(cnt)

        if len(approx) == 4 and area > 100:  # 4 sides and not too small
            # Optionally check for near-square shape
            x, y, w, h = cv2.boundingRect(approx)
            aspect_ratio = float(w) / h
            if 0.9 <= aspect_ratio <= 1.1:
                squares.append(approx)
                cv2.drawContours(img, [approx], 0, (0, 255, 0), 2)
                cx, cy = x + w // 2, y + h // 2
                print(f"Detected black square at: center=({cx}, {cy}), size={w}x{h}")

    # Show result
    cv2.imshow("Detected Black Squares", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()



if __name__ == "__main__":
    live_camera_with_key_capture()