import cv2
import numpy as np

'''def adjust_camera_settings(cap, contrast=0.5, brightness=0.5, exposure=-4, saturation=0.5, gain=0):
    cap.set(cv2.CAP_PROP_CONTRAST, contrast)
    cap.set(cv2.CAP_PROP_BRIGHTNESS, brightness)
    cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
    cap.set(cv2.CAP_PROP_SATURATION, saturation)
    cap.set(cv2.CAP_PROP_GAIN, gain)'''

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

  cv2.imshow("Detected Objects", frame)
  cv2.waitKey(0)  # Wait until a key is pressed
  cv2.destroyAllWindows()

  return frame, list(positions_sorted), list(colors_sorted)


def live_camera_with_key_capture(cam_index=2):
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
        elif key == 32:  # Spacebar to capture and detect
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
    pass


if __name__ == "__main__":
    live_camera_with_key_capture()