
import cv2
import numpy as np
import math

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Failed to open camera")
    exit()

def detect_ball(mask, color_bgr, label, frame):
    # Remove noise
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        area = cv2.contourArea(cnt)

        if area < 800:
            continue

        perimeter = cv2.arcLength(cnt, True)
        if perimeter == 0:
            continue

        circularity = 4 * math.pi * area / (perimeter * perimeter)

        # Accept only round objects
        if circularity > 0.7:
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            center = (int(x), int(y))
            radius = int(radius)

            cv2.circle(frame, center, radius, color_bgr, 3)
            cv2.putText(frame, label,
                        (center[0] - 40, center[1] - radius - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, color_bgr, 2)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Yellow
    yellow_lower = np.array([20, 80, 80])
    yellow_upper = np.array([40, 255, 255])

    # Blue
    blue_lower = np.array([100, 100, 80])
    blue_upper = np.array([130, 255, 255])

    mask_yellow = cv2.inRange(hsv, yellow_lower, yellow_upper)
    mask_blue = cv2.inRange(hsv, blue_lower, blue_upper)

    detect_ball(mask_yellow, (0, 255, 255), "Yellow Ball", frame)
    detect_ball(mask_blue, (255, 0, 0), "Blue Ball", frame)

    cv2.imshow("Ball Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    if cv2.getWindowProperty("Ball Detection", cv2.WND_PROP_VISIBLE) < 1:
        break

cap.release()
cv2.destroyAllWindows()