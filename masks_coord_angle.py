import cv2
import numpy as np
import math

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Failed to open camera")
    exit()

# Camera manual settings
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
cap.set(cv2.CAP_PROP_EXPOSURE, -6)
cap.set(cv2.CAP_PROP_GAIN, 0)
cap.set(cv2.CAP_PROP_BRIGHTNESS, 0)

# Camera field of view (примерные значения для обычной вебки)
FOV_X = 60  # горизонтальный угол камеры
FOV_Y = 45  # вертикальный угол камеры

# HSV ranges
yellow_lower = np.array([20, 130, 100])
yellow_upper = np.array([50, 255, 255])

blue_lower = np.array([100, 100, 50])
blue_upper = np.array([130, 255, 255])


def draw_crosshair(frame):
    h, w = frame.shape[:2]

    cx = w // 2
    cy = h // 2

    cv2.line(frame, (cx, 0), (cx, h), (0, 255, 0), 1)
    cv2.line(frame, (0, cy), (w, cy), (0, 255, 0), 1)

    cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)

    return cx, cy, w, h


def detect_ball_hsv(frame, lower_hsv, upper_hsv, color_bgr, label, frame_center, frame_size):

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

    mask[hsv[:, :, 2] > 240] = 0

    kernel = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cx_frame, cy_frame = frame_center
    width, height = frame_size

    for cnt in contours:

        area = cv2.contourArea(cnt)

        if area < 300:
            continue

        perimeter = cv2.arcLength(cnt, True)

        if perimeter == 0:
            continue

        circularity = 4 * math.pi * area / (perimeter * perimeter)

        x, y, w, h = cv2.boundingRect(cnt)

        aspect_ratio = w / float(h)

        if circularity > 0.6 and 0.7 < aspect_ratio < 1.3:

            (cx, cy), radius = cv2.minEnclosingCircle(cnt)

            center = (int(cx), int(cy))
            radius = int(radius)

            # Рисуем круг
            cv2.circle(frame, center, radius, color_bgr, 3)

            # Смещение относительно центра
            dx = center[0] - cx_frame
            dy = cy_frame - center[1]

            # Перевод в градусы
            angle_x = dx * (FOV_X / width)
            angle_y = dy * (FOV_Y / height)

            # Подпись шарика
            cv2.putText(frame, label,
                        (center[0] - 50, center[1] - radius - 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_bgr, 2)

            # Координаты в пикселях
            coord_text = f"px: ({dx}, {dy})"

            cv2.putText(frame, coord_text,
                        (center[0] - 60, center[1] - radius - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_bgr, 2)

            # Углы
            angle_text = f"deg: ({angle_x:.2f}, {angle_y:.2f})"

            cv2.putText(frame, angle_text,
                        (center[0] - 60, center[1] + radius + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_bgr, 2)

    return mask


while True:

    ret, frame = cap.read()

    if not ret:
        break

    # рисуем перекрестие
    cx, cy, width, height = draw_crosshair(frame)

    frame_center = (cx, cy)
    frame_size = (width, height)

    mask_yellow = detect_ball_hsv(
        frame,
        yellow_lower,
        yellow_upper,
        (0, 255, 255),
        "Yellow Ball",
        frame_center,
        frame_size
    )

    # mask_blue = detect_ball_hsv(
    #     frame,
    #     blue_lower,
    #     blue_upper,
    #     (255, 0, 0),
    #     "Blue Ball",
    #     frame_center,
    #     frame_size
    # )

    cv2.imshow("Ball Detection", frame)
    cv2.imshow("Mask Yellow", mask_yellow)
    # cv2.imshow("Mask Blue", mask_blue)

    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'):
        break

    if cv2.getWindowProperty("Ball Detection", cv2.WND_PROP_VISIBLE) < 1:
        break


cap.release()
cv2.destroyAllWindows()