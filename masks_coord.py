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

# HSV ranges (подобрано под твою комнату)
yellow_lower = np.array([20, 130, 100])
yellow_upper = np.array([50, 255, 255])

# blue_lower = np.array([113, 134, 34])
# blue_upper = np.array([130, 255, 255])

blue_lower = np.array([100, 100, 50])  # H, S, V - более мягкий нижний порог
blue_upper = np.array([130, 255, 255]) # H, S, V - верхний порог

def detect_ball_hsv(frame, lower_hsv, upper_hsv, color_bgr, label):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    mask[hsv[:,:,2] > 240] = 0  # убрать пересвет

    # Морфология для снижения шума
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 300:  # мягкая граница площади
            continue
        
        perimeter = cv2.arcLength(cnt, True)
        if perimeter == 0:
            continue
        
        circularity = 4 * math.pi * area / (perimeter * perimeter)
        x, y, w, h = cv2.boundingRect(cnt)
        aspect_ratio = w / float(h)
        
        # Более мягкие фильтры
        if circularity > 0.6 and 0.7 < aspect_ratio < 1.3:
            (cx, cy), radius = cv2.minEnclosingCircle(cnt)
            center = (int(cx), int(cy))
            radius = int(radius)
            
            # Нарисовать круг
            cv2.circle(frame, center, radius, color_bgr, 3)
            
            # Нарисовать подпись
            cv2.putText(frame, label, (center[0] - 40, center[1] - radius - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_bgr, 2)
            
            # Нарисовать координаты
            coord_text = f"({center[0]}, {center[1]})"
            cv2.putText(frame, coord_text, (center[0] - 40, center[1] + radius + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_bgr, 2)

    return mask

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    mask_yellow = detect_ball_hsv(frame, yellow_lower, yellow_upper, (0, 255, 255), "Yellow Ball")
    mask_blue = detect_ball_hsv(frame, blue_lower, blue_upper, (255, 0, 0), "Blue Ball")
    
    # Отображение
    cv2.imshow("Ball Detection", frame)
    cv2.imshow("Mask Yellow", mask_yellow)
    cv2.imshow("Mask Blue", mask_blue)
    
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    
    if cv2.getWindowProperty("Ball Detection", cv2.WND_PROP_VISIBLE) < 1:
        break

cap.release()
cv2.destroyAllWindows()