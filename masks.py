



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

# Create a window for trackbars
cv2.namedWindow("HSV Trackbars")

def nothing(x):
    pass

# Yellow trackbars
cv2.createTrackbar("Y_H_L", "HSV Trackbars", 20, 179, nothing)
cv2.createTrackbar("Y_H_U", "HSV Trackbars", 35, 179, nothing)
cv2.createTrackbar("Y_S_L", "HSV Trackbars", 100, 255, nothing)
cv2.createTrackbar("Y_S_U", "HSV Trackbars", 255, 255, nothing)
cv2.createTrackbar("Y_V_L", "HSV Trackbars", 100, 255, nothing)
cv2.createTrackbar("Y_V_U", "HSV Trackbars", 255, 255, nothing)

# Blue trackbars
cv2.createTrackbar("B_H_L", "HSV Trackbars", 100, 179, nothing)
cv2.createTrackbar("B_H_U", "HSV Trackbars", 130, 179, nothing)
cv2.createTrackbar("B_S_L", "HSV Trackbars", 120, 255, nothing)
cv2.createTrackbar("B_S_U", "HSV Trackbars", 255, 255, nothing)
cv2.createTrackbar("B_V_L", "HSV Trackbars", 80, 255, nothing)
cv2.createTrackbar("B_V_U", "HSV Trackbars", 255, 255, nothing)

# Function to detect balls
def detect_ball_hsv(frame, lower_hsv, upper_hsv, color_bgr, label):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    
    mask[hsv[:,:,2] > 240] = 0  # remove overexposed pixels
    
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 1500:
            continue
        
        perimeter = cv2.arcLength(cnt, True)
        if perimeter == 0:
            continue
        
        circularity = 4 * math.pi * area / (perimeter*perimeter)
        x, y, w, h = cv2.boundingRect(cnt)
        aspect_ratio = w / float(h)
        
        if circularity > 0.8 and 0.85 < aspect_ratio < 1.15:
            (cx, cy), radius = cv2.minEnclosingCircle(cnt)
            center = (int(cx), int(cy))
            radius = int(radius)
            
            cv2.circle(frame, center, radius, color_bgr, 3)
            cv2.putText(frame, label,
                        (center[0] - 40, center[1] - radius - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, color_bgr, 2)
    return mask

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    # # Get current trackbar positions
    # y_h_l = cv2.getTrackbarPos("Y_H_L", "HSV Trackbars")
    # y_h_u = cv2.getTrackbarPos("Y_H_U", "HSV Trackbars")
    # y_s_l = cv2.getTrackbarPos("Y_S_L", "HSV Trackbars")
    # y_s_u = cv2.getTrackbarPos("Y_S_U", "HSV Trackbars")
    # y_v_l = cv2.getTrackbarPos("Y_V_L", "HSV Trackbars")
    # y_v_u = cv2.getTrackbarPos("Y_V_U", "HSV Trackbars")
    
    # b_h_l = cv2.getTrackbarPos("B_H_L", "HSV Trackbars")
    # b_h_u = cv2.getTrackbarPos("B_H_U", "HSV Trackbars")
    # b_s_l = cv2.getTrackbarPos("B_S_L", "HSV Trackbars")
    # b_s_u = cv2.getTrackbarPos("B_S_U", "HSV Trackbars")
    # b_v_l = cv2.getTrackbarPos("B_V_L", "HSV Trackbars")
    # b_v_u = cv2.getTrackbarPos("B_V_U", "HSV Trackbars")
    
    # # Define HSV ranges from trackbars
    # yellow_lower = np.array([y_h_l, y_s_l, y_v_l])
    # yellow_upper = np.array([y_h_u, y_s_u, y_v_u])
    
    # blue_lower = np.array([b_h_l, b_s_l, b_v_l])
    # blue_upper = np.array([b_h_u, b_s_u, b_v_u])

        # Define HSV ranges from trackbars
    yellow_lower = np.array([20, 130, 100])
    yellow_upper = np.array([50, 255, 255])
    
    blue_lower = np.array([113, 134, 34])
    blue_upper = np.array([130, 255, 255])
    
    # Detect balls
    mask_yellow = detect_ball_hsv(frame, yellow_lower, yellow_upper, (0,255,255), "Yellow Ball")
    mask_blue = detect_ball_hsv(frame, blue_lower, blue_upper, (255,0,0), "Blue Ball")
    
    # Show result
    cv2.imshow("Ball Detection", frame)
    cv2.imshow("Mask Yellow", mask_yellow)
    cv2.imshow("Mask Blue", mask_blue)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
    if cv2.getWindowProperty("Ball Detection", cv2.WND_PROP_VISIBLE) < 1:
        break

cap.release()
cv2.destroyAllWindows()

