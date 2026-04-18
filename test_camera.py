import cv2

# Open default USB camera (index 0)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Failed to open camera")
    exit()

while True:
    # Read frame from camera
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    # Show frame in window
    cv2.imshow("USB Camera", frame)

    # Exit if 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    # Exit if window is closed manually
    if cv2.getWindowProperty("USB Camera", cv2.WND_PROP_VISIBLE) < 1:
        break

# Release camera and close windows
cap.release()
cv2.destroyAllWindows()
