from picamera2 import Picamera2
from ultralytics import YOLO
import cv2

# === Known object for calibration ===
CALIB_LABEL = 'bottle'       # object you will use for calibration
CALIB_HEIGHT = 30.0         # cm, real height of the object
CALIB_DISTANCE = 90.0      # cm, distance from camera during calibration

# === Dictionary of real heights for all objects ===
KNOWN_HEIGHTS = {
    'person': 170.0,
    'bicycle': 100.0,
    'car': 150.0,
    'motorcycle': 120.0,
    'bus': 300.0,
    'truck': 320.0,
    'traffic light': 80.0,
    'stop sign': 75.0,
    'bottle': 25.0,
    'chair': 90.0
}

# === Camera setup ===
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.configure("preview")
picam2.start()

# === YOLO model ===
model = YOLO("yolov8n.pt")
print("[INFO] Place the calibration object in view and press 'c' to calibrate.")

# --- Calibration loop ---
FOCAL_LENGTH = None
while FOCAL_LENGTH is None:
    frame = picam2.capture_array()
    results = model(frame)
    annotated = frame.copy()

    for box in results[0].boxes:
        cls = int(box.cls)
        label = model.names[cls].lower()
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        cv2.rectangle(annotated, (x1, y1), (x2, y2), (255, 0, 0), 2)
        cv2.putText(annotated, label, (x1, max(20, y1 - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

    cv2.imshow("Calibration", annotated)
    key = cv2.waitKey(1)

    if key == ord('c'):
        # compute focal length if calibration object detected
        for box in results[0].boxes:
            cls = int(box.cls)
            label = model.names[cls].lower()
            if label == CALIB_LABEL.lower():
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                h_pixels = max(1, y2 - y1)
                FOCAL_LENGTH = (h_pixels * CALIB_DISTANCE) / CALIB_HEIGHT
                print(f"[INFO] Calibrated focal length = {FOCAL_LENGTH:.2f} pixels")
                break

    elif key == ord('q'):
        exit()

cv2.destroyAllWindows()

# --- Detection loop with distances ---
print("[INFO] Starting detection with distance estimation. Press 'q' to quit.")
while True:
    frame = picam2.capture_array()
    results = model(frame, stream=True)
    annotated = frame.copy()

    for r in results:
        for box in r.boxes:
            cls = int(box.cls)
            label = model.names[cls].lower()
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            h_pixels = max(1, y2 - y1)

            # compute distance if known height
            if label in KNOWN_HEIGHTS:
                distance = (KNOWN_HEIGHTS[label] * FOCAL_LENGTH) / h_pixels
                text = f"{label} {distance/100:.2f} m"
            else:
                text = label

            # Draw box + text
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(annotated, text, (x1, max(20, y1 - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    cv2.imshow("SmartCar Vision", annotated)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
picam2.stop()
