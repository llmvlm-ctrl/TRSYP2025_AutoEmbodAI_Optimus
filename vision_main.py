from picamera2 import Picamera2
from ultralytics import YOLO
import cv2

# === Constants ===
FOCAL_LENGTH = 600.0  # pixels
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

# === Model setup ===
model = YOLO("yolov8n.pt")

print("[INFO] Detection and distance estimation started — press 'q' to quit")

while True:
    frame = picam2.capture_array()
    results = model(frame, stream=True)

    for r in results:
        annotated = frame.copy()

        for box in r.boxes:
            cls = int(box.cls)
            label = model.names[cls].lower()
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            h_pixels = max(1, y2 - y1)

            if label in KNOWN_HEIGHTS:
                distance = (KNOWN_HEIGHTS[label] * FOCAL_LENGTH) / h_pixels
                text = f"{label} {distance/100:.2f} m"
            else:
                text = label

            # Draw everything
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(annotated, text, (x1, max(20, y1 - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.imshow("SmartCar Vision", annotated)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
picam2.stop()

