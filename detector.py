"""2D object detection on RGB frames using YOLOv8 (Ultralytics)."""

from dataclasses import dataclass

from ultralytics import YOLO


@dataclass
class Detection2D:
    label: str
    confidence: float
    x1: int
    y1: int
    x2: int
    y2: int

    @property
    def center(self):
        return (self.x1 + self.x2) // 2, (self.y1 + self.y2) // 2


class ObjectDetector:
    def __init__(self, model_path="yolov8n.pt", device="cuda", conf_threshold=0.5):
        self.model = YOLO(model_path)
        self.device = device
        self.conf_threshold = conf_threshold

    def detect(self, color_image):
        results = self.model.predict(
            color_image, device=self.device, conf=self.conf_threshold, verbose=False
        )[0]

        detections = []
        for box in results.boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            label = self.model.names[int(box.cls[0])]
            conf = float(box.conf[0])
            detections.append(Detection2D(label, conf, int(x1), int(y1), int(x2), int(y2)))
        return detections
