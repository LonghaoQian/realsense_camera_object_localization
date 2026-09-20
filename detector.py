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
    def __init__(self, model_path="yolov8n.pt", device="cuda", conf_threshold=0.5,
                 classes=None):
        """classes: iterable of COCO class NAMES to keep (e.g. ["bottle"]).

        None keeps every class. Filtering is passed down to Ultralytics rather
        than applied to the results afterwards: the model then skips non-matching
        classes during NMS, so a single-class run is cheaper than detecting all 80
        and discarding 79 of them. It also means the console and any downstream
        consumer never see the other objects at all.
        """
        self.model = YOLO(model_path)
        self.device = device
        self.conf_threshold = conf_threshold
        self.class_ids = self._resolve_classes(classes)

    def _resolve_classes(self, classes):
        """Map class names to the model's own integer ids, failing loudly on typos.

        A silently-ignored bad name is the worst outcome here: the loop would run
        forever reporting nothing, looking exactly like a camera or detection
        fault. So an unknown name raises instead.
        """
        if not classes:
            return None
        name_to_id = {name: idx for idx, name in self.model.names.items()}
        ids = []
        for name in classes:
            key = name.strip().lower()
            if key not in name_to_id:
                close = [n for n in name_to_id if key in n or n in key]
                raise ValueError(
                    f"unknown class {name!r} for model {self.model.model_name if hasattr(self.model, 'model_name') else ''}. "
                    + (f"Did you mean {close}? " if close else "")
                    + f"This model knows {len(name_to_id)} classes; run with --list-classes to see them."
                )
            ids.append(name_to_id[key])
        return ids

    @property
    def class_names(self):
        """Human-readable version of the active filter, for logging."""
        if self.class_ids is None:
            return "all"
        return ",".join(self.model.names[i] for i in self.class_ids)

    def detect(self, color_image):
        results = self.model.predict(
            color_image, device=self.device, conf=self.conf_threshold,
            classes=self.class_ids, verbose=False
        )[0]

        detections = []
        for box in results.boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            label = self.model.names[int(box.cls[0])]
            conf = float(box.conf[0])
            detections.append(Detection2D(label, conf, int(x1), int(y1), int(x2), int(y2)))
        return detections
