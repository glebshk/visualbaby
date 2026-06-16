from ultralytics import YOLO

from src.drone_vision.inference.count_babies import BabyCounter


class BabyDetector:
    def __init__(self, model_path: str = "yolov8n.pt", confidence_threshold: float = 0.7):
        self.model = YOLO(model_path)
        self.confidence_threshold = confidence_threshold
        self.counter = BabyCounter()

    def detect(self, image):
        results = self.model.predict(
            source=image,
            conf=self.confidence_threshold,
            verbose=False,
        )
        return results[0]

    def count(self, image) -> int:
        """
        Per-frame detection count. Resets to 0 whenever nothing is detected
        in the current frame; use track_count() for a persistent total.
        """
        result = self.detect(image)
        return len(result.boxes)

    def track_count(self, image) -> int:
        """
        Persistent count of unique babies/dolls seen so far, updated with
        the detections from this frame. Does not reset when no detections
        are found and does not double-count an object already being tracked.
        """
        result = self.detect(image)
        boxes = [tuple(box.xyxy[0].tolist()) for box in result.boxes]
        return self.counter.update(boxes)
