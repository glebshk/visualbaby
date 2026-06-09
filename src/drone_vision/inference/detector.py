from ultralytics import YOLO

class BabyDetector:
    def __init__(self, model_path: str = "yolov8n.pt", confidence_threshold: float = 0.7):
        self.model = YOLO(model_path)
        self.confidence_threshold = confidence_threshold

    def detect(self, image):
        results = self.model.predict(
            source=image,
            conf=self.confidence_threshold,
            verbose=False,
        )
        return results[0]

    def count(self, image) -> int:
        result = self.detect(image)
        return len(result.boxes)
