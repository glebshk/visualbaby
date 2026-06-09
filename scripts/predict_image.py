import argparse

from src.drone_vision.data.rgb_loader import load_rgb_image
from src.drone_vision.inference.detector import BabyDetector


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", required=True, help="Path to RGB image")
    parser.add_argument("--model", default="yolov8n.pt", help="Path to YOLO model")
    parser.add_argument("--conf", type=float, default=0.7, help="Confidence threshold")
    args = parser.parse_args()

    image = load_rgb_image(args.image)

    detector = BabyDetector(
        model_path=args.model,
        confidence_threshold=args.conf,
    )

    count = detector.count(image)

    print(f"Detected objects: {count}")


if __name__ == "__main__":
    main()
