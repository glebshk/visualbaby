import argparse
import csv
from collections import Counter
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO


def get_stable_count(counts: list[int], method: str = "mode") -> int:
    """
    Calculate a stable final count from frame-level counts.

    mode:
        most frequent count
    median:
        median count rounded to int

    We avoid max() because one false positive can overestimate the result.
    """
    if not counts:
        return 0

    if method == "mode":
        return Counter(counts).most_common(1)[0][0]

    if method == "median":
        return int(round(float(np.median(counts))))

    raise ValueError(f"Unknown count method: {method}")


def draw_detections(frame, boxes, count: int):
    """
    Draw bounding boxes and current count on frame.
    """
    for box in boxes:
        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
        conf = float(box.conf[0].cpu().numpy())

        label = f"baby {conf:.2f}"

        cv2.rectangle(
            frame,
            (x1, y1),
            (x2, y2),
            (255, 0, 0),
            2,
        )

        cv2.putText(
            frame,
            label,
            (x1, max(y1 - 10, 20)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 0, 0),
            2,
        )

    cv2.putText(
        frame,
        f"Frame count: {count}",
        (30, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (255, 255, 255),
        2,
    )

    return frame


def predict_video(
    video_path: str,
    model_path: str,
    output_path: str,
    confidence_threshold: float,
    frame_step: int,
    count_method: str,
) -> None:
    video_path = Path(video_path)
    output_path = Path(output_path)

    if not video_path.exists():
        raise FileNotFoundError(f"Video file not found: {video_path}")

    output_path.parent.mkdir(parents=True, exist_ok=True)

    model = YOLO(model_path)

    cap = cv2.VideoCapture(str(video_path))

    if not cap.isOpened():
        raise RuntimeError(f"Could not open video: {video_path}")

    source_fps = cap.get(cv2.CAP_PROP_FPS)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    if source_fps <= 0:
        source_fps = 30.0

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(
        str(output_path),
        fourcc,
        source_fps,
        (width, height),
    )

    if not writer.isOpened():
        raise RuntimeError(f"Could not create output video: {output_path}")

    counts = []
    frame_idx = 0
    last_boxes = []
    last_count = 0

    csv_path = output_path.with_suffix(".csv")

    with open(csv_path, "w", newline="") as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(["frame_index", "timestamp_sec", "count"])

        print(f"Input video: {video_path}")
        print(f"Output video: {output_path}")
        print(f"Resolution: {width}x{height}")
        print(f"FPS: {source_fps:.2f}")
        print(f"Total frames: {total_frames}")
        print(f"Confidence threshold: {confidence_threshold}")
        print(f"Frame step: {frame_step}")

        while True:
            ret, frame = cap.read()

            if not ret:
                break

            should_predict = frame_idx % frame_step == 0

            if should_predict:
                results = model.predict(
                    source=frame,
                    conf=confidence_threshold,
                    verbose=False,
                )

                result = results[0]
                last_boxes = result.boxes
                last_count = len(last_boxes)

                counts.append(last_count)

                timestamp_sec = frame_idx / source_fps
                csv_writer.writerow([frame_idx, f"{timestamp_sec:.3f}", last_count])

            output_frame = frame.copy()
            output_frame = draw_detections(output_frame, last_boxes, last_count)

            writer.write(output_frame)

            frame_idx += 1

    cap.release()
    writer.release()

    final_count = get_stable_count(counts, method=count_method)

    count_distribution = Counter(counts)

    print("\nPrediction finished.")
    print(f"Frames processed: {frame_idx}")
    print(f"Prediction frames: {len(counts)}")
    print(f"Count distribution: {dict(sorted(count_distribution.items()))}")
    print(f"Min count: {min(counts) if counts else 0}")
    print(f"Max count: {max(counts) if counts else 0}")
    print(f"Final stable count ({count_method}): {final_count}")
    print(f"Saved output video: {output_path}")
    print(f"Saved frame counts CSV: {csv_path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--video", required=True, help="Path to input video")
    parser.add_argument("--model", default="models/baby_detector_v2.pt", help="Path to YOLO model")
    parser.add_argument("--output", default="runs/video_predictions/output.mp4", help="Path to output video")
    parser.add_argument("--conf", type=float, default=0.75, help="Confidence threshold")
    parser.add_argument("--frame-step", type=int, default=1, help="Run prediction every N frames")
    parser.add_argument("--count-method", choices=["mode", "median"], default="mode", help="Stable count method")
    args = parser.parse_args()

    predict_video(
        video_path=args.video,
        model_path=args.model,
        output_path=args.output,
        confidence_threshold=args.conf,
        frame_step=args.frame_step,
        count_method=args.count_method,
    )


if __name__ == "__main__":
    main()
