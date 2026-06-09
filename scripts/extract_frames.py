import argparse
import csv
from pathlib import Path

import cv2


def compute_sharpness(frame):
    """
    Estimate image sharpness using variance of Laplacian.
    Higher value means the image is sharper.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return cv2.Laplacian(gray, cv2.CV_64F).var()


def extract_frames_from_video(
    video_path: Path,
    output_dir: Path,
    target_fps: float,
    image_format: str,
    min_sharpness: float | None,
    jpeg_quality: int,
    global_start_index: int,
    metadata_writer,
) -> int:
    if not video_path.exists():
        raise FileNotFoundError(f"Video file not found: {video_path}")

    cap = cv2.VideoCapture(str(video_path))

    if not cap.isOpened():
        raise RuntimeError(f"Could not open video: {video_path}")

    source_fps = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    if source_fps <= 0:
        raise RuntimeError(f"Could not read FPS from video: {video_path}")

    frame_interval = max(int(round(source_fps / target_fps)), 1)

    print(f"\nProcessing video: {video_path}")
    print(f"  Source FPS: {source_fps:.2f}")
    print(f"  Total frames: {total_frames}")
    print(f"  Target extraction FPS: {target_fps}")
    print(f"  Frame interval: every {frame_interval} frame(s)")

    saved_count = 0
    skipped_blurry = 0
    frame_idx = 0

    while True:
        ret, frame = cap.read()

        if not ret:
            break

        if frame_idx % frame_interval == 0:
            timestamp_sec = frame_idx / source_fps
            sharpness = compute_sharpness(frame)

            if min_sharpness is not None and sharpness < min_sharpness:
                skipped_blurry += 1
                frame_idx += 1
                continue

            global_frame_index = global_start_index + saved_count
            filename = f"frame_{global_frame_index:06d}_{video_path.stem}.{image_format}"
            save_path = output_dir / filename

            if image_format.lower() in ["jpg", "jpeg"]:
                cv2.imwrite(
                    str(save_path),
                    frame,
                    [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality],
                )
            elif image_format.lower() == "png":
                cv2.imwrite(str(save_path), frame)
            else:
                raise ValueError("image_format must be 'jpg' or 'png'")

            height, width = frame.shape[:2]

            metadata_writer.writerow(
                [
                    filename,
                    str(video_path),
                    frame_idx,
                    f"{timestamp_sec:.3f}",
                    width,
                    height,
                    f"{sharpness:.3f}",
                ]
            )

            saved_count += 1

        frame_idx += 1

    cap.release()

    print(f"  Saved frames: {saved_count}")
    print(f"  Skipped blurry frames: {skipped_blurry}")

    return saved_count


def extract_frames(
    video_paths: list[str],
    output_dir: str,
    target_fps: float = 3.3,
    image_format: str = "jpg",
    min_sharpness: float | None = None,
    jpeg_quality: int = 95,
) -> None:
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    metadata_path = output_dir / "metadata.csv"

    total_saved = 0

    with open(metadata_path, "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(
            [
                "filename",
                "source_video",
                "source_frame_index",
                "timestamp_sec",
                "width",
                "height",
                "sharpness",
            ]
        )

        for video_path in video_paths:
            saved_from_video = extract_frames_from_video(
                video_path=Path(video_path),
                output_dir=output_dir,
                target_fps=target_fps,
                image_format=image_format,
                min_sharpness=min_sharpness,
                jpeg_quality=jpeg_quality,
                global_start_index=total_saved,
                metadata_writer=writer,
            )

            total_saved += saved_from_video

    print("\nExtraction finished.")
    print(f"Total saved frames: {total_saved}")
    print(f"Output directory: {output_dir}")
    print(f"Metadata saved to: {metadata_path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--videos",
        nargs="+",
        required=True,
        help="Paths to input MP4 videos",
    )
    parser.add_argument(
        "--output",
        required=True,
        help="Output folder for extracted frames",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=3.3,
        help="How many frames per second to extract from each video",
    )
    parser.add_argument(
        "--format",
        default="jpg",
        choices=["jpg", "png"],
        help="Image format for extracted frames",
    )
    parser.add_argument(
        "--min-sharpness",
        type=float,
        default=None,
        help="Optional blur filter threshold",
    )
    parser.add_argument(
        "--jpeg-quality",
        type=int,
        default=95,
        help="JPEG quality, only used for jpg",
    )

    args = parser.parse_args()

    extract_frames(
        video_paths=args.videos,
        output_dir=args.output,
        target_fps=args.fps,
        image_format=args.format,
        min_sharpness=args.min_sharpness,
        jpeg_quality=args.jpeg_quality,
    )


if __name__ == "__main__":
    main()
