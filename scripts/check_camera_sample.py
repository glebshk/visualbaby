import argparse
import numpy as np

from src.drone_vision.data.rgb_loader import load_rgb_image
from src.drone_vision.data.depth_loader import load_depth_r16


def describe_rgb(image: np.ndarray) -> None:
    print("RGB image:")
    print(f"  shape: {image.shape}")
    print(f"  dtype: {image.dtype}")
    print(f"  min pixel value: {image.min()}")
    print(f"  max pixel value: {image.max()}")
    print(f"  mean brightness: {image.mean():.2f}")


def describe_depth(depth: np.ndarray) -> None:
    valid_depth = depth[depth > 0]

    print("Depth image:")
    print(f"  shape: {depth.shape}")
    print(f"  dtype: {depth.dtype}")
    print(f"  min raw depth value: {depth.min()}")
    print(f"  max raw depth value: {depth.max()}")

    if valid_depth.size > 0:
        print(f"  valid pixels: {valid_depth.size}")
        print(f"  median valid depth: {np.median(valid_depth):.2f}")
        print(f"  min valid depth: {valid_depth.min()}")
        print(f"  max valid depth: {valid_depth.max()}")
    else:
        print("  no valid depth pixels found")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--rgb", required=True, help="Path to RGB image")
    parser.add_argument("--depth", required=False, help="Path to R16 depth image")
    args = parser.parse_args()

    rgb = load_rgb_image(args.rgb)
    describe_rgb(rgb)

    if args.depth:
        depth = load_depth_r16(args.depth)
        describe_depth(depth)


if __name__ == "__main__":
    main()
