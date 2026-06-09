import cv2
import numpy as np


def load_depth_r16(path: str) -> np.ndarray:
    """
    Load an R16 depth image.

    Expected format:
    - single-channel 16-bit image
    - resolution: 1280x720
    """
    depth = cv2.imread(path, cv2.IMREAD_UNCHANGED)

    if depth is None:
        raise RuntimeError(f"Could not read depth image: {path}")

    if depth.dtype != np.uint16:
        raise ValueError(f"Expected uint16 depth image, got {depth.dtype}")

    if len(depth.shape) != 2:
        raise ValueError(f"Expected single-channel depth image, got shape {depth.shape}")

    return depth
