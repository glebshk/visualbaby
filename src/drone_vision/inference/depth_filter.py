import numpy as np


def get_median_depth_in_bbox(depth_image: np.ndarray, bbox: tuple[int, int, int, int]) -> float:
    """
    Calculate median valid depth value inside bounding box.

    bbox format:
    (x1, y1, x2, y2)
    """
    x1, y1, x2, y2 = bbox

    crop = depth_image[y1:y2, x1:x2]
    valid_values = crop[crop > 0]

    if valid_values.size == 0:
        return float("nan")

    return float(np.median(valid_values))


def is_depth_valid(
    depth_value: float,
    min_depth: float | None = None,
    max_depth: float | None = None,
) -> bool:
    """
    Check if depth value is within expected range.

    The actual min/max thresholds should be defined after seeing real camera data.
    """
    if np.isnan(depth_value):
        return False

    if min_depth is not None and depth_value < min_depth:
        return False

    if max_depth is not None and depth_value > max_depth:
        return False

    return True
