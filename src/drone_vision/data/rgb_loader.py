import cv2
import numpy as np


def load_rgb_image(path: str) -> np.ndarray:
    """
    Load RGB image.

    Expected resolution:
    - 1280x720
    """
    image_bgr = cv2.imread(path, cv2.IMREAD_COLOR)

    if image_bgr is None:
        raise RuntimeError(f"Could not read RGB image: {path}")

    image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)

    return image_rgb
