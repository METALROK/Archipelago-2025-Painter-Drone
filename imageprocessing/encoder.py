"""
Extraction of coordinates from an image
"""

import cv2
import numpy as np


def get_line_points(image_path: str, threshold: int=200) -> tuple:
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise ValueError("Изображение не загружено. Проверьте путь: " + image_path)

    _, binary = cv2.threshold(img, threshold, 255, cv2.THRESH_BINARY)

    y_coords, x_coords = np.where(binary == 0)
    points = tuple(zip(x_coords, y_coords))

    return points
