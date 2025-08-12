import cv2
import numpy as np

image = cv2.imread("indane_lowres.jpg", cv2.IMREAD_GRAYSCALE)
#РАБОТАЕТ ТОЛЬКО С ЧЕРНО-БЕЛЫМИ КАРТИНКАМИ
black_threshold = 10

black_pixels = np.argwhere(image <= black_threshold)

z_coords = black_pixels[:, 0]
x_coords = black_pixels[:, 1]

print('x values: ', x_coords)
print('z values: ', z_coords)