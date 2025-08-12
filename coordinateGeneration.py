import cv2
import numpy as np

# #Версия для одного дрона

# image = cv2.imread("indane_lowres.jpg", cv2.IMREAD_GRAYSCALE)
# black_threshold = 10
#
# black_pixels = np.argwhere(image <= black_threshold)
#
#
# z_coords = black_pixels[:, 0]
# x_coords = black_pixels[:, 1]
#
# print('x values: ', x_coords)
# print('z values: ', z_coords)

#Версия для четырех дронов
image1 = cv2.imread("indane_11.jpg", cv2.IMREAD_GRAYSCALE)
image2 = cv2.imread("indane_12.jpg", cv2.IMREAD_GRAYSCALE)
image3 = cv2.imread("indane_21.jpg", cv2.IMREAD_GRAYSCALE)
image4 = cv2.imread("indane_22.jpg", cv2.IMREAD_GRAYSCALE)
black_threshold = 10

black_pixels1 = np.argwhere(image1 <= black_threshold)
black_pixels2 = np.argwhere(image2 <= black_threshold)
black_pixels3 = np.argwhere(image3 <= black_threshold)
black_pixels4 = np.argwhere(image4 <= black_threshold)

black_pixels = {
    0: black_pixels1,
    1: black_pixels2,
    2: black_pixels3,
    3: black_pixels4,
}

z_coords1 = black_pixels1[:, 0]
z_coords2 = black_pixels2[:, 0]
z_coords3 = black_pixels3[:, 0]
z_coords4 = black_pixels4[:, 0]

z_coords = {
    0: z_coords1,
    1: z_coords2,
    2: z_coords3,
    3: z_coords4,
}

x_coords1 = black_pixels1[:, 1]
x_coords2 = black_pixels2[:, 1]
x_coords3 = black_pixels3[:, 1]
x_coords4 = black_pixels4[:, 1]

x_coords = {
    0: x_coords1,
    1: x_coords2,
    2: x_coords3,
    3: x_coords4,
}
for n in range(4):
    print(f'Кол-во пикселей в {n+1} изображении: ', len(black_pixels[n]))
    #print(f'{n+1} Часть: ', z_coords[n], x_coords[n])