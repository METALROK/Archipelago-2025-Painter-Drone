import cv2
import numpy as np
import json

def coordinateGeneration(image_name_lowres):
    #Версия для одного дрона

    image = cv2.imread(image_name_lowres, cv2.IMREAD_GRAYSCALE)
    black_threshold = 10

    black_pixels_single = np.argwhere(image <= black_threshold)
    print(np.array2string(black_pixels_single, separator=', '))

    with open('coordinates.json', 'w') as f:
        json.dump(black_pixels_single.tolist(), f, indent=2)

if __name__ == '__main__':
    coordinateGeneration(image_name_lowres)

def coordinateGenerationFourDrones(image_name):
    #Версия для четырех дронов(херня т.к. не JSON)
    image1 = cv2.imread(f"{image_name}_1.jpg", cv2.IMREAD_GRAYSCALE)
    image2 = cv2.imread(f"{image_name}_2.jpg", cv2.IMREAD_GRAYSCALE)
    image3 = cv2.imread(f"{image_name}_3.jpg", cv2.IMREAD_GRAYSCALE)
    image4 = cv2.imread(f"{image_name}_4.jpg", cv2.IMREAD_GRAYSCALE)
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

    for i in range(0, 4):
        with open(f'coordinates{i}.json', 'w') as f:
            json.dump(black_pixels[i].tolist(), f, indent=2)

    for n in range(4):
        print(f'Кол-во пикселей в {n+1} изображении: ', len(black_pixels[n]))

if __name__ == '__main__':
    coordinateGenerationFourDrones(image_name_lowres)