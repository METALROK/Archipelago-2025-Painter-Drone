
from imageConditioning import imageConditioning
from coordinateGeneration import coordinateGeneration

def main_processor(image_name):

    lowres_image = imageConditioning(image_name)

    coordinateGeneration(lowres_image)
    print("По идее все должно быть готово")

if __name__ == '__main__':
    print("Введите название файла без расширения")
    input_image_name = input()
    main_processor(input_image_name)






