
from imageConditioning import imageConditioning, imageConditioningFourDrones
from coordinateGeneration import coordinateGeneration, coordinateGenerationFourDrones


def main_processorOneDrone(image_name):

    lowres_image = imageConditioning(image_name)

    coordinateGeneration(lowres_image)
    print("По идее все должно быть готово")

def main_processorFourDrones(image_name):

    image_name1 = imageConditioningFourDrones(image_name)

    coordinateGenerationFourDrones(image_name1)
    print("По идее все должно быть готово для 4 дронов")

if __name__ == '__main__':
    print("Выберите 1 или 4 дронов")
    number_of_drones = int(input())
    print("Введите название файла без расширения")
    input_image_name = input()
    if number_of_drones == 1:
        main_processorOneDrone(input_image_name)
    elif number_of_drones == 4:
        main_processorFourDrones(input_image_name)
    else:
        print("Хорош прикалываться, введите 1 или 4")








