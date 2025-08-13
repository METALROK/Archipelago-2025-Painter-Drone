import cv2


def imageConditioning(image_name):
    image_name_full = image_name + ".jpg"
    image_name_lowres = "lowres_" + image_name_full
    image = cv2.imread(image_name_full)

    # cv2.imshow(image_name, image)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    height, width = image.shape[:2]
    new_height = height // 8
    new_width = width // 8

    resized = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_NEAREST_EXACT)
    cv2.imwrite(image_name_lowres, resized, [cv2.IMWRITE_JPEG_QUALITY, 100])
    return image_name_lowres


def imageConditioningFourDrones(image_name):
    image_name_full = image_name + ".jpg"
    image_name_lowres = "lowres_" + image_name_full
    image = cv2.imread(image_name_full)

    # cv2.imshow(image_name, image)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    height, width = image.shape[:2]
    new_height = height // 8
    new_width = width // 8

    resized = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_NEAREST_EXACT)
    cv2.imwrite(image_name_lowres, resized, [cv2.IMWRITE_JPEG_QUALITY, 100])

    image1 = cv2.imread(image_name_lowres)
    height, width = image1.shape[:2]
    part_height = height // 2
    part_width = width // 2

    parts = []
    part_counter = 1
    for i in range(2):
        for j in range(2):
            # Определение координат для вырезания части
            y_start = i * part_height
            y_end = (i + 1) * part_height
            x_start = j * part_width
            x_end = (j + 1) * part_width

            # Вырезание части
            part = image1[y_start:y_end, x_start:x_end]
            parts.append(part)

            # Сохранение части
            part_name = f"{image_name}_{part_counter}.jpg"
            cv2.imwrite(part_name, part)
            part_counter += 1
    return image_name


if __name__ == "__main__":
    print("Введите название файла без расширения")
    image_name = input()
    result = imageConditioningFourDrones(image_name)
    imageConditioningFourDrones(image_name)