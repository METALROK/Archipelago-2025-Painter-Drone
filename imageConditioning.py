import cv2

image = cv2.imread("indane_structure.jpg")

# cv2.imshow('Indane', image)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

height, width = image.shape[:2]
new_height = height // 8
new_width = width // 8

resized = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_NEAREST_EXACT)
cv2.imwrite("indane_lowres.jpg", resized, [cv2.IMWRITE_JPEG_QUALITY, 100])

image1 = cv2.imread("indane_lowres.jpg")
height, width = image1.shape[:2]
part_height = height // 2
part_width = width // 2

parts = []
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
        part_name = f"{'indane'}_{i + 1}{j + 1}.jpg"
        cv2.imwrite(part_name, part)