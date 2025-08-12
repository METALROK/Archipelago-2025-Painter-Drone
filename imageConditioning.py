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