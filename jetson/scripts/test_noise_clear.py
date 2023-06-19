from cv2 import *

def removeUpperNoise(image, limit):
    width, height, _ = image.shape
    for x in range(width):
        for y in range(height - 1, -1, -1):
            if all(val < limit for val in image[x, y]):
                image[0:y, x] = (0, 0, 0)
                break

    return image

def nothing(x):
    return

def main():
    namedWindow("Image")
    createTrackbar("Limit", "Image", 1, 255, nothing)

    while True:
        or_image = imread("images/img_6.png")
        ed_image = removeUpperNoise(image=or_image, limit=getTrackbarPos("Limit", "Image"))
        imshow("Image", ed_image)
        waitKey(1)

if __name__ == "__main__":
    or_image = imread("images/img_6.png")
    for i in range(25, 45, 5):
        imshow(str(i), removeUpperNoise(image=or_image, limit=i))
    waitKey(0)
