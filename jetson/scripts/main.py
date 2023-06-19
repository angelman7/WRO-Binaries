def removeUpperNoise(image, limit=25):
    width, height = image.size

    for x in range(width):
        for y in range(height - 1, -1, -1):
            if all(val < limit for val in image[x, y]):
                image[x, 0:y] = (0, 0, 0)
                break

    return image
