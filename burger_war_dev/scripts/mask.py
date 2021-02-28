import cv2
import numpy as np
def mask_red(img):
    #img = cv2.imread(path)
    #hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    red_min = np.array([0, 0, 105], np.uint8)
    red_max = np.array([100, 100, 255], np.uint8)

    red_region = cv2.inRange(img, red_min, red_max)
    inv_mask = cv2.bitwise_not(red_region)  # make mask for not-blue area


    extracted = cv2.bitwise_and(img, img, mask=red_region)

    white = np.full(img.shape, 255, dtype=img.dtype)
    background = cv2.bitwise_and(white, white, mask=inv_mask)

    masked = cv2.add(extracted, background)

    return masked

def mask_green(img):
    #img = cv2.imread(path)
    #hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    green_min = np.array([0, 105, 0], np.uint8)
    green_max = np.array([100, 255, 100], np.uint8)

    green_region = cv2.inRange(img, green_min, green_max)
    inv_mask = cv2.bitwise_not(green_region)  # make mask for not-blue area


    extracted = cv2.bitwise_and(img, img, mask=green_region)

    white = np.full(img.shape, 255, dtype=img.dtype)
    background = cv2.bitwise_and(white, white, mask=inv_mask)

    masked = cv2.add(extracted, background)

    return masked

def mask_red_green(img):
    #img = cv2.imread(path)
    #hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    red_min = np.array([0, 0, 105], np.uint8)
    red_max = np.array([100, 100, 255], np.uint8)

    green_min = np.array([0, 105, 0], np.uint8)
    green_max = np.array([100, 255, 100], np.uint8)

    red_region = cv2.inRange(img, red_min, red_max)
    green_region = cv2.inRange(img, green_min, green_max)

    inv_mask_r = cv2.bitwise_not(red_region)  # make mask for not-blue area
    inv_mask_g = cv2.bitwise_not(green_region)  # make mask for not-blue area


    extracted_r = cv2.bitwise_and(img, img, mask=red_region)
    extracted_g = cv2.bitwise_and(img, img, mask=green_region)

    white = np.full(img.shape, 255, dtype=img.dtype)
    background_r = cv2.bitwise_and(white, white, mask=inv_mask_r)
    background_g = cv2.bitwise_and(white, white, mask=inv_mask_g)


    masked_r = cv2.add(extracted_r, background_r)
    masked_g = cv2.add(extracted_g, background_g)
    masked = cv2.add(masked_r, masked_g)

    return masked




def mask_blue(img):
    #img = cv2.imread(path)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    blue_min = np.array([100, 30, 30], np.uint8)
    blue_max = np.array([255, 120, 120], np.uint8)

    blue_region = cv2.inRange(hsv, blue_min, blue_max)
    white = np.full(img.shape, 255, dtype=img.dtype)
    background = cv2.bitwise_and(white, white, mask=blue_region)  # detected blue area becomes white

    inv_mask = cv2.bitwise_not(blue_region)  # make mask for not-blue area
    extracted = cv2.bitwise_and(img, img, mask=inv_mask)

    masked = cv2.add(extracted, background)

    return masked

    