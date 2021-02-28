#-*- coding: utf-8 -*- 
import cv2
import numpy as np


def detect_red_color(img):
    #img = cv2.imread(path)
    
    #Convert to HSV color space
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    #Red HSV Range 1
    hsv_min = np.array([0,64,0], np.uint8)
    hsv_max = np.array([30,255,255], np.uint8)
    mask1 = cv2.inRange(hsv,hsv_min,hsv_max)
    #red_min = np.array([0, 0, 105], np.uint8)
    #red_max = np.array([100, 100, 255], np.uint8)
    #Red HSV Range 2
    hsv_min = np.array([150,64,0], np.uint8)
    hsv_max = np.array([179,255,255], np.uint8)
    mask2 = cv2.inRange(hsv,hsv_min,hsv_max)

    #Mask in red area(255:Red, 0:other than red)
    mask = mask1 + mask2
    
    #Making process
    masked_img = cv2.bitwise_and(img, img, mask=mask)

   #Making numeber count
    mask_mean = np.mean(mask)
    #print('mask_mean')
    #print(mask_mean)
   
    return mask_mean


def detect_green_color(img):
  
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    #
    hsv_min = np.array([30, 64, 0], np.uint8)
    hsv_max = np.array([90,255,255], np.uint8)

    # ）    
    mask = cv2.inRange(hsv, hsv_min, hsv_max)
    
    # 
    masked_img = cv2.bitwise_and(img, img, mask=mask)
    #Making numeber count
    mask_mean = np.mean(mask)
    #print('mask_mean')
    #print(mask_mean)
   
    return mask_mean
    #return mask

# 
def detect_blue_color(img):
    
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    
    hsv_min = np.array([90, 64, 0], np.uint8)
    hsv_max = np.array([150,255,255], np.uint8)

    #   
    mask = cv2.inRange(hsv, hsv_min, hsv_max)

    # 
    masked_img = cv2.bitwise_and(img, img, mask=mask)
    #Making numeber count
    mask_mean = np.mean(mask)
    #print('mask_mean')
    #print(mask_mean)
   
    return mask_mean

# detect yellow color
def detect_yellow_color(img):
    # 
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    #
    hsv_min = np.array([20, 80, 10], np.uint8)
    hsv_max = np.array([50,255,255], np.uint8)

    #   
    mask = cv2.inRange(hsv, hsv_min, hsv_max)

    # マスキング処理
    masked_img = cv2.bitwise_and(img, img, mask=mask)
    #Making numeber count
    mask_mean = np.mean(mask)
    #print('mask_mean')
    #print(mask_mean)
   
    return mask_mean
    #return mask, masked_img

# detect white color
def detect_white_color(img):
    # 
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # 
    hsv_min = np.array([0, 0, 100], np.uint8)
    hsv_max = np.array([180,45,255], np.uint8)

    #    
    mask = cv2.inRange(hsv, hsv_min, hsv_max)

    # 
    masked_img = cv2.bitwise_and(img, img, mask=mask)
    #Making numeber count
    mask_mean = np.mean(mask)
    #print('mask_mean')
    #print(mask_mean)
   
    return mask_mean
    #return mask