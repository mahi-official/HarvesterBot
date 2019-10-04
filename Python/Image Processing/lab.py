import cv2
import numpy as np

blue = np.uint8([[[255,0,0 ]]])
green = np.uint8([[[0,255,0 ]]])
red = np.uint8([[[0,0,255 ]]])
lab_blue = cv2.cvtColor(blue,cv2.COLOR_BGR2LAB)
lab_green = cv2.cvtColor(green,cv2.COLOR_BGR2LAB)
lab_red = cv2.cvtColor(red,cv2.COLOR_BGR2LAB)
print lab_blue
print lab_green
print lab_red
