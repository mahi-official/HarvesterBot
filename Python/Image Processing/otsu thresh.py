import cv2
import numpy as np
from matplotlib import pyplot as plt

cap = cv2.VideoCapture(0)

while(1):
    # Take each frame
    _, frame = cap.read()

    #cvt to gray
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Otsu's thresholding after Gaussian filtering
    blur = cv2.GaussianBlur(gray,(5,5),0)
    ret3,th3 = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

    cv2.imshow('frame',th3)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
       break

cv2.destroyAllWindows()
