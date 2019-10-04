import cv2
import numpy as np
from matplotlib import pyplot as plt

#e1 = cv2.getTickCount()

cap = cv2.VideoCapture(1)
template = cv2.imread('Sample Fruits\large_blueberry.jpg',0)
w, h = template.shape[::-1]

while(1):

    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply template Matching
    res = cv2.matchTemplate(gray,template,cv2.TM_SQDIFF)
    cv2.imshow('frame',gray)

    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

    top_left = min_loc
    bottom_right = (top_left[0] + w, top_left[1] + h)
    cv2.rectangle(gray ,top_left, bottom_right, (50, 0, 130), 2)

    cv2.imshow('output',gray)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
cap.release()
cv2.destroyAllWindows()

#e2 = cv2.getTickCount()
#t = (e2 - e1)/cv2.getTickFrequency()
#print t

