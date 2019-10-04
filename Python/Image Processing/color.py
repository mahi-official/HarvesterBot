import cv2
import numpy as np
import os
import time
import imutils
import math

cap = cv2.VideoCapture(1)

while(1):
    
    def colorDistance(kCol, uCol):
    # compute sum of square of differences between each channel 
        d = (kCol[0] - uCol[0])**2 + (kCol[1] - uCol[1])**2 +(kCol[2] - uCol[2])**2
            
        # square root of sum is the Euclidean distance between the
        # two colors
        return math.sqrt(d)

    # Take each frame
    _, frame = cap.read()

    # Convert BGR to HSV
##    frame = imutils.resize(frame, width=240)
##    frame = imutils.resize(frame, height=320)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    blur = cv2.medianBlur(hsv,(5))

    # define range of red color in HSV
    lower_blue = np.array([100,70,40])
    upper_blue = np.array([130,255,255])
    lower_orange = np.array([5,80,80])
    upper_orange = np.array([20,220,200])
    lower_red = np.array([160,100,100])
    upper_red = np.array([189,250,200])

    # Threshold the HSV image to get only colors
    mask1 = cv2.inRange(blur, lower_blue, upper_blue)
    mask2 = cv2.inRange(blur, lower_red, upper_red)
    mask3 = cv2.inRange(blur, lower_orange, upper_orange)
    mask = mask1 + mask2 + mask3
    kernel = np.ones((5,5),np.uint8)
    opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame,frame, mask= mask)
    #th3 = cv2.adaptiveThreshold(mask,200,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)
    ret,th3 = cv2.threshold(mask,200,255,cv2.THRESH_BINARY)
    th3 = cv2.dilate(th3, None, iterations=2)
    (_,cnts, _) = cv2.findContours(th3,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
##    avg = 0
##    for c in contours:
##        avg += len(c)
##        
    if (len(cnts) > 0):
        cv2.drawContours(res, cnts, -1, 255, 3)
        cv2.drawContours(frame, cnts, -1, 255, 3)
        #minDist = (np.inf, None)
        c = max(cnts, key = cv2.contourArea)
        (x,y),radius = cv2.minEnclosingCircle(c)
        p = int(x)
        q = int(y)
        center = (int(x),int(y))
        radius = int(radius)
        print center
        res = cv2.circle(res,center,radius,(0,255,0),2)
        frame = cv2.circle(frame,center,radius,(0,255,0),2)
##        M = cv2.moments(c)
##        cx = int(M['m10']/M['m00'])
##        cy = int(M['m01']/M['m00'])

        # reference colors
        ORANGE = (0, 0, 0)
        RED = (0, 0, 255)
        BLUE = (255, 0, 0)
        
        # find average color for 9 pixel kernel around centroid
        b = frame[q - 3 : q + 3, p - 3 : p + 3, 0]
        g = frame[q - 3 : q + 3, p - 3 : p + 3, 1]
        r = frame[q - 3 : q + 3, p - 3 : p + 3, 2]

        bAvg = np.mean(b)
        gAvg = np.mean(g)
        rAvg = np.mean(r)
        # find distances to known reference colors
        dist = []
        dist.append(colorDistance(ORANGE, (bAvg, gAvg, rAvg)))
        dist.append(colorDistance(BLUE, (bAvg, gAvg, rAvg)))
        dist.append(colorDistance(RED, (bAvg, gAvg, rAvg)))
        #finding the closest color
        minDist = min(dist)
        # for the desired count the shape
        if dist.index(minDist) == 1:
            print("BLUE:")
        elif dist.index(minDist) == 2:
             print("RED:")
        elif dist.index(minDist) == 0:
             print("ORANGE")
       # print mean
##        if int(x)<480:
##         px =frame[int(x+10),int(y),0]
##         print px
##        else:
##          break
##        print radius
#         time.sleep(5)
##        if (radius>200):
##         print 'large'
##        elif (radius<120):
##         print 'small'
##        else:
##         print 'medium'
##        x,y,w,h = cv2.boundingRect(c)
##        cv2.rectangle(res,(x,y),(x+w,y+h),(0,255,255),3)
##        cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,255),3)
##        rect = frame[x:x+w, y:y+h]
##        rect.mean()
##        print(rect.mean)
        
##    cnt = contours[0]
##    M = cv2.moments(cnt)
##    print M

    cv2.imshow('frame',frame)
    cv2.imshow('mask',mask)
    cv2.imshow('res',res)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()
