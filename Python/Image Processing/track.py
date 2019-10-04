#importing modules

import cv2
import serial
import numpy as np
import serial

#capturing video through webcam
cap=cv2.VideoCapture(1)
#ser = serial.Serial('COM3', 9600)

while(1):
    _, img = cap.read()
        
    #converting frame(img i.e BGR) to HSV (hue-saturation-value)

    hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

    #definig the range of red color
    red_lower=np.array([165,100,135],np.uint8)
    red_upper=np.array([180,190,255],np.uint8)

    #defining the Range of Blue color
    blue_lower=np.array([100,110,130],np.uint8)
    blue_upper=np.array([110,255,255],np.uint8)

    #defining the Range of orange color
    orange_lower=np.array([22,60,200],np.uint8)
    orange_upper=np.array([60,255,255],np.uint8)

    #finding the range of red,blue and orange color in the image
    red=cv2.inRange(hsv, red_lower, red_upper)
    blue=cv2.inRange(hsv,blue_lower,blue_upper)
    orange=cv2.inRange(hsv,orange_lower,orange_upper)

    #Morphological transformation, Dilation  	
    kernal = np.ones((5 ,5), "uint8")

    red=cv2.dilate(red, kernal)
    res=cv2.bitwise_and(img, img, mask = red)

    blue=cv2.dilate(blue,kernal)
    res1=cv2.bitwise_and(img, img, mask = blue)

    orange=cv2.dilate(orange,kernal)
    res2=cv2.bitwise_and(img, img, mask = orange)    


    #Tracking the Red Color
    (_,contours,hierarchy)=cv2.findContours(red,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if(area>3500):
               c = max(contours, key = cv2.contourArea)
               (x,y),radius = cv2.minEnclosingCircle(c)
               center = (int(x),int(y))
               radius = int(radius)
               img = cv2.circle(img,center,radius,(0,255,0),2)
               cv2.putText(img,"APPLE",(int(x),int(y)),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0))
               print radius
               if (radius>200):
                print 'large'
                cv2.putText(img,"LARGE",(int(x+40),int(y+40)),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0))
               elif (radius<120):
                print 'small'
                cv2.putText(img,"SMALL",(int(x+40),int(y+40)),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0))
               else:
                print 'medium'
                cv2.putText(img,"MEDIUM",(int(x+40),int(y+40)),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0))
               #ser.write('7')
                        
    #Tracking the Blue Color
    (_,contours,hierarchy)=cv2.findContours(blue,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if(area>3500):
               c = max(contours, key = cv2.contourArea)
               (x,y),radius = cv2.minEnclosingCircle(c)
               center = (int(x),int(y))
               radius = int(radius)
               img = cv2.circle(img,center,radius,(0,255,0),2)
               cv2.putText(img,"tu chutiya",(int(x),int(y)),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255))

    #Tracking the orange Color
    (_,contours,hierarchy)=cv2.findContours(orange,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if(area>500):
                    x,y,w,h = cv2.boundingRect(contour)	
                    img = cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
                    cv2.putText(img,"tatti color",(x,y),cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,0,0))  
        
       
    cv2.imshow("Redcolour",red)
    cv2.imshow("bluecolour",blue)
    cv2.imshow("Color Tracking",img)
    cv2.imshow("red",res)
    cv2.imshow("blu",res1)
    if cv2.waitKey(10) & 0xFF == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            break
