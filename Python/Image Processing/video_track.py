#importing modules
import cv2
import numpy as np
import serial
import time
import RPi.GPIO as GPIO

ser = serial.Serial('/dev/ttyUSB0', 9600)

img = hsv =res =None
cap = cv2.VideoCapture(0)
cap.set(3,320)
cap.set(4,240)
GPIO.setmode(GPIO.BCM)
GPIO.setup(4, GPIO.IN,pull_up_down=GPIO.PUD_DOWN)

def click_image():
  global img, hsv
  #skipping frames for great processing
  for x in xrange(1,30):
    ret, img = cap.read()
  #Showing Image
  cv2.imshow("image", img)
  cv2.imwrite("/home/pi/Desktop/image.png",img)
  #converting frame(img i.e BGR) to HSV (hue-saturation-value)
  hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
  cap.release()
  return img



def detect_red(img):
  #img=cv2.imread("/home/pi/Desktop/image.png")
  #Return Variable
  ret = None
  #definig the range of red color
  red_lower=np.array([165,100,135],np.uint8)
  red_upper=np.array([180,190,255],np.uint8)
  #finding the range of red color in the image
  red=cv2.inRange(hsv, red_lower, red_upper)
  #Morphological transformation, Dilation   
  kernal = np.ones((5 ,5), "uint8")
  red=cv2.dilate(red, kernal)
  res=cv2.bitwise_and(img, img, mask = red)
  #Tracking the Red Color
  (contours,_)=cv2.findContours(red,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
  for pic, contour in enumerate(contours):
    area = cv2.contourArea(contour)
    if(area>3500):
      c = max(contours, key = cv2.contourArea)
      (x,y),radius = cv2.minEnclosingCircle(c)
      center = (int(x),int(y))
      radius = int(radius)
      #ser.write('8')
      print center
      img = cv2.circle(res,center,radius,(0,255,0),2)
      if (radius>200):
        ret = 1
        cv2.putText(res,"Apple (L)",(int(x+40),int(y+40)),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0))
      elif (radius<120):
        ret = -1
        cv2.putText(res,"Apple (S)",(int(x+40),int(y+40)),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0))
      else:
        ret = 0
        cv2.putText(res,"Apple (M)",(int(x+40),int(y+40)),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0))
  cv2.imshow("Red-Color",res)
  cv2.imwrite("/home/pi/Desktop/res.png",res)
  return ret


def detect_blue(img):
  #img=cv2.imread("/home/pi/Desktop/image.png")
  ret = None
  #defining the Range of Blue color
  blue_lower=np.array([100,110,130],np.uint8)
  blue_upper=np.array([110,255,255],np.uint8)
  #finding the range of blue color in the image
  blue=cv2.inRange(hsv,blue_lower,blue_upper)
  #Morphological transformation, Dilation   
  kernal = np.ones((5 ,5), "uint8")
  blue=cv2.dilate(blue,kernal)
  res1=cv2.bitwise_and(img, img, mask = blue)
  #Tracking the Blue Color
  (contours,_)=cv2.findContours(blue,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
  for pic, contour in enumerate(contours):
    area = cv2.contourArea(contour)
    if(area>3500):
      c = max(contours, key = cv2.contourArea)
      (x,y),radius = cv2.minEnclosingCircle(c)
      center = (int(x),int(y))
      radius = int(radius)
      img = cv2.circle(res1,center,radius,(0,255,0),2)
      cv2.putText(res,"Blueberry",(int(x),int(y)),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255))
      if (radius>200):
        ret = 1
        cv2.putText(res1,"LARGE",(int(x+40),int(y+40)),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0))
      elif (radius<120):
        ret = -1
        cv2.putText(res1,"SMALL",(int(x+40),int(y+40)),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0))
      else:
        ret = 0
        cv2.putText(res1,"MEDIUM",(int(x+40),int(y+40)),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0))
  cv2.imshow("bluecolor",res1)
  return ret
        
def detect_orange(img):
  #img=cv2.imread("/home/pi/Desktop/image.png")
  ret = None
  #defining the Range of orange color
  orange_lower=np.array([0,49,163],np.uint8)
  orange_upper=np.array([114,195,255],np.uint8)
  #finding the range of orange color in the image
  orange=cv2.inRange(hsv,orange_lower,orange_upper)
  #Morphological transformation, Dilation   
  kernal = np.ones((5 ,5), "uint8")
  orange=cv2.dilate(orange,kernal)
  res2=cv2.bitwise_and(img, img, mask = orange)    
  #Tracking the orange Color
  (contours,_)=cv2.findContours(orange,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
  for pic, contour in enumerate(contours):
    area = cv2.contourArea(contour)
    if(area>3500):
      c = max(contours, key = cv2.contourArea)
      (x,y),radius = cv2.minEnclosingCircle(c)
      center = (int(x),int(y))
      radius = int(radius)
      img = cv2.circle(res2,center,radius,(0,255,0),2)
      cv2.putText(res2,"Orange",(int(x),int(y)),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255))
      if (radius>200):
        ret = 1
        cv2.putText(res2,"LARGE",(int(x+40),int(y+40)),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0))
      elif (radius<120):
        ret = -1
        cv2.putText(res2,"SMALL",(int(x+40),int(y+40)),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0))
      else:
        ret = 0
        cv2.putText(res2,"MEDIUM",(int(x+40),int(y+40)),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0))
  cv2.imshow("Orange-Color",res2)
  return ret


fruit_table = [
  [
    ["APPLE", -1, 1],
    ["APPLE", 0, 1],
    ["APPLE", 1, 1]
  ], [
    ["BLUEBERRY", -1, 1],
    ["BLUEBERRY", 0, 1],
    ["BLUEBERRY", 1, 1]
  ], [
    ["ORANGE", -1, 0],
    ["ORANGE", 0, 2],
    ["ORANGE", 1, 0]
  ]
]

fruit_check = fruit_table

def pluck_fruit(fruit, size):
  global fruit_check, fruit_table;
  for x in range(0,len(fruit_table)):
    for y in range(0,len(fruit_table[x])):
      if fruit_table[x][y][0] == fruit and fruit_table[x][y][1] == size:
          if fruit_check[x][y][2] > 0:
            #Sending Pluck Signal
            print(fruit + " " + str(size) + " " + str(fruit_check[x][y][2]) + " BEFORE");
            fruit_check[x][y][2] -= 1; 
            print(fruit + " " + str(size) + " " + str(fruit_check[x][y][2]) + " AFTER");
          else:
            #Sending Move Next Node Signal 
            print(fruit + " " + str(size) + " " + str(fruit_check[x][y][2]) + " ERROR");
            print("Fruit not required");

def print_fruit_check():
  string = "";
  for x in range(0,len(fruit_check)):
    for y in range(0,len(fruit_check[x])):
      for z in range(0,len(fruit_check[x][y])):
        string = string + str(fruit_check[x][y][z]) + " ";
      print(string);
      string = "";

def detect_fruit(image):
  print_fruit_check() 
  size = None
  fruit = None
  if detect_red(image) != None:
    size = detect_red(image)
    fruit = "APPLE"
  if detect_blue(image) != None:
    size = detect_blue(image)
    fruit = "BLUEBERRY"
  if detect_orange(image) != None:
    size = detect_orange(image)
    fruit = "ORANGE"
  if fruit != None and size != None:
    pluck_fruit(fruit, size)
    print_fruit_check()

def live_feed():
  cap = cv2.VideoCapture(0)
  cap.set(3,320)
  cap.set(4,240)

  while(1):
      _, frame = cap.read()
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
      res0=res+res1+res2


      #Tracking the Red Color
      (contours,_)=cv2.findContours(red,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

      for pic, contour in enumerate(contours):
              area = cv2.contourArea(contour)
              if(1000>area>500):
                 c = max(contours, key = cv2.contourArea)
                 (x,y),radius = cv2.minEnclosingCircle(c)
                 center = (int(x),int(y))
                 radius = int(radius)
                 img = cv2.circle(res0,center,radius,(0,255,0),2)
                 cv2.putText(res0,"APPLE",(int(x),int(y)),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0))
                 print radius
                 if (radius>200):
                  print 'large'
                  cv2.putText(res0,"LARGE",(int(x+40),int(y+40)),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0))
                 elif (radius<120):
                  print 'small'
                  cv2.putText(res0,"SMALL",(int(x+40),int(y+40)),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0))
                 else:
                  print 'medium'
                  cv2.putText(res0,"MEDIUM",(int(x+40),int(y+40)),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0))
                 #ser.write('7')
                          
      #Tracking the Blue Color
      (contours,_)=cv2.findContours(blue,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
      for pic, contour in enumerate(contours):
              area = cv2.contourArea(contour)
              if(area>1500):
                 c = max(contours, key = cv2.contourArea)
                 (x,y),radius = cv2.minEnclosingCircle(c)
                 center = (int(x),int(y))
                 radius = int(radius)
                 img = cv2.circle(res0,center,radius,(0,255,0),2)
                 cv2.putText(res0,"blue",(int(x),int(y)),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255))
                 print radius
                 if (radius>200):
                  print 'large'
                  cv2.putText(res0,"LARGE",(int(x+40),int(y+40)),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0))
                 elif (radius<120):
                  print 'small'
                  cv2.putText(res0,"SMALL",(int(x+40),int(y+40)),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0))
                 else:
                  print 'medium'
                  cv2.putText(res0,"MEDIUM",(int(x+40),int(y+40)),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0))

      #Tracking the orange Color
      (contours,_)=cv2.findContours(orange,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
      for pic, contour in enumerate(contours):
              area = cv2.contourArea(contour)
              if(area>1500):
                 c = max(contours, key = cv2.contourArea)
                 (x,y),radius = cv2.minEnclosingCircle(c)
                 center = (int(x),int(y))
                 radius = int(radius)
                 img = cv2.circle(res0,center,radius,(0,255,0),2)
                 cv2.putText(img,"orange",(int(x),int(y)),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255))
                 print radius
                 if (radius>200):
                  print 'large'
                  cv2.putText(res0,"LARGE",(int(x+40),int(y+40)),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0))
                 elif (radius<120):
                  print 'small'
                  cv2.putText(res0,"SMALL",(int(x+40),int(y+40)),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0))
                 else:
                  print 'medium'
                  cv2.putText(res0,"MEDIUM",(int(x+40),int(y+40)),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0))
                  
      #cv2.imshow("Redcolour",res)
      #cv2.imshow("bluecolour",res1)
      cv2.imshow("Live Input",frame)
      #cv2.imshow("red",res)
      cv2.imshow("Color",res0)
      if cv2.waitKey(10) & 0xFF == ord('q'):
              cap.release()
              cv2.destroyAllWindows()
              break

 
def call_main():
  while 1:
    if  GPIO.input(4)==True:
      image=click_image()
      detect_fruit(image)
      live_feed()
      cv2.waitKey(0)
      time.sleep(10)
      cv2.destroyAllWindows()

    else:
      print ("No Signal")
      time.sleep(1)
    time.sleep(12)
call_main()

  
