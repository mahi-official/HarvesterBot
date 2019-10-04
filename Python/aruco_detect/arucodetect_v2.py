#############################################################################################################################
import numpy as np
import cv2
import cv2.aruco as aruco
import aruco_lib as ar

#############################################################################################################################
# initialize the colors
colors = {"Red": (255, 0, 0), "Green": (0, 255, 0), "Blue": (0, 0, 255), "Yellow":(255,255,0), "Orange":(255,165,0)}

# allocate memory for the L*a*b* image, then initialize the color names list
lab = np.zeros((len(colors), 1, 3), dtype="uint8")
colorNames = []

# loop over the colors
for (i, (name, rgb)) in enumerate(colors.items()):

        # update the Lab array and the color names list
        lab[i] = rgb
        colorNames.append(name)
        
# convert the array from the RGB color space to LAB
color_array = cv2.cvtColor(lab, cv2.COLOR_RGB2LAB)

#############################################################################################################################
dictionaries = ["DICT_4X4_50",
                "DICT_4X4_100",
                "DICT_4X4_250",
                "DICT_4X4_1000",
                "DICT_5X5_50",
                "DICT_5X5_100",
                "DICT_5X5_250",
                "DICT_5X5_1000",
                "DICT_6X6_50",
                "DICT_6X6_100",
                "DICT_6X6_250",
                "DICT_6X6_1000",
                "DICT_7X7_50",
                "DICT_7X7_100",
                "DICT_7X7_250",
                "DICT_7X7_1000",
                "DICT_ARUCO_ORIGINAL"]

colours = ["Red","Green", "Blue", "Yellow", "Orange"]

geometory  = ["Triangle", "Square","Circle","Ellipse"]

#############################################################################################################################
#write results to csv
def writecsv(data):
    #open csv file in append mode
    filep = open('#40_Task1.2.csv','a')
    #write to csv
    filep.write(data)
    filep.close()

#############################################################################################################################
def colors(image, cnts):

    global color_array
    global colorNames

    # construct a mask for the contour, then find the average Lab value for the masked region
    mask = np.zeros(image.shape[:2], dtype="uint8")
    cv2.drawContours(mask, [cnts], -1, 255, -1)
    mean = cv2.mean(image, mask=mask)[:3]

    # initialize the minimum distance 
    minDist = (np.inf, None)

    # loop over the known Lab color values
    for (i, row) in enumerate(color_array):

            # compute the distance between colours
            dist = np.linalg.norm(row[0] - mean)
            
            # if the distance is smaller than the current distance,
            # then update the bookkeeping variable
            if dist < minDist[0]:
                    minDist = (dist, i)

    # return the name of the color with the smallest distance
    return colorNames[minDist[1]]

#############################################################################################################################
def shapes(c):
    
        # initialize the shape name and approximate the contour
        shape = "Unidentified"
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.04 * peri, True)

        # if the shape is a triangle, it will have 3 vertices
        if len(approx) == 3:
                shape = "Triangle"

        # if the shape has 4 vertices, it is either a square or a rectangle
        elif len(approx) == 4:
                # compute the bounding box of the contour and use the
                # bounding box to compute the aspect ratio
                (x, y, w, h) = cv2.boundingRect(approx)
                ar = w / float(h)

                # a square will have an aspect ratio that is approximately
                # equal to one, otherwise, the shape is a rectangle
                shape = "Square" if ar >= 0.95 and ar <= 1.05 else "Rectangle"

        # if the shape is a circle, it will have MajorAxis/MinorAxis = 1
        else:
            (x,y),(MA,ma),angle = cv2.fitEllipse(c)
            ar = MA/float(ma)
            shape = "Circle" if ar >= 0.95 and ar <= 1.05 else "Ellipse"

        # return the name of the shape
        return shape

#############################################################################################################################
def aruco_detect(mask, dictionary):
    id_aruco_trace = 0
    det_aruco_list = {}
    det_aruco_list = ar.detect_Aruco(mask,dictionary)
    if det_aruco_list:
        img = ar.mark_Aruco(mask,det_aruco_list)
        id_aruco_trace = ar.calculate_Robot_State(img,det_aruco_list)
        print(id_aruco_trace)
        ar_id = list(id_aruco_trace.keys())
        writecsv(str(ar_id[0]))
        cv2.imshow('Marker',img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

#############################################################################################################################
count = 1
def detect(image_path,dictionary,clr,shp):

    extra = []
    contours= []
    
    # load the image
    image = cv2.imread(image_path)
    
    #Convert to gray Scale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)

    #blur the image
    blur = cv2.bilateralFilter(gray,9,80,80)

    #finding threshold of the blur image
    ret,thresh = cv2.threshold(blur,250,255,cv2.THRESH_BINARY_INV)
    
    # find contours in the thresholded image
    img, contour, hierarchy = cv2.findContours(thresh.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    
    #look for parent-child contours for finding Aruco Marker using heirarchy
    for x in range(0,len(hierarchy[0])):

        #when child is found it's value is non-negative
        if(hierarchy[0][x][2] != -1):

            c = contour[x]  #Parent contour
            extra.append(c)

            #find all it's child and remove them before shape detection
            for y in range(x,len(hierarchy[0])):
                if(hierarchy[0][y][3] == x):
                    d = contour[y]
                    extra.append(d)

            #find the bounding box of parent contour
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.1 * peri, True)
            (x, y, w, h) = cv2.boundingRect(approx)

            #mask that area for finding aruco ids and angle
            mask = image[ x:w+20, y:h+20]
            aruco_detect(mask,dictionary)

    for ac in contour:
        if ac not in extra:
            contours.append(ac)
            
    # looping over the contours
    for cnts in contours:

        # compute the center of the contour
        M = cv2.moments(cnts)
        fcx = int((M["m10"] / (M["m00"]+1)))
        fcy = int((M["m01"] / (M["m00"]+1)))

        # detect the shape of the contour and label the color
        shape = shapes(cnts)
        color = colors(lab, cnts)

        #add input shape constraint 
        for i in range(0,len(shp)):
            
            if (shape == shp[i] and color == clr[i]):
                
                #draw the contours and the name of the shape and labeled color on the image
                font = cv2.FONT_HERSHEY_SIMPLEX
                text = "({}-{})".format(fcx, fcy)
                writecsv(text)

                if(color == "Red"):
                    cv2.drawContours(image, [cnts], -1, (0, 255, 0), 25)
                if(color == "Green"):
                    cv2.drawContours(image, [cnts], -1, (255, 0, 0), 25)
                if(color == "Blue"):
                    cv2.drawContours(image, [cnts], -1, (0, 0, 255), 25)
                if(color == "Yellow"):
                    cv2.drawContours(image, [cnts], -1, (0, 165, 255), 25)
                if(color == "Orange"):
                    cv2.drawContours(image, [cnts], -1, (0, 255, 255), 25)

                cv2.putText(image, text, (fcx, fcy),font, 0.4, (0, 0, 0), 1)
                
                cv2.imshow("Image",image)
                cv2.waitKey(0)
                cv2.destroyAllWindows()

    #save output images
    cv2.imwrite('ArUco{}.jpg'.format(count),image)
    count+=1

#############################################################################################################################

if __name__ == "__main__":

    print("How many images are to be detected (ex: 5)")
    n = input()
    
    writecsv("Image Names,")
    writecsv("ArUco ID,")
    writecsv("(x-y)Obj1,")
    writecsv("(x-y)Obj2,")
    writecsv('\n')
    
    for i in range(0, int(n)):
        path = "Images/Image{}.jpg".format(int(i)+1)
        
        writecsv(path.split("/")[-1])

        print(" 1. DICT_4X4_50\n",
              "2. DICT_4X4_100\n",
              "3. DICT_4X4_250\n",
              "4. DICT_4X4_1000\n",
              "5. DICT_5X5_50\n",
              "6. DICT_5X5_100\n",
              "7. DICT_5X5_250\n",
              "8. DICT_5X5_1000\n",
              "9. DICT_6X6_50\n",
              "10. DICT_6X6_100\n",
              "11. DICT_6X6_250\n",
              "12. DICT_6X6_1000\n",
              "13. DICT_7X7_50\n",
              "14. DICT_7X7_100\n",
              "15. DICT_7X7_250\n",
              "16. DICT_7X7_1000\n",
              "17. DICT_ARUCO_ORIGINAL\n")

        print("Enter the dictionary to which marker in Image{} belongs(ex: 2)".format(i+1))
        d = input()
        dictionary = dictionaries[int(d)-1]
        
        print(" 1. Red\n","2. Green\n","3. Blue\n","4. Yellow\n","5. Orange\n")
        print("Enter color of objects in image to be found respective to shapes(ex:1 2 3 4 5)")
        z = input()
        z = z.split(" ")
        clr = []
        for i in z:
            clr.append(colours[int(i)-1])
        print(clr)

        print(" 1. Triangle\n", "2. Square\n","3. Circle\n","4. Ellipse\n")
        print("Enter the shape of objects to be found respective to colors(ex:1 2 3 4)")
        x = input()
        x = x.split(" ")
        shp = []
        for i in x:
            shp.append(geometory[int(i)-1])
        print(shp)
        
        #process the image
        detect(path,dictionary,clr,shp)

        writecsv('\n')
