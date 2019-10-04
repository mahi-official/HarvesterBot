#classes and subclasses to import
import cv2
import numpy as np
import os

#################################################################################################
#subroutine to write rerults to a csv
def writecsv(color,shape,size,count):
    #open csv file in append mode
    filep = open('results.csv','a')
    # create string data to write per image
    datastr = "," + color + "-" + shape + "-" + size + "-" + count
    #write to csv
    filep.write(datastr)
    filep.close()

#####################################################################################################

def colors(image, c):

    # initialize the colors
    colors = {"Red": (255, 0, 0), "Green": (0, 255, 0), "Blue": (0, 0, 255), "Yellow": (255, 255, 0), "Orange": (255,165,0) }

    # allocate memory for the L*a*b* image, then initialize the color names list
    lab = np.zeros((len(colors), 1, 3), dtype="uint8")
    colorNames = []                                     

    # loop over the colors
    for (i, (name, rgb)) in enumerate(colors.items()):

            # update the Lab array and the color names list
            lab[i] = rgb
            colorNames.append(name)

    # convert the array from the RGB color space to Lab
    lab = cv2.cvtColor(lab, cv2.COLOR_RGB2LAB)
    
    # construct a mask for the contour, then find the average Lab value for the masked region
    mask = np.zeros(image.shape[:2], dtype="uint8")
    cv2.drawContours(mask, [c], -1, 255, -1)
    mean = cv2.mean(image, mask=mask)[:3]

    # initialize the minimum distance 
    minDist = (np.inf, None)

    # loop over the known Lab color values
    for (i, row) in enumerate(lab):

            # compute the distance between colours
            d = np.linalg.norm(row[0] - mean)
            
            # if the distance is smaller than the current distance,
            # then update the bookkeeping variable
            if d < minDist[0]:
                    minDist = (d, i)

    # return the name of the color with the smallest distance
    return colorNames[minDist[1]]

###############################################################################################
	    
def shapes(c):

    # initialize the shape name and approximate the contour
    shape = "Unidentified"
    peri = cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, 0.04 * peri, True)

    # if the shape is a triangle, it will have 3 vertices
    if len(approx) == 3:
            shape = "Triangle"

    # if the shape has 4 vertices, it is either a square or
    # a rectangle
    elif len(approx) == 4:
            # compute the bounding box of the contour and use the
            # bounding box to compute the aspect ratio
            (x, y, w, h) = cv2.boundingRect(approx)
            ar = w / float(h)

            # a square will have an aspect ratio that is approximately
            # equal to one, otherwise, the shape is a rectangle
            shape = "Square" if ar >= 0.95 and ar <= 1.05 else "Rectangle"

    # otherwise, we assume the shape is a circle
    else:
            shape = "Circle"

    # return the name of the shape
    return shape
    
##################################################################################################

def samplearea():
    spath = 'Sample Images'
    images = []
    imarea = []
    #iterate over all the files
    for file in os.listdir(spath):

        #find .png files in folder
        if file.endswith(".png"):
            
            #append files in images array
            images.append(os.path.join(spath,file))
        else:
            continue

    for spath in images:

        #read files from array in gray 
        sgray = cv2.imread(spath,0)

        #find thresh of sample images
        _,sthresh = cv2.threshold(sgray, 245, 255, cv2.THRESH_BINARY_INV)

        #find contours of thresh
        _,scnts,shierarchy = cv2.findContours(sthresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        #find area of contours
        sarea = cv2.contourArea(scnts[0])
        #print sarea
        #append area in list
        imarea.append(sarea)
        
        #display image
        #cv2.imshow(spath,sthresh)
        key = cv2.waitKey(0)
        if key == 27:
            break
    return imarea

##################################################################################################

def sizes(shape,c):

    a = samplearea()
    area = cv2.contourArea(c)
    csize = "Undefined"
    
    if shape is 'Circle':
        if area >= a[0]:
            csize = "Large"
        elif area <= a[1]:
            csize = "Small"
        else:
            csize = "Medium"
    elif shape is 'Rectangle':
        if area >= a[2]:
            csize = "Large"
        elif area <= a[3]:
            csize = "Small"
        else:
            csize = "Medium"
    elif shape is 'Square':
        if area >= a[4]:
            csize = "Large"
        elif area <= a[5]:
            csize = "Small"
        else:
            csize = "Medium"
    elif shape is 'Triangle':
        if area >= a[6]:
            csize = "Large"
        elif a <= a[7]:
            csize = "Small"
        else:
            csize = "Medium"
    else:
        print "Exception Occoured"

    return csize

##################################################################################################

i=1
def main(path):
    global i
        
    # load the image from folder
    image = cv2.imread(path)

    #Convert to gray Scale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)

    #finding threshold of the blur image
    thresh = cv2.adaptiveThreshold(gray, 255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY_INV, 11, 2)
    
    # find contours in the thresholded image
    _,contour,hierarchy = cv2.findContours(thresh.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    #selecting the contour
    x = []
    y = []
    arr = []
    
    for c in contour:
            M = cv2.moments(c)
            cX = int((M["m10"] / M["m00"]))
            cY = int((M["m01"] / M["m00"]))
            if cX not in y:
                y.append(cX)
                x.append(c)
            
    # looping over the selected contours
    for c in x:

        count = 0
        # compute the center of the contour
        M = cv2.moments(c)
        fcx = int((M["m10"] / M["m00"]))
        fcy = int((M["m01"] / M["m00"]))

        # detect the shape of the contour and label the color
        shape = shapes(c)
        color = colors(lab, c)
        size = sizes(shape,c)

        #detect the count of similar shapes
        for d in x:
            shp = shapes(d)
            clr = colors(lab,d)
            siz = sizes(shp,d)
            if (shape == shp and color == clr and size == siz):
                count = count+1

        # create list for storing shape, size, color, count of a single contour
        ap = []
        ap.append(color)
        ap.append(shape)
        ap.append(size)
        ap.append(count)

        # append the list to a list which stores output of all contours
        if ap not in arr:
            arr.append([ j for j in ap])

        #draw the contours and the name of the shape and color on the image
        font = cv2.FONT_HERSHEY_SIMPLEX
        text = "{}-{}-{}-{}".format(color, shape, size,count)
        cv2.drawContours(image, [c], -1, (0, 0, 0), 1)
        cv2.putText(image, text, (fcx-60, fcy),font, 0.4, (0, 0, 0), 1)

    #write output to csv file
    for v in range(0,len(arr)):
            writecsv(arr[v][0],arr[v][1],arr[v][2],str(arr[v][3]))

    #save output images
    cv2.imwrite('Output{}.png'.format(i),image)
    i = i+1

#################################################################################################
#main where the path is set for the directory containing the test images
if __name__ == "__main__":
    mypath = '.\\'
    #getting all files in the directory
    onlyfiles = [os.path.join(mypath, f) for f in os.listdir(mypath) if f.endswith(".png")]
    #iterate over each file in the directory
    for fp in onlyfiles:
        #Open the csv to write in append mode
        filep = open('results.csv','a')
        #this csv will later be used to save processed data, thus write the file name of the image 
        filep.write(fp)
        #close the file so that it can be reopened again later
        filep.close()
        #process the image
        data = main(fp)
        print data
        #open the csv
        filep = open('results.csv','a')
        #make a newline entry so that the next image data is written on a newline
        filep.write('\n')
        #close the file
        filep.close()
