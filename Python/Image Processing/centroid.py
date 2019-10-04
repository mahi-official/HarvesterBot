cx = int(M['m10']/M['m00'])
cy = int(M['m01']/M['m00'])

area = cv2.contourArea(cnt)

perimeter = cv2.arcLength(cnt,True)

epsilon = 0.1*cv2.arcLength(cnt,True)
approx = cv2.approxPolyDP(cnt,epsilon,True)

hull = cv2.convexHull(points[, hull[, clockwise[, returnPoints]]])
#points are the contours we pass into.
#hull is the output, normally we avoid it.
#lockwise : Orientation flag.
#returnPoints : By default,true.
hull = cv2.convexHull(cnt)
