# Made by Matthew Kurtz
from __future__ import division 
import cv2
import numpy as np
import math



# Read image
img = cv2.imread('Left.jpg')

# Convert BGR to HSV
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# Define range of orange in HSV
lower_orange = np.array([0,70,70])
upper_orange = np.array([250,250,300])

# Threshold the HSV image to get only orange colors
mask = cv2.inRange(hsv, lower_orange, upper_orange)

# Bitwise AND mask and original image
res = cv2.bitwise_and(img, img, mask=mask)

# Detect edges
edges = cv2.Canny(mask,50,150,apertureSize = 3)

# Convert edges back to color
color = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

# Probabilistic hough line transform
linesP = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, None, 50, 40)

# Draw the lines
if linesP is not None:
    for i in range(0, len(linesP)):
        l = linesP[i][0]
        #cv.line(image, lines, rho, theta, threshold, minLineLength, maxLineGap)
        cv2.line(color,(l[0], l[1]), (l[2], l[3]), (0,0,255), 0, cv2.LINE_AA)
           
#print linesP
'''for object in linesP:
    theta = object[0][1]
    rho = object[0][0]
    one = object [0][2]
    two = object [0][3]

    print theta, " ",  rho, " ",one, " ",two
'''    

'''
    Everything from here down is error prone and copied from internet and modified
    (link here: https://stackoverflow.com/questions/20677795/how-do-i-compute-the-intersection-point-of-two-lines-in-python )
    ask someone who understands python how to proceed with fixing the current error
'''


# Define paramaters for a line
def line(p1, p2):
    A = (p1[1] - p2[1])
    B = (p2[0] - p1[0])
    C = (p1[0]*p2[1] - p2[0]*p1[1])
    return A, B, -C

# Calculate intersection
def intersection(L1, L2):
    D  = L1[0] * L2[1] - L1[1] * L2[0]
    Dx = L1[2] * L2[1] - L1[1] * L2[2]
    Dy = L1[0] * L2[2] - L1[2] * L2[0]
    if D != 0:
        x = Dx / D
        y = Dy / D
        return x,y
    else:
        return False
        
def polarToCartesianX(r, theta):
    return r*math.cos(theta)
def polarToCartesianY(r, theta):
    return r*math.sin(theta)

# Make an array of all the points, where each index is another array of [x][y] variables
points = []
print type(linesP)
print linesP

'''
# assumming points are in polar coordinates
for point in linesP:
    points.append([polarToCartesianX(point[0][0], point[0][1]), polarToCartesianY(point[0][0], point[0][1])])
    points.append([polarToCartesianX(point[0][2], point[0][3]), polarToCartesianY(point[0][2], point[0][3])])
'''
 
# assuming points are in cartesian coordinates
for point in linesP:
    points.append([point[0][0], point[0][1]])
    points.append([point[0][1], point[0][2]])

#for i in linesP:
    #print object[0][1], object[0][0]
    

#print points
print points[0]
print len(points[0])

for i in range(0, 10, 2):
    print i


#Iterate through all possible combinations of lines and check for intersections
for i in range(0, len(points), 2):
    #define the first line
    L1 = line(points[i], points[i+1])
    #iterate through all the options for the second line intersect before moving on
    for j in range(2, len(points)-1, 2):
        #define second line
        L2 = line(points[j], points[j+1])
        #find the intersection
        R = intersection(L1, L2)
        #Readout of if there is an intersection
        if R is not False:
           # print "Intersection:", R
            color[(R[0].astype(int).item()-1):R[0].astype(int).item(), (R[1].astype(int).item()-1):R[1].astype(int).item()] = (0,255,0)
            #print type(R[0].astype(int).item())
        #print points[i], points[j]
        

print type(color)
print type(img)
print type(300)



# Display images
cv2.imshow('image', img)
#cv2.imshow('edges', edges)
cv2.imshow('cdstP', color)
#cv2.imshow('res', res)
#cv2.imshow('mask', mask)
cv2.waitKey(0)




















