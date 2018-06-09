import numpy as np
import cv2

import heapq

def Thresholds(upperThresh=(255,255,255), lowerThresh=(0,0,0)):
	upper = upperThresh
        lower = lowerThresh

def contourCircularity(contour):
    perimeter = cv2.arcLength(contour, True)
    area = cv2.contourArea(contour)
    
    radius = sqrt(area/3.14159)
    ratio = perimeter/(2*radius*3.14159)
    
    return min(ratio, 1/ratio) 
    
def greatestAreaContour(contours):
    if len(contours) > 0:
        return max(contours, key=cv2.contourArea)

def nlargest(n, contours, key):
    largestContours = []
    if key == cv2.contourArea:
        sortedContours = sorted(contours, key=cv2.contourArea, reverse=True)

        for i in range(n):
            largestContours.append(sortedContours[i])
    return largestContours

def greatestNAreaContours(contours, n):
    return nlargest(n, contours, key=cv2.contourArea)
    
def ThreshAndContour(srcImage, upper, lower):
    imgThreshed = cv2.inRange(srcImage, lower, upper)
    
    kernel = np.ones((5,5),np.uint8)
    opening = cv2.morphologyEx(imgThreshed, cv2.MORPH_OPEN, kernel)
    closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
    
    contours = cv2.findContours(closing, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    return imgThreshed, contours
    

def cartesian(arrays, out=None): #from http://stackoverflow.com/a/1235363
    """
    Generate a cartesian product of input arrays.
    Parameters
    ----------
    arrays : list of array-like
        1-D arrays to form the cartesian product of.
    out : ndarray
        Array to place the cartesian product in.
    Returns
    -------
    out : ndarray
        2-D array of shape (M, len(arrays)) containing cartesian products
        formed of input arrays.
    Examples
    --------
    >>> cartesian(([1, 2, 3], [4, 5], [6, 7]))
    array([[1, 4, 6],
           [1, 4, 7],
           [1, 5, 6],
           [1, 5, 7],
           [2, 4, 6],
           [2, 4, 7],
           [2, 5, 6],
           [2, 5, 7],
           [3, 4, 6],
           [3, 4, 7],
           [3, 5, 6],
           [3, 5, 7]])
    """

    arrays = [np.asarray(x) for x in arrays]
    dtype = arrays[0].dtype

    n = np.prod([x.size for x in arrays])
    if out is None:
        out = np.zeros([n, len(arrays)], dtype=dtype)

    m = n / arrays[0].size
    out[:,0] = np.repeat(arrays[0], m)
    if arrays[1:]:
        cartesian(arrays[1:], out=out[0:m,1:])
        for j in xrange(1, arrays[0].size):
            out[j*m:(j+1)*m,1:] = out[0:m,1:]
    return out
