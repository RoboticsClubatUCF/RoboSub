import cv2
import numpy as np
import time 

image = cv2.imread('test.png')
image = cv2.flip(image,2)
rows, cols,_ = image.shape
M = cv2.getRotationMatrix2D((cols/2,rows/2),40,1)
image = cv2.warpAffine(image,M,(cols,rows))
mask=cv2.inRange(image, np.array([0, 180, 255],dtype='uint8'),np.array([0, 180, 255],dtype='uint8')) 
_, cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

t0 = time.time()
size = np.size(mask)
skel = np.zeros(mask.shape,np.uint8)
element = cv2.getStructuringElement(cv2.MORPH_CROSS,(20,20))
done = False

while( not done):
    eroded = cv2.erode(mask,element)
    temp = cv2.dilate(eroded,element)
    temp = cv2.subtract(mask,temp)
    skel = cv2.bitwise_or(skel,temp)
    img = eroded.copy()
    done = True 
    zeros = size - cv2.countNonZero(mask)
    if zeros==size:
        break

t1=time.time()
print(t1-t0)
lines = cv2.HoughLines(skel,1,np.pi/90,52)
print(lines)

if lines[0][0][1] > lines[1][0][1]:
	newPath = lines[0]

else:
	newPath = lines[1]

for i in range(len(lines)):
	for rho,theta in lines[1]:
	    a = np.cos(theta)
	    b = np.sin(theta)
	    x0 = a*rho
	    y0 = b*rho
	    x1 = int(x0 + 1000*(-b))
	    y1 = int(y0 + 1000*(a))
	    x2 = int(x0 - 1000*(-b))
	    y2 = int(y0 - 1000*(a))
	    print(x1,y1,x2,y2)
	    cv2.line(image,(x1,y1),(x2,y2),(0,0,255),2)


cv2.imshow("skel",image)
cv2.waitKey(0)
cv2.destroyAllWindows()