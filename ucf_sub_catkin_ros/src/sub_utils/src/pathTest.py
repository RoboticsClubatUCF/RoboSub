import numpy as np
import cv2

cap = cv2.VideoCapture('/home/robotics/Videos/2018.07.21-13.02.34_bottom.avi')

while(cap.isOpened()):
	ret, image = cap.read()

	imageHLS = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)

	mask=cv2.inRange(imageHLS, np.array([5, 75, 110],dtype='uint8'),np.array([50, 230, 255],dtype='uint8'))

	rows, cols,_ = image.shape
	_, cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	kernel = np.ones((5,5),np.uint8)
	mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
	output = cv2.bitwise_and(image, image, mask=mask)

	pole = []
	bestFilled = 0
	bestArea = 0

	for c in cnts:
			
			area = cv2.contourArea(c)
			if area < 500:
				continue
			
			rect = cv2.fillPoly(np.zeros(mask.shape), [c], (255))
			rect = cv2.bitwise_and(rect, rect, mask=mask)
			filled = cv2.countNonZero(rect)
			if area/filled > bestFilled:
				bestFilled = area/filled
				bestArea = area
				pole = c
			elif area/filled >= (0.75 * bestFilled) and area > bestArea:
				bestFilled = area/filled
				bestArea = area
				pole = c

	output = cv2.fillPoly(output, [np.int0(cv2.boxPoints(cv2.minAreaRect(pole)))], (255,255,255))
	output = cv2.bitwise_and(output, output, mask=mask)

	output = cv2.drawContours(output, [c for c in cnts if cv2.contourArea(c) > 500], -1, (0,255,0),1)
	output = cv2.drawContours(output, [cv2.approxPolyDP(c, 0.001*cv2.arcLength(c, True), False) for c in cnts if cv2.contourArea(c) > 500], -1, (255,0,0),1)

	rects = [cv2.boundingRect(c) for c in cnts if cv2.contourArea(c) > 100]
	for r in rects:
		output = cv2.rectangle(output, (r[0], r[1]), (r[0]+r[2], r[1]+r[3]), (255,0,0))

	size = np.size(mask)
	skel = np.zeros(mask.shape,np.uint8)
	element = cv2.getStructuringElement(cv2.MORPH_CROSS,(20,20))
	done = False

	while(not done):
	    eroded = cv2.erode(mask,element)
	    temp = cv2.dilate(eroded,element)
	    temp = cv2.subtract(mask,temp)
	    skel = cv2.bitwise_or(skel,temp)
	    img = eroded.copy()
	    done = True 
	    zeros = size - cv2.countNonZero(mask)
	    if zeros==size:
	        break

	lines = cv2.HoughLines(skel,1,np.pi/90,52)
	try:
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
			    cv2.line(image,(x1,y1),(x2,y2),(0,0,255),2)

	except:
		pass

	fourcc = cv2.VideoWriter_fourcc(*'XVID')
	out = cv2.VideoWriter('output.avi',fourcc, 20.0, (640,480))
	#out.write(image)
	cv2.imshow('frame', image)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

cap.release()
cv2.destroyAllWindows()