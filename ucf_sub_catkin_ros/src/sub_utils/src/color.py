from matplotlib import pyplot as plt
import numpy as np
import argparse
import cv2
from imutils import paths
import imutils 
import os
from sklearn.externals import joblib


#Argument Parsing
ap = argparse.ArgumentParser()
ap.add_argument("-p", "--positive", required=True, help="path to positive images directory")
ap.add_argument("-n", "--negative", required=True, help="path to negative images directory")
ap.add_argument("--nf", required = True, help="path to negative feature directory")
ap.add_argument("-r", required = True, help="path to red features directory")
ap.add_argument("-g", required = True, help="path to green features directory")
ap.add_argument("-y", required = True, help="path to yellow features directory")
args = vars(ap.parse_args())

'''This Loop goes through every image in the specified directory and does the following:
	1. Read in the image
	2. Resize it to a constant size
	3. Split color channels
	4. Calculate color histogram for each channel and concatenate
	5. Flatten feature vector
	6. Determine which color buoy this is
	7. Dump to appropriate file
''' 
for imagePath in paths.list_images(args["positive"]):
	image = cv2.imread(imagePath)
	image = imutils.resize(image, width=64)
	image = imutils.resize(image, height=64)
	chans = cv2.split(image)
	colors = ("b", "g", "r")
	'''plt.figure()
	plt.title("'Flattened' Color Histogram")
	plt.xlabel("Bins")
	plt.ylabel("# of Pixels")'''
	features = []

	for (chan, color) in zip(chans, colors):
		hist = cv2.calcHist([chan], [0], None, [256], [0, 256])
		features.extend(hist)

	features = np.asarray(features)
	features = features.flatten()
	if("R" in imagePath):
		fd_name = os.path.split(imagePath)[1].split(".")[0] + ".feat"
		fd_path = os.path.join(args["r"], fd_name)
		joblib.dump(features, fd_path)

	if("Y" in imagePath):
		fd_name = os.path.split(imagePath)[1].split(".")[0] + ".feat"
		fd_path = os.path.join(args["y"], fd_name)
		joblib.dump(features, fd_path)

	if("G" in imagePath):
		fd_name = os.path.split(imagePath)[1].split(".")[0] + ".feat"
		fd_path = os.path.join(args["g"], fd_name)
		joblib.dump(features, fd_path)		

for imagePath in paths.list_images(args["negative"]):
	image = cv2.imread(imagePath)
	image = imutils.resize(image, width=64)
	image = imutils.resize(image, height=64)
	chans = cv2.split(image)
	colors = ("b", "g", "r")
	'''plt.figure()
	plt.title("'Flattened' Color Histogram")
	plt.xlabel("Bins")
	plt.ylabel("# of Pixels")'''
	features = []

	for (chan, color) in zip(chans, colors):
		hist = cv2.calcHist([chan], [0], None, [256], [0, 256])
		features.extend(hist)

	features = np.asarray(features)
	features = features.flatten()
	fd_name = os.path.split(imagePath)[1].split(".")[0] + ".feat"
	fd_path = os.path.join(args["nf"], fd_name)
	joblib.dump(features, fd_path)


'''plt.plot(hist, color = color)
plt.xlim([0, 256])
plt.show()
print "flattened feature vector size: %d" % (np.array(features).flatten().shape)'''