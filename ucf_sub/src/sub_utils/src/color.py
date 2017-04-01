from matplotlib import pyplot as plt
import numpy as np
import argparse
import cv2
from imutils import paths
import imutils 
import os
from sklearn.externals import joblib
import time

ap = argparse.ArgumentParser()
ap.add_argument("-p", "--positive", required=True, help="path to positive images directory")
ap.add_argument("-n", "--negative", required=True, help="path to negative images directory")
ap.add_argument("--pf", required = True, help="path to positive features directory")
ap.add_argument("--nf", required = True, help="path to negative feature directory")
args = vars(ap.parse_args())

t0 = time.time()
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
	fd_name = os.path.split(imagePath)[1].split(".")[0] + ".feat"
	fd_path = os.path.join(args["pf"], fd_name)
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
t1 = time.time()
print(t1-t0
print "flattened feature vector size: %d" % (np.array(features).flatten().shape)'''