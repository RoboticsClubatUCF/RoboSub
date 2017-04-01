import argparse
import cv2
from imutils import paths
import imutils 
from sklearn.externals import joblib
import os

#Allows user to specify image path and config file
ap = argparse.ArgumentParser()
ap.add_argument("-p", "--positive", required=True, help="path to positive images directory")
ap.add_argument("-n", "--negative", required=True, help="path to negative images directory")
ap.add_argument("-c", "--config", required=True, help="path to config file")
ap.add_argument("--pf", required = True, help="path to positive features directory")
ap.add_argument("--nf", required = True, help="path to negative feature directory")
args = vars(ap.parse_args())


'''Initialize HOGDescriptor with config file specified by the config argument. 
   As far as I can tell the only constructor for the Hog Descriptors in the python bindings to specify parameters is this one
'''
hog = cv2.HOGDescriptor(args["config"])



'''This Loop goes through every image in the specified directory and does the following:
	1. Read in the image
	2. Resize it to a constant size
	3. Compute Hog Descriptors
	4. Flatten to a 1D array
	5. Dump to a file
''' 
for imagePath in paths.list_images(args["positive"]):
	image = cv2.imread(imagePath)
	image = imutils.resize(image, width=64)
	image = imutils.resize(image, height=64)
	descriptors = hog.compute(image)
	#For some reason that I'm not sure of the Hog Descriptors return an array of size (descriptorsize, 1) but the SVM will want a 1D array
	#flat_descriptors = descriptors.flatten()
	flat_descriptors = descriptors.flatten()
	fd_name = os.path.split(imagePath)[1].split(".")[0] + ".feat"
	fd_path = os.path.join(args["pf"], fd_name)
	joblib.dump(flat_descriptors, fd_path)

for imagePath in paths.list_images(args["negative"]):
	image = cv2.imread(imagePath)
	image = imutils.resize(image, width=64)
	image = imutils.resize(image, height=64)
	descriptors = hog.compute(image)
	flat_descriptors = descriptors.flatten()
	fd_name = os.path.split(imagePath)[1].split(".")[0] + ".feat"
	fd_path = os.path.join(args["nf"], fd_name)
	joblib.dump(flat_descriptors, fd_path)