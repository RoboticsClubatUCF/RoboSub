from sklearn.externals import joblib
from sklearn.model_selection import train_test_split
from sklearn.svm import LinearSVC
from sklearn.model_selection import cross_val_score
import argparse
import glob
import os
import numpy as np

#Parse in arguments
ap = argparse.ArgumentParser()
ap.add_argument("-r", "--rfeat", required=True, help="path to red buoy histograms")
ap.add_argument("-g", "--gfeat", required=True, help="path to green buoy histograms")
ap.add_argument("-y", "--yfeat", required=True, help="path to yellow buoy histograms")
ap.add_argument("-n", "--negfeat", required=True, help="path to negative image histograms")
args = vars(ap.parse_args())

#Store feature paths for ease of use later on
red_feat_path = args["rfeat"]
green_feat_path = args["gfeat"]
yellow_feat_path = args["yfeat"]
neg_feat_path = args["negfeat"]

#Lists for the training data and labels
data = []
labels = []

#Load histograms of negative features. This is class 0
for feat_path in glob.glob(os.path.join(neg_feat_path,"*.feat")):
	d = joblib.load(feat_path)
	data.append(d)
	labels.append(0)

#Loead red histograms. This is class 1
for feat_path in glob.glob(os.path.join(red_feat_path,"*.feat")):
	d = joblib.load(feat_path)
	data.append(d)
	labels.append(1)

#Load green histograms. This is class 2
for feat_path in glob.glob(os.path.join(green_feat_path,"*.feat")):
	d = joblib.load(feat_path)
	data.append(d)
	labels.append(2)

#Load yellow histograms this is class 3
for feat_path in glob.glob(os.path.join(yellow_feat_path,"*.feat")):
	d = joblib.load(feat_path)
	data.append(d)
	labels.append(3)

#Convert to numpy
np.asarray(data)
np.asarray(labels)

#Split training for cross-validation
data_train, data_test, labels_train, labels_test = train_test_split(data, labels, test_size = 0.4, random_state = 0)

#Create a Linear SVM and train it
clf = LinearSVC()
print "Training a Linear SVM Classifier"
clf.fit(data_train, labels_train)
print "Model trained"

#Perform cross validation and display mean and std
scores = cross_val_score(clf, data_test, labels_test, cv=5, scoring = 'f1_macro')
print("Accuracy: %0.2f (+/- %0.2f)" % (scores.mean(), scores.std() * 2))

#Save models and weights
joblib.dump(clf, "modelcol.pkl")
print "Classifier saved as model.pkl"
weights = clf.coef_
joblib.dump(weights, "weightscol.pkl")
print "Coefficients saved as weights.pkl"