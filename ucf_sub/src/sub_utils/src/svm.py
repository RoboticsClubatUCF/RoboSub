from sklearn.externals import joblib
from sklearn.model_selection import train_test_split
from sklearn.svm import LinearSVC
from sklearn.model_selection import cross_val_score
import argparse
import glob
import os
import numpy as np

#Argument parsing
ap = argparse.ArgumentParser()
ap.add_argument("-p", "--posfeat", required=True, help="path to positive image hog descriptors")
ap.add_argument("-n", "--negfeat", required=True, help="path to negative image hog descriptors")
args = vars(ap.parse_args())

#Store positive and negative feature directory
pos_feat_path = args["posfeat"]
neg_feat_path = args["negfeat"]

#Lists for the training data and labels
data = []
labels = []

#Load the positive hog descriptors and add to the feature vector and labels
for feat_path in glob.glob(os.path.join(pos_feat_path,"*.feat")):
	d = joblib.load(feat_path)
	data.append(d)
	labels.append(1)

#Load the positive hog descriptors and add to the feature vector and labels
for feat_path in glob.glob(os.path.join(neg_feat_path,"*.feat")):
	d = joblib.load(feat_path)
	data.append(d)
	labels.append(0)

#Convert to numpy
np.asarray(data)
np.asarray(labels)

#Split training set for Cross-Validation
data_train, data_test, labels_train, labels_test = train_test_split(data, labels, test_size = 0.4, random_state = 0)

#Create a Linear SVM and train it
clf = LinearSVC()
print "Training a Linear SVM Classifier"
clf.fit(data_train, labels_train)
print "Model trained"

#Perform cross validation
scores = cross_val_score(clf, data_train, labels_train, cv=5, scoring = 'f1_macro')
print("Accuracy: %0.2f (+/- %0.2f)" % (scores.mean(), scores.std() * 2))

#Dump model and weights
joblib.dump(clf, "hogmodel.pkl")
print "Classifier saved as model.pkl"
weights = clf.coef_
joblib.dump(weights, "hogweights.pkl")
print "Coefficients saved as weights.pkl"