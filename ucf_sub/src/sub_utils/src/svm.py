from sklearn.externals import joblib
from sklearn.model_selection import train_test_split
from sklearn.svm import LinearSVC
from sklearn.model_selection import cross_val_score
import argparse
import glob
import os
import numpy as np


ap = argparse.ArgumentParser()
ap.add_argument("-p", "--posfeat", required=True, help="path to positive image hog descriptors")
ap.add_argument("-n", "--negfeat", required=True, help="path to negative image hog descriptors")
args = vars(ap.parse_args())

pos_feat_path = args["posfeat"]
neg_feat_path = args["negfeat"]

fds = []
labels = []
for feat_path in glob.glob(os.path.join(pos_feat_path,"*.feat")):
	fd = joblib.load(feat_path)
	fds.append(fd)
	labels.append(1)

for feat_path in glob.glob(os.path.join(neg_feat_path,"*.feat")):
	fd = joblib.load(feat_path)
	fds.append(fd)
	labels.append(0)

np.asarray(fds)
np.asarray(labels)

fds_train, fds_test, labels_train, labels_test = train_test_split(fds, labels, test_size = 0.4, random_state = 0)
clf = LinearSVC()
print "Training a Linear SVM Classifier"
clf.fit(fds_train, labels_train)
print "Model trained"
scores = cross_val_score(clf, fds_train, labels_train, cv=5, scoring = 'f1_macro')
print("Accuracy: %0.2f (+/- %0.2f)" % (scores.mean(), scores.std() * 2))
joblib.dump(clf, "model.pkl")
print "Classifier saved as model.pkl"
weights = clf.coef_
joblib.dump(weights, "weights.pkl")
print "Coefficients saved as weights.pkl"