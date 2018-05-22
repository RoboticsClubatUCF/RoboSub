import matplotlib.pyplot as plt
import numpy as np
import os
from skimage.color import rgb2gray
from skimage.feature import hog
from skimage.io import imread, imshow
from skimage.transform import resize
from sklearn.externals import joblib
from skimage import exposure
import time


orientations = 9
feature_vector = True
normalize = True
visualize = False
block_norm = 'L2-Hys'
cells_per_block = [2, 2]
pixels_per_cell = [20, 20]


def getData(filePath, label):
    fileNames = []
    images = []
    num = 0
    for childDir in os.listdir(filePath):
        f = os.path.join(filePath, childDir)
        image = imread(f)
        image = resize(image, (200, 200, 3))
        fileNames.append(childDir)
        images.append(image)
        num += 1
        print("%d processing: %s" % (num, childDir))
    return images, num, fileNames


def visualize():
    fig, (ax1, ax2) = plt.subplots(
        1, 2, figsize=(8, 4), sharex=True, sharey=True)

    ax1.axis('off')
    ax1.imshow(data, cmap=plt.cm.gray)
    ax1.set_title('Input image')
    ax1.set_adjustable('box-forced')

    # Rescale histogram for better display
    hog_image_rescaled = exposure.rescale_intensity(
        hog_image, in_range=(0, 0.02))

    ax2.axis('off')
    ax2.imshow(hog_image_rescaled, cmap=plt.cm.gray)
    ax2.set_title('Histogram of Oriented Gradients')
    ax1.set_adjustable('box-forced')
    plt.show()


def getFeat(Data, mode, fileNames, positive):
    num = 0

    for image in Data:
        gray = rgb2gray(image)
        fd, hog_image = hog(gray, orientations, pixels_per_cell,
                            cells_per_block, block_norm, visualize, normalize, feature_vector)

        if(visualize):
            visualize(data, hog_image)

        fd_name = str(fileNames[num]) + '.feat'  # set file name

        if mode == 'train':
            if positive == True:
                fd_path = os.path.join('./features/train/positive/', fd_name)
            else:
                fd_path = os.path.join(
                    './features/train/negative/', fd_name)

        else:
            if positive == True:
                fd_path = os.path.join('./features/test/positive/', fd_name)
            else:
                fd_path = os.path.join('./features/test/negative/', fd_name)

        joblib.dump(fd, fd_path, compress=3)  # save data to local
        num += 1
        print("%d saving: ." % (num))

if __name__ == '__main__':
    t0 = time.time()

    Ptrain_filePath = r'./Positive'
    Ptest_filePath = r'./test/positive'

    PTrainData, P_train_num, fileNames = getData(
        Ptrain_filePath, np.array([[1]]))
    getFeat(PTrainData, 'train', fileNames, True)

    PTestData, P_test_num = getData(Ptest_filePath, np.array([[1]]))
    getFeat(PTestData, 'test')

    Ntrain_filePath = r'./Negative'
    Ntest_filePath = r'./test/negative'

    NTrainData, N_train_num, fileNames = getData(
        Ntrain_filePath, np.array([[0]]))
    getFeat(NTrainData, 'train', fileNames, False)

    NTestData, N_test_num = getData(Ntest_filePath, np.array([[0]]))
    getFeat(NTestData, 'test')

    t1 = time.time()
	
print('The cast of time is:%f' % (t1 - t0))
