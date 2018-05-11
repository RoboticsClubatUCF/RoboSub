import numpy as np
import cv2

def distance_to_camera(KNOWN_HEIGHT, focalLength, perWidth):
    return (KNOWN_HEIGHT * focalLength) / perWidth


def findFocalLength(marker, focalLength):
    return (marker[1][0] * 36) / 57


def findDistance(marker, focalLength):
    KNOWN_DISTANCE = 36
    KNOWN_HEIGHT = 57

    if focalLength == None:
        focalLength = findFocalLength(marker, focalLength)

    inches = distance_to_camera(KNOWN_HEIGHT, focalLength, marker[1][0])

    print(inches)
    return inches, focalLength
