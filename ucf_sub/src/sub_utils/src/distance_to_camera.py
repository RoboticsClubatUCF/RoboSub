import numpy as np
import cv2


def distance_to_camera(knownWidth, focalLength, perWidth):
    return (knownWidth * focalLength) / perWidth


def findFocalLength(marker, focalLength)
    return (marker[1][0] * KNOWN_DISTANCE) / KNOWN_WIDTH


def findDistance(marker, focalLength):
    KNOWN_DISTANCE = 0
    KNOWN_WIDTH = 0

    if focalLength == None:
        focalLength = findFocalLength(marker, focalLength)

    inches = distance_to_camera(KNOWN_WIDTH, focalLength, marker[1][0])

    return inches, focalLength
