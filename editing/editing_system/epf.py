# Scripts for editing point finder
# EPF will try to find frames in videos that have the most
# logic compatibility

import cv2
import numpy as np
import gradient


def compare_gradient(img1, img2):
    img1 = cv2.cvtColor(img1, cv2.COLOR_BRG2GRAY)
    img1_blur = gradient.gaussian_blur(img1)


