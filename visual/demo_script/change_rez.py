import cv2
import numpy as np

img = cv2.imread("./src_picture/darth_vadar.jpg", 0)
resized_img = cv2.resize(img, (150,150))
cv2.imwrite("darth.jpg", resized_img)
