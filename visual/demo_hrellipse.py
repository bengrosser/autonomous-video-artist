#-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.
 #File Name : demo_hrcorner.py
 #Creation Date : 02-06-2017
 #Last Modified : Tue Jun 13 14:41:42 2017
 #Created By : Rui An  
#_._._._._._._._._._._._._._._._._._._._._.

'''
This is a demo that shows and visualizes the procedure of how computer determines features(corners) 
in mordern computer vision libraries
Ellipse version
'''

#TODO: check out the performance and determine to use threading or not

import cv2
import numpy as np
import math
from matplotlib import pyplot as plt
import struct


#This time it is all about ellipse
def harris_visual(img): #TODO: Not quite sure about the size of the kernal
    def is_neg_zero(n):
        return struct.pack('>d', n) == '\x80\x00\x00\x00\x00\x00\x00\x00'
  
  
     #TODO: Need to check the output to determine
    def gaussian_blur(img):
        return cv2.GaussianBlur(img, (5,5), 0)


    def sobel_filter(img):
        sobelx = cv2.Sobel(img.copy(), cv2.CV_64F, 1, 0, ksize=5)
        sobely = cv2.Sobel(img.copy(), cv2.CV_64F, 0, 1, ksize=5)
        return (sobelx, sobely) 


    def get_direction(vector_1, vector_2):
        #The rotation in the drawing function is anti-clockwise 
        #So we do 360-the actual result
        def unit_vector(vector):
            return vector / np.linalg.norm(vector)

        def angle_between(v1, v2):
            v1_u = unit_vector(v1)
            v2_u = unit_vector(v2)
            return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

        return 360-np.degrees(angle_between(vector_1, vector_2))
        



    def harris_measure(Sx2, Sy2, Sxy):
        trace_matrix =  Sx2 + Sy2
        trace_square_matrix = trace_matrix * trace_matrix
        det_matrix = Sx2 * Sy2 - Sxy * Sxy
        harris_matrix = det_matrix - 0.01 * trace_square_matrix 
        #Harris_matrix now bears the "score" of how possible it can be the corner
        #TODO: Figure out whether I should dilate it or not
        harris_matrix = cv2.dilate(harris_matrix, None)
        return harris_matrix

    
    def get_eigen(i, j):
        sx2 = Sx2[i][j]
        sy2 = Sy2[i][j]
        sxy = Sxy[i][j]
        harris_measure_matrix = np.array([[sx2, sxy], [sxy, sy2]])
        eigen_vals, eigen_vector = np.linalg.eigh(harris_measure_matrix)
        # print eigen_vector
        # eigen_vals = np.linalg.eigvals(harris_measure_matrix)
        return eigen_vals,eigen_vector


    def get_eigen_matrix(harris_result):
        row, column = harris_result.shape
        eigen_value_matrix = np.zeros((row, column, 2))
        eigen_vector_matrix = np.zeros((row, column, 2, 2))
        # eigen_vector_matrix = 
        harris_mask = np.zeros_like(harris_result)
        harris_max = harris_result.max()
        # max_i, max_j = np.unravel_index(harris_result.argmax(), harris_result.shape)
        # max_eigen_x, max_eigen_y = get_eigenval(max_i, max_j) 
        # print max_eigen_x, max_eigen_y
        for i in range(row):
            for j in range(column):
                if harris_result[i][j] > 0.15*harris_max:
                    # harris_measure_matrix = np.array([[sx2, sxy], [sxy, sy2]])
                    harris_mask[i][j] = 1
                    eigen_vals, eigen_vector = get_eigen(i, j) 
                    eigen_value_matrix[i][j] = eigen_vals
                    eigen_vector_matrix[i][j] = eigen_vector
        # print "---------------------"
        # print eigen_value_matrix
        return (eigen_value_matrix, eigen_vector_matrix, harris_mask)

    
    def visualize(eigen_value_matrix, eigen_vector_matrix, harris_mask):
        #TODO:In here I chose the smallest eigen as the first value that should be in here
        #TODO:which means there might be chance that the ellipses went out of the way 
        #TODO:Adding another color map here
        #TODO:There is a compromise we have to make so that we can actually
        #visualize the whole thing properly
        x,y = np.shape(harris_mask)
        eigen_max = eigen_value_matrix.max()
        img = np.full((x, y, 3), 0, np.uint8)
        counter = 1
        for i in range(x):
            for j in range(y):
                if harris_mask[i][j]:
                    #In here we have to reverse the order since 
                    min_index = eigen_value_matrix[i][j].argmax(axis=0)  
                    max_index = eigen_value_matrix[i][j].argmin(axis=0)
                    min_axis_val = np.rint(1/(eigen_value_matrix[i][j][min_index]/eigen_max))
                    max_axis_val = np.rint(1/(eigen_value_matrix[i][j][max_index]/eigen_max))
                    # print min_index, max_index
                    min_axis_vector = eigen_vector_matrix[i][j][min_index]
                    rotation_in_degrees = np.rint(get_direction(min_axis_vector, np.array([0,1])))
                    color = np.random.randint(255, size=3)
                    color = tuple(color)
                    img = cv2.ellipse(img, (j, i), (int(max_axis_val), int(min_axis_val)), int(rotation_in_degrees), 0, 360, color, 1) 
                    counter += 1
        print counter
        return img
    sobelx, sobely = sobel_filter(img.copy())
    Ix2 = sobelx * sobelx
    Iy2 = sobely * sobely
    Ixy = sobelx * sobely
    Sx2 = gaussian_blur(Ix2)
    Sy2 = gaussian_blur(Iy2)
    Sxy = gaussian_blur(Ixy)
    # direction_rotation = get_direction(sobelx, sobely) 
    harris_result = harris_measure(Sx2, Sy2, Sxy)
    eigen_value_matrix, eigen_vector_matrix, harris_mask = get_eigen_matrix(harris_result)
    img = visualize(eigen_value_matrix, eigen_vector_matrix, harris_mask)
    # max_i, max_j = np.unravel_index(harris_result.argmax(), harris_result.shape)
    # max_eigen_x, max_eigen_y = get_eigenval(max_i, max_j) 
    # print max_eigen_x, max_eigen_y
    return img 


#Static image testing
# img = cv2.imread("./src_picture/exp.jpg")
# grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 
# result = harris_visual(grey)
# print "hello"
# cv2.imwrite("result.jpg", result)



camera = cv2.VideoCapture("./src_video/matrix-woman-red-142x60.mov")
fourcc = cv2.VideoWriter_fourcc(*'XVID')
frame_rate = 24 
resolution = (142, 60)
out = cv2.VideoWriter("let'ssee.avi" ,fourcc, frame_rate, resolution)

while True:
    grabbed, frame = camera.read()
    if grabbed:
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        harris_result = harris_visual(img)
        harris_result = np.uint8(harris_result)
        # harris_result = cv2.cvtColor(harris_result, cv2.COLOR_GRAY2RGB)
        out.write(harris_result)
        # print harris_result
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        print("No video feed available")
        break
camera.release()
out.release()
cv2.destroyAllWindows()








