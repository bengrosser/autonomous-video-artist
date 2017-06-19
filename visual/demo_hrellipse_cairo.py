#-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.
 #File Name : demo_hrcorner.py
 #Creation Date : 02-06-2017
 #Last Modified : Mon Jun 19 11:37:34 2017
 #Created By : Rui An  
#_._._._._._._._._._._._._._._._._._._._._.

'''
This is a demo that shows and visualizes the procedure of how computer determines features(corners) 
in mordern computer vision libraries
Ellipse version
also with the help of the cairo library
'''

#TODO: check out the performance and determine to use threading or not

import cv2
import numpy as np
import math
from matplotlib import pyplot as plt
import struct
import time 
import cairo
import scipy.optimize as op
from math import pi

#This time it is all about ellipse def harris_visual(img): #TODO: Not quite sure about the size of the kernal
def harris_visual(img):
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
        return eigen_vals,eigen_vector


    def get_eigen_matrix(harris_result):
        row, column = harris_result.shape
        eigen_value_matrix = np.zeros((row, column, 2))
        eigen_vector_matrix = np.zeros((row, column, 2, 2))
        harris_mask = np.zeros_like(harris_result)
        harris_max = harris_result.max()
        for i in range(row):
            for j in range(column):
                if harris_result[i][j] > 0.08*harris_max:
                    harris_mask[i][j] = 1
                    eigen_vals, eigen_vector = get_eigen(i, j) 
                    eigen_value_matrix[i][j] = eigen_vals
                    eigen_vector_matrix[i][j] = eigen_vector
        return (eigen_value_matrix, eigen_vector_matrix, harris_mask)

    
    def alpha_blending(fore_ground, back_ground, alpha):
        x,y,z = fore_ground.shape
        point_x, point_y, point_z = np.where(fore_ground != 255)
        num_points = len(point_x)
        for i in range(num_points):
            x_coor = point_x[i]
            y_coor = point_y[i]
            result_color = alpha*fore_ground[x_coor][y_coor] + (1-alpha)*back_ground[x_coor][y_coor]
            back_ground[x_coor][y_coor] = result_color
        return back_ground


    def drawEllipse(x, y, w, h, rotation,ctx):
        ctx.save()
        ctx.translate(x,y)
        ctx.rotate(np.deg2rad(rotation))
        ctx.scale(w,h)
        ctx.arc(0,0,1.0,0.0,2*pi)
        ctx.restore()
        ctx.stroke() 
    

    def visualize(eigen_value_matrix, eigen_vector_matrix, harris_result, harris_mask):
        #TODO:In here I chose the smallest eigen as the first value that should be in here
        #TODO:which means there might be chance that the ellipses went out of the way 
        #TODO:Adding another color map hereball_so_hard
        #TODO:There is a compromise we have to make so that we can actually #visualize the whole thing properly
        x,y = np.shape(harris_mask)
        surface = cairo.ImageSurface(cairo.FORMAT_ARGB32, y, x)
        ctx = cairo.Context(surface)
        ctx.set_antialias(cairo.ANTIALIAS_SUBPIXEL)
        ctx.set_source_rgb(1,1,1)
        ctx.paint()
        
        eigen_max = eigen_value_matrix.max()
        harris_max = harris_result.max()
        for i in range(x):
            for j in range(y):
                if harris_mask[i][j]:
                    #In here we have to reverse the order since 
                    min_index = eigen_value_matrix[i][j].argmax(axis=0)  
                    max_index = eigen_value_matrix[i][j].argmin(axis=0)
                    min_axis_val = np.rint(1/(eigen_value_matrix[i][j][min_index]/eigen_max))
                    max_axis_val = np.rint(1/(eigen_value_matrix[i][j][max_index]/eigen_max))
                    min_axis_vector = eigen_vector_matrix[i][j][min_index]
                    rotation_in_degrees = np.rint(get_direction(min_axis_vector, np.array([0,1])))
                    color = (0,0,0) 
                    alpha = harris_result[i][j]/float(harris_max) 

                    #This is the scale change version
                    # if alpha > 0.85:
                        # pass
                    # else:
                        # alpha = 0.5*alpha

                    #This is the curve fitting version
                    #Check the curve_fit.py for generation parameter
                    alpha = 0.04118395*np.exp(3.21999742*alpha)
                    ctx.set_source_rgba(0,0,0,alpha)
                    drawEllipse(j, i, int(max_axis_val), int(min_axis_val), int(rotation_in_degrees), ctx)
        return surface 

    sobelx, sobely = sobel_filter(img.copy())
    Ix2 = sobelx * sobelx
    Iy2 = sobely * sobely
    Ixy = sobelx * sobely
    Sx2 = gaussian_blur(Ix2)
    Sy2 = gaussian_blur(Iy2)
    Sxy = gaussian_blur(Ixy)
    harris_result = harris_measure(Sx2, Sy2, Sxy)
    eigen_value_matrix, eigen_vector_matrix, harris_mask = get_eigen_matrix(harris_result)
    img = visualize(eigen_value_matrix, eigen_vector_matrix, harris_result, harris_mask)
    return img 


# Static image testing
# img = cv2.imread("./src_picture/exp.jpg")
# grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 
# x,y = grey.shape
# start_time = time.time()
# result = harris_visual(grey)
# buf = result.get_data()
# a = np.frombuffer(buf, np.uint8)
# a.shape = (x,y,4)
# b = a[:,:,0:3]
# cv2.imwrite("debug_cairo_with_curve.jpg", b)
# end_time = time.time()
# print end_time - start_time 


camera = cv2.VideoCapture("./src_video/matrix-woman-red.mp4")
fourcc = cv2.VideoWriter_fourcc(*'XVID')
frame_rate = 24 
resolution = (1920, 800)
out = cv2.VideoWriter("highrez_with_curve_fit.avi" ,fourcc, frame_rate, resolution)
start_time = time.time()

while True:
    grabbed, frame = camera.read()
    if grabbed:
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        x,y = img.shape
        harris_result = harris_visual(img)
        buf = harris_result.get_data()
        temp_result_matrix = np.frombuffer(buf, np.uint8)
        temp_result_matrix.shape = (x,y,4)
        result_matrix = temp_result_matrix[:,:,0:3]
        out.write(result_matrix)
        print "finished on frame"
    else:
        print("No video feed available")
        break
end_time = time.time()
print end_time - start_time 
camera.release()
out.release()
cv2.destroyAllWindows()

