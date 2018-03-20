import cv2
import numpy as np
import math
from matplotlib import pyplot as plt
import struct
import argparse
import sys
import time
from numba import jit

sys.setrecursionlimit(100000)
dir_map = {"90":(1,0),"-90":(-1,0),"1":(0,1),"-1":(0,-1), "-45":(-1,-1), "45":(1,1)} 
color_map = {"90":0,"-90":1,"1":2,"-1":3,
        "-45":4, "45":5} 


def is_neg_zero(n):
    return struct.pack('>d', n) == '\x80\x00\x00\x00\x00\x00\x00\x00'


def is_negative(n):
    return ord(struct.pack('>d', n)[0]) & 0x80 != 0


#A modification of color channel so the display order is right
def showPic(img):
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    plt.imshow(img)
    plt.axis("off")
    plt.show()

#First step in canny edge algorithm
def gaussian_blur(img):
    return cv2.GaussianBlur(img, (5,5), 0)


def sobel_filter(img):
    sobelx = cv2.Sobel(img.copy(), cv2.CV_64F, 1, 0, ksize=5)
    sobely = cv2.Sobel(img.copy(), cv2.CV_64F, 0, 1, ksize=5)
    return (sobelx, sobely)


@jit(nopython=True)
def mapping_direction_for_jit(raw_direction_indegree):
    result = np.zeros(raw_direction_indegree.shape)
    approx_array = np.array([-90, -45, -1, 1, 45, 90])
    rows, cols = raw_direction_indegree.shape
    for row in range(rows):
        for col in range(cols):
            element = raw_direction_indegree[row, col]
            if math.isnan(element):
                element = 90
            if element == 0:
                element = 1
            tempResult = np.abs(approx_array - element)
            element = approx_array[np.argmin(tempResult)]
            result[row, col] = element
    return result


def get_gradients_with_jit(sobelx, sobely):
    # start_time = time.time()
    gradient_intensity = np.sqrt(sobelx**2 + sobely**2)
    # print "Get intensity in", time.time()-start_time, "seconds"
    # start_time = time.time()
    for x in np.nditer(sobelx, op_flags=["readwrite"]):
        if x==0:
            x[...] = 0.01
    raw_direction_inradiant = np.arctan(sobely/sobelx)
    raw_direction_indegree = np.rad2deg(raw_direction_inradiant)
    # make sure that there is no negatvie zero inside the matrix
    for x in np.nditer(raw_direction_indegree, op_flags=["readwrite"]):
        if is_neg_zero(x):
            x[...] = -1
    result_direction = mapping_direction_for_jit(raw_direction_indegree)
    return (gradient_intensity, result_direction)


def get_gradients(sobelx, sobely):
    approx_array = np.array([-90, -45, -1, 1, 45, 90])
    # start_time = time.time()
    gradient_intensity = np.sqrt(sobelx**2 + sobely**2)
    # print "Get intensity in", time.time()-start_time, "seconds"

    # start_time = time.time()
    for x in np.nditer(sobelx, op_flags=["readwrite"]):
        if x==0:
            x[...] = 0.01
    raw_direction_inradiant = np.arctan(sobely/sobelx)
    raw_direction_indegree = np.rad2deg(raw_direction_inradiant)
    # make sure that there is no negatvie zero inside the matrix
    for x in np.nditer(raw_direction_indegree, op_flags=['readwrite']):
        if math.isnan(x):
            x[...] = 90
        if is_neg_zero(x):
            x[...] = -1
        if x == 0:
            x[...] = 1
        tempResult = np.abs(approx_array - x)
        x[...] = approx_array[np.argmin(tempResult)]
    return (gradient_intensity, raw_direction_indegree)


def hit_boundary(row, collumn, check_mask):
    row_limit, collumn_limit = np.shape(check_mask)
    row_limit -= 1
    collumn_limit -= 1
    if(row > row_limit or collumn > collumn_limit or 
            row < 0 or collumn < 0):
        return True 
    else: 
        return False 


def follow_dir(row, collumn, raw_direction_indegree, prev_dir, check_mask):
    temp_dir = -1*prev_dir
    temp_dir = int(temp_dir)
    go_row, go_collumn = dir_map[str(temp_dir)]
    if hit_boundary(row, collumn, check_mask):     
        return (row+go_row, collumn+go_collumn)
    if check_mask[row][collumn]:
        return (row+go_row, collumn+go_collumn)
    if raw_direction_indegree[row][collumn] != prev_dir:
        return (row+go_row, collumn+go_collumn)
    else:
        go_dir = prev_dir
        go_dir = int(go_dir)
        go_row, go_collumn = dir_map[str(go_dir)]
        check_mask[row][collumn] = 1
        return follow_dir(row+go_row, collumn+go_collumn, raw_direction_indegree, prev_dir, check_mask)


def visualize(img, gradient_intensity, raw_direction_indegree, color_dir_map, background_color):
    x, y = np.shape(img) 
    white = np.zeros((x,y,3), np.uint8)
    colors = color_dir_map
    for i in range(x):
        for j in range(y):
            white[i][j] = (255, 0, 0)
    check_mask = np.zeros((x,y))
    for i in range(x):
        for j in range(y):
            if check_mask[i][j]:
                continue
            else:
                start_dir = raw_direction_indegree[i][j]
                if start_dir < 0:
                    continue
                end_x, end_y = follow_dir(i, j, raw_direction_indegree, start_dir, 
                        check_mask)
                color = colors[color_map[str(int(start_dir))]].tolist()
                cv2.line(white, (end_y, end_x), (j,i), color, 2)
    for i in range(x-1,-1,-1):
        for j in range(y-1,-1,-1):
            if check_mask[i][j]:
                continue
            else:
                start_dir = raw_direction_indegree[i][j]
                if start_dir > 0:
                    continue
                end_x, end_y = follow_dir(i, j, raw_direction_indegree, start_dir, 
                        check_mask)
                color = colors[color_map[str(int(start_dir))]].tolist()
                cv2.line(white, (end_y, end_x), (j,i), color, 2)
    # cv2.imwrite("result.jpg", white)
    return white

#Change the color space from BGR to HSV taking consideration of intensity 
def change_color(result, gradient_intensity):
    hsv = cv2.cvtColor(result, cv2.COLOR_BGR2HSV)
    row, collumn = np.shape(gradient_intensity)
    max_gradient = gradient_intensity.max()
    for i in range(row):
        for j in range(collumn):
            hsv[i][j][2] = gradient_intensity[i][j]
    return hsv
            
#Testing functions
def test_gradientDir():
    the_dir = np.array([[45,1,1],[90,45,1],[90,1,45]])
    white = np.zeros((3,3,3), np.uint8)
    check_mask = np.zeros((3,3))
    print follow_dir(0,0,the_dir,45,check_mask)


def produce_gradient_video(src, output, framerate, res1, res2):
    # src_path = "./data/video/" + src
    camera = cv2.VideoCapture(src)
    # fourcc = cv2.VideoWriter_fourcc(*'XVID')
    fourcc = cv2.cv.CV_FOURCC(*'XVID')
    frame_rate = float(framerate)
    resolution = (int(res1), int(res2))
    out = cv2.VideoWriter(output, fourcc, frame_rate, resolution)
    background_color = (0, 0, 0)
    color_dir_map = np.array([[0,190,200],[200,70,70],[51,204,233],[120,80,220],[220,0,80],[200,200,60]])
    count = 1
    start_time = time.time()
    while True:
        grabbed, frame = camera.read()
        if grabbed and count < 2160:
            gradient_time = time.time()
            img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            img_blur = gaussian_blur(img.copy())
            sobelx, sobely = sobel_filter(img_blur)
            gradient_intensity, raw_direction_indegree = get_gradients_with_jit(sobelx, sobely)
            result = visualize(img_blur, gradient_intensity, raw_direction_indegree, color_dir_map, background_color)
            frame_time = float(count/24.0)
            hsv = change_color(result, gradient_intensity)
            final_result = cv2.cvtColor(hsv , cv2.COLOR_HSV2RGB)
            out.write(final_result)
            print "Finished this frame spends", time.time()-gradient_time, "seconds"
            print "finished", count, "frames"
            count += 1
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break
        else:
            print("No video feed available")
            print "spending ", (time.time()-start_time), "seconds"
            break
    camera.release()
    out.release()
    cv2.destroyAllWindows()
