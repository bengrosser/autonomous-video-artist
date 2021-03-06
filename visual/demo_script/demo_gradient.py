#-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.
 #File Name : demo_gradient.py
 #Creation Date : 24-05-2017 
 #Last Modified : Tue Jun 13 14:00:30 2017
 #Created By : Rui An  
#_._._._._._._._._._._._._._._._._._._._._.

# This is a demo that shows and visualizes the procedure of how computer determines the edges
# in modern computer vision libraries

import cv2
import cv2
import numpy as np
import math
from matplotlib import pyplot as plt
import struct
import argparse
import csv
import numpy as np
import math
from matplotlib import pyplot as plt
import struct
import argparse
import sys
import time

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
    # Below is the right way to convert
    # img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    # plt.imshow(img, cmap="gray")
    plt.imshow(img)
    plt.axis("off")
    plt.show()


#First step in canny edge algorithm
def gaussian_blur(img):
    return cv2.GaussianBlur(img, (5,5), 0)


#CV_64F has a way better looking results than the converted ones
#I am now gonna keep this perform the calculations to see
#whether it will exceeds the limits or not.
def sobel_filter(img):
    sobelx = cv2.Sobel(img.copy(), cv2.CV_64F, 1, 0, ksize=5)
    # abs_sobelx = np.absolute(sobelx)
    # sobel_8x = np.uint8(abs_sobelx)
    # showPic(sobelx)
    sobely = cv2.Sobel(img.copy(), cv2.CV_64F, 0, 1, ksize=5)
    # abs_sobely = np.absolute(sobely)
    # sobel_8y = np.uint8(abs_sobely)
    # showPic(sobely)
    return (sobelx, sobely)


def get_gradients(sobelx, sobely): 
    approx_array = np.array([-90, -45, -1, 1, 45, 90]) 
    gradient_intensity = np.sqrt(sobelx**2 + sobely**2)
    # showPic(gradient_intensity)
    for x in np.nditer(sobelx, op_flags=["readwrite"]):
        if x==0:
            x[...] = 0.01
    raw_direction_inradiant = np.arctan(sobely/sobelx)
    raw_direction_indegree = np.rad2deg(raw_direction_inradiant)
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


#check boundary conditions so we can stop in time
def hit_boundary(row, collumn, check_mask):
    row_limit, collumn_limit = np.shape(check_mask)
    row_limit -= 1
    collumn_limit -= 1
    if(row > row_limit or collumn > collumn_limit or 
            row < 0 or collumn < 0):
        return True 
    else: 
        return False 



#This function follows the direction of the gradient of one pixel
#and finds all the pixel that has the same direction along the way
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


#Based on the intensity of the pixel and the direction of the gradient
#we can visualize the intermediate results from the canny edge algorithm
def visualize(img, gradient_intensity, raw_direction_indegree, color_dir_map, background_color): 
    x, y = np.shape(img) 
    white = np.zeros((x,y,3), np.uint8)
    #colors = np.random.randint(0,255,(6,3))
    colors = color_dir_map
    # print colors
    for i in range(x):
        for j in range(y):
            white[i][j] = background_color 
    #to keep track of which pixels are already been processed along
    #the way
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
                white = cv2.line(white, (end_y, end_x), (j,i), color, 2)
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
                white = cv2.line(white, (end_y, end_x), (j,i), color, 2)
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



#Test with static images
#It is very important to convert the colors from BGR to Gray
# img = cv2.imread("exp.jpg")
# cv2.imshow("image", img)
# cv2.waitKey(0)
# img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# img_blur = gaussian_blur(img.copy())
# sobelx, sobely = sobel_filter(img_blur)
# gradient_intensity, raw_direction_indegree = get_gradients(sobelx, sobely)
# visualize(img_blur, gradient_intensity, raw_direction_indegree)


# def main():
parser = argparse.ArgumentParser()
parser.add_argument("src", help = "the video file you want to process")
parser.add_argument("output", help = "the video file you want the result to be written in")
parser.add_argument("framerate", help="the frame rate of the video")
parser.add_argument("res1", help="the first resolution of the video")
parser.add_argument("res2", help="the second resolution of the video")
parser.add_argument("-s", "--show", action = "store_true", help =
        "Only show the processed frame")
parser.add_argument("-c", "--color", default = None, help="provide color csv file")
args = parser.parse_args()

#TODO:Test those functions when get back
camera = cv2.VideoCapture(args.src)
fourcc = cv2.VideoWriter_fourcc(*'XVID')
frame_rate = float(args.framerate)
resolution = (int(args.res1), int(args.res2))
out = cv2.VideoWriter(args.output,fourcc, frame_rate, resolution)


color_dir_map = []
background_color = (255, 0, 0)
if(args.color != None):
    with open(args.color, 'rU') as list_color:
        counter = 0
        for row in list_color:
           raw = row.split(" ")
           print raw
           color_element = map(int, raw)
           if counter == 0:
               background_color = tuple(color_element) 
           else:
               color_dir_map.append(color_element)
           counter += 1
        color_dir_map = np.array(color_dir_map)
else:
    color_dir_map = np.array([[0,190,200],[200,70,70],[51,204,233],[120,80,220],[220,0,80],[200,200,60]])
print background_color
print color_dir_map
    
# Using the recommended format from the documents
# camera = cv2.VideoCapture("matrix-woman-red-142x60.mov")
# fourcc = cv2.VideoWriter_fourcc(*'XVID')
# out = cv2.VideoWriter('matrix_output_color.avi',fourcc, 23.975850, (142,60))
# count = 0

count = 1
start_time = time.time()
while True:
    grabbed, frame = camera.read()
    if grabbed:
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        img_blur = gaussian_blur(img.copy())

        sobelx, sobely = sobel_filter(img_blur)

        gradient_intensity, raw_direction_indegree = get_gradients(sobelx, sobely)

        result = visualize(img_blur, gradient_intensity, raw_direction_indegree, color_dir_map, background_color)
        
        frame_time = float(count/24.0)
        print "Finished the frame ", count, " spending ", (time.time()-start_time) , " seconds ", frame_time, " processed"  
        hsv = change_color(result, gradient_intensity)
        final_result = cv2.cvtColor(hsv , cv2.COLOR_HSV2RGB)
        if(args.show):
            cv2.imshow("result", final_result)
        else:
            out.write(final_result)
        # if(args.show):
            # cv2.imshow("result", result)
        # else:
            # out.write(result)

        count += 1
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        print("No video feed available")
        print "spending ", (time.time()-start_time), seconds 
        break
camera.release()
out.release()
cv2.destroyAllWindows()


# if __name__ == '__main__':
    # main()
