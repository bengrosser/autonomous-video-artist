#-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.
 #File Name :
 #Creation Date :
 #Created By : Rui An  
#_._._._._._._._._._._._._._._._._._._._._.

import numpy as np
from numpy import pi as pi
import cv2
from mpl_toolkits.mplot3d import Axes3D 
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import matplotlib.animation as animation
from matplotlib.collections import PolyCollection
import time
import sys

# img = np.zeros((300,300,3), np.uint8)
# cv2.rectangle(img,(55,55),(105,105),(255,255,255),-1)
# cv2.ellipse(img, (30,30),(20,20), 0, 0, 360, (255,255,255), -1) # cv2.ellipse(img, (30,120),(20,20), 0, 0, 360, (255,255,255), -1) # img_hori = cv2.flip(img, 0)
# img_left = cv2.bitwise_or(img_hori, img)
# img_right = cv2.flip(img_left, 1)
# result_image = cv2.bitwise_or(img_left, img_right)



def create_grid_flip(img):
    row_size, col_size = img.shape
    grid = np.zeros((2*row_size, 2*col_size))
    grid[0:row_size, 0:col_size] = img
    img_hori = cv2.flip(grid, 0)
    img_left = cv2.bitwise_or(img_hori, grid)
    img_right = cv2.flip(img_left, 1)
    result_image = cv2.bitwise_or(img_left, img_right)
    cv2.imwrite("canvas_testing.jpg", result_image)



def create_grid_copy(img):
    row_size, col_size = img.shape
    grid = np.zeros((2*row_size, 2*col_size))
    grid[0:row_size, 0:col_size] = img
    grid[0:row_size, col_size:(2*col_size)] = img
    grid[row_size:2*row_size, 0:col_size] = img
    grid[row_size:(2*row_size), col_size:(2*col_size)] = img
    cv2.imwrite("canvas_testing.jpg", grid)
    

'''
I am going to use cv2 version here to since it has both of the values
'''
def fft():
    img = cv2.imread('./canvas_testing.jpg', 0)
    # img = cv2.imread('./canvas_testing.jpg', 0)
    x,y = img.shape
    #This dft has all the values we need for the recovery of the photo, I hope
    dft = cv2.dft(np.float32(img),flags = cv2.DFT_COMPLEX_OUTPUT)
    # dft = np.fft.fftshift(dft) # I actually don't want to have it shift
    # dft_shift = np.fft.fftshift(dft)
    result_magnititude = 20*np.log(cv2.magnitude(dft[:,:,0],dft[:,:,1]))
    result_magnititude = np.uint8(result_magnititude)
    result_color_magnititude = cv2.cvtColor(result_magnititude, cv2.COLOR_GRAY2RGB)
    cv2.imwrite("wc_frequency_magnitude.jpg", result_color_magnititude)
    return dft, result_magnititude


def synthesis(x_freq, y_freq, complex_a, complex_b, row_size, col_size, complex_dft_result, phase):
    X_data = np.arange(row_size)
    Y_data = np.arange(col_size)
    X_data, Y_data = np.meshgrid(X_data, Y_data)
    '''
    This line can make a symmetry of the input images
    '''
    Z_data = complex_a*(np.cos(x_freq*X_data))*(np.cos(y_freq*Y_data)) + \
            complex_b*(np.sin(x_freq*X_data))*(np.sin(y_freq*Y_data))
    return Z_data
    

def visualize(frame_num, frame_count, camera):
    grabbed, frame = camera.read()
    if grabbed:
        # cv2.imshow("the result", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            return
    else:
        print "No video feed available"
        return
    
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    create_grid_flip(img)
    complex_dft_result,result_magnititude = fft()
    x,y,_ = complex_dft_result.shape
    Z_value = np.zeros((y,x))

    global surf
    
    x,y,_= complex_dft_result.shape
    #Also I need to build up the range for drawing functions
    X_value = np.arange(x) 
    Y_value = np.arange(y) 

    if surf != None:
        surf.remove()
    print frame_num
    for i in range(frame_num):
        i_index = int(i)/20
        j_index = int(i)%20 
        x_freq = 2*np.pi*(i_index/float(x))
        progress = 100*(float(i_index*x)/float(x*y))
        # if result_magnititude[i_index][j_index] < 20:
            # continue
        y_freq = 2*np.pi*(j_index/float(y))

        if ((i_index==0) and (j_index==0)) or ((i_index==((x/2)-1)) and ((j_index==((y/2)-1)))):
            complex_a = complex_dft_result[i_index][j_index][0]/(x)
            complex_b = -complex_dft_result[i_index][j_index][1]/(y/2)
        else:
            complex_a = complex_dft_result[i_index][j_index][0]/(x/2)
            complex_b = -complex_dft_result[i_index][j_index][1]/(y/2)
                
                # complex_a = complex_dft_result[i][j][0]
                # complex_b = complex_dft_result[i][j][1]
        phase = np.arctan2(complex_b,complex_a)
        temp_result = synthesis(x_freq, y_freq, complex_a, complex_b, x, y, complex_dft_result, phase)
        Z_value = Z_value + temp_result
   # print Z_value
    to_show = Z_value/float(x*y)
    X_value, Y_value = np.meshgrid(X_value, Y_value)
    X_row, X_col = X_value.shape
    Y_row, Y_col = Y_value.shape
    Z_row, Z_col = Z_value.shape
    surf = ax.plot_surface(X_value[0:(X_row/2),0:(X_col/2)], Y_value[0:(Y_row/2),0:(Y_col/2)], to_show[0:(Z_row/2),0:(Z_col/2)], cmap=cm.coolwarm,linewidth=0, antialiased=False)
    return fig 


video_name = sys.argv[3]
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.set_frame_on(False)
ax.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))
surf = None

azim_value = int(sys.argv[1])
elev_value = int(sys.argv[2])
ax.view_init(azim=azim_value, elev=elev_value)

camera = cv2.VideoCapture(video_name)
frame_count = int(camera.get(cv2.cv.CV_CAP_PROP_FRAME_COUNT))
frame_rate = 24 
resolution = (142, 60)


FFwriter = animation.FFMpegFileWriter(bitrate = 5000000)
anim = animation.FuncAnimation(fig, visualize, fargs=(frame_count, camera),
                                   interval=150, blit=False, repeat_delay=0,frames=frame_count, repeat=False)
result_name = video_name[0:-4] + "_" +str(azim_value)+"_"+str(elev_value)+".mp4"
# plt.show()
anim.save(result_name)






