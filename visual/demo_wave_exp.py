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
import time

# img = np.zeros((300,300,3), np.uint8)
# cv2.rectangle(img,(55,55),(105,105),(255,255,255),-1)
# cv2.ellipse(img, (30,30),(20,20), 0, 0, 360, (255,255,255), -1)
# cv2.ellipse(img, (30,120),(20,20), 0, 0, 360, (255,255,255), -1)

# img_hori = cv2.flip(img, 0)
# img_left = cv2.bitwise_or(img_hori, img)
# img_right = cv2.flip(img_left, 1)
# result_image = cv2.bitwise_or(img_left, img_right)


img = cv2.imread("./darth.jpg", 0)

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
    

# create_grid_copy(img)
create_grid_flip(img)



# cv2.rectangle(img,(245,55),(195,105),(255,255,255),-1)
# cv2.ellipse(img, (270,30),(20,20), 0, 0, 360, (255,255,255), -1)
# cv2.ellipse(img, (270,90),(20,20), 0, 0, 360, (255,255,255), -1)



# cv2.ellipse(img, (100,100),(20,20), 0, 0, 360, (255,255,255), -1)
# cv2.rectangle(img, (180,180),(220,220),(255,255,255), -1)
# cv2.imwrite("canvas_testing.jpg", result_image)


#And Plot it in 3d space
# img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# X = np.arange(512)
# Y = np.arange(512)
# origin_figure = plt.figure()
# X, Y = np.meshgrid(X, Y)
# ax = origin_figure.gca(projection='3d')
# surf = ax.plot_surface(X, Y, img, cmap=cm.coolwarm,
                       # linewidth=0, antialiased=False)
# plt.show()



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
    #Now I have added phase into considerations
    # Z_data = complex_a*(np.cos(x_freq*X_data + phase))*(np.cos(y_freq*Y_data + phase)) + \
            # complex_b*(np.sin(x_freq*X_data+phase))*(np.sin(y_freq*Y_data+phase))
    # Z_data = complex_a*(np.cos(x_freq*X_data))*(np.cos(y_freq*Y_data)) + \
            # complex_b*(np.sin(x_freq*X_data))*(np.sin(y_freq*Y_data))

    # Z_data = complex_a*(np.cos(x_freq*X_data))*(np.cos(y_freq*Y_data)) + \
            # complex_b*(np.sin(x_freq*X_data) + (np.pi/2))*(np.sin(y_freq*Y_data))

    
    '''
    This line can make a symmetry of the input images
    '''
    # Z_data = complex_a*(np.cos(x_freq*X_data + phase))*(np.cos(y_freq*Y_data + phase)) + \
            # complex_b*(np.sin(x_freq*X_data + (np.pi/2) + phase))*(np.sin(y_freq*Y_data + (np.pi/2) + phase))
    
    # Z_data = complex_a*(np.cos(x_freq*X_data + phase))*(np.cos(y_freq*Y_data + phase)) + \
            # complex_b*(np.sin(x_freq*X_data + phase))*(np.sin(y_freq*Y_data + (np.pi/2) + phase))

    # Z_data = complex_a*(np.cos(x_freq*X_data))*(np.cos(y_freq*Y_data)) + \
            # complex_b*(np.sin(x_freq*X_data))*(np.sin(y_freq*Y_data + (np.pi/2)))

    # phase_shift_x = float(x_freq)*np.pi/2 
    # phase_shift_y = float(y_freq)*np.pi/2 
    # Z_data = complex_a*(np.cos(x_freq*X_data))*(np.cos(y_freq*Y_data)) + \
            # complex_b*(np.sin(x_freq*X_data))*(np.sin(y_freq*Y_data+(np.pi/2)))

    # phase_shift_x = float(x_freq)*phase 
    # phase_shift_y = float(y_freq)*phase 
    
    # phase_shift_x_sin = float(x_freq)*(phase+(np.pi/2))
    # phase_shift_y_sin = float(y_freq)*(phase+(np.pi/2)) 

    Z_data = complex_a*(np.cos(x_freq*X_data))*(np.cos(y_freq*Y_data)) + \
            complex_b*(np.sin(x_freq*X_data))*(np.sin(y_freq*Y_data))
    # Z_data = complex_a*(np.cos(x_freq*X_data + y_freq*Y_data)) + complex_b*(np.sin(x_freq*X_data + y_freq*Y_data + pi/2))




    # Z_data = complex_a*(np.cos(x_freq*X_data))*(np.cos(y_freq*Y_data)) + \
            # complex_b*(np.sin(x_freq*X_data+(np.pi/2)))*(np.sin(y_freq*Y_data+(np.pi/2)))




    # Z_data = complex_b*(np.sin(x_freq*X_data))*(np.sin(y_freq*Y_data))
    # complex_part = (1j)*(x_freq*X_data + y_freq*Y_data)
    return Z_data
    

def visualize(complex_dft_result, result_magnititude):
    '''
    I won't scale down the result, since I found it actually
    has to work that way

    Shit, I think I will follow the path from the signal
    Processing book, separate down the wave into two forms
    one cos and one sin, and added them back together
    '''
    x,y,_= complex_dft_result.shape
    #Also I need to build up the range for drawing functions
    X_value = np.arange(x) 
    Y_value = np.arange(y) 
    Z_value = np.zeros((y,x))
    # plt.ion()
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    '''
    Probably one way of increasing the performance is to 
    only care about four corners of the intensity map
    '''
    for i in range(x/2):
        x_freq = 2*np.pi*(i/float(x))
        progress = 100*(float(i*x)/float(x*y))
        print "Finished", progress,"%"
        for j in range(y/2):
            # if result_magnititude[i][j] < 20:
                # continue
            y_freq = 2*np.pi*(j/float(y))

            if ((i==0) and (j==0)) or ((i==((x/2)-1)) and ((j==((y/2)-1)))):
                print i,j
                complex_a = complex_dft_result[i][j][0]/(x)
                complex_b = -complex_dft_result[i][j][1]/(y/2)
            else:
                complex_a = complex_dft_result[i][j][0]/(x/2)
                complex_b = -complex_dft_result[i][j][1]/(y/2)
            
            # complex_a = complex_dft_result[i][j][0]
            # complex_b = complex_dft_result[i][j][1]

            phase = np.arctan2(complex_b,complex_a)
            temp_result = synthesis(x_freq, y_freq, complex_a, complex_b, x, y, complex_dft_result, phase)
            Z_value = Z_value + temp_result
    print Z_value
    Z_value = Z_value/float(x*y)
    X_value, Y_value = np.meshgrid(X_value, Y_value)
    # surf = ax.plot_surface(X_value, Y_value, np.abs(Z_value), cmap=cm.coolwarm,linewidth=0, antialiased=False)
    X_row, X_col = X_value.shape
    Y_row, Y_col = Y_value.shape
    Z_row, Z_col = Z_value.shape
    surf = ax.plot_surface(X_value[0:(X_row/2),0:(X_col/2)], Y_value[0:(Y_row/2),0:(Y_col/2)], Z_value[0:(Z_row/2),0:(Z_col/2)], cmap=cm.coolwarm,linewidth=0, antialiased=False)
    # surf = ax.plot_surface(X_value, Y_value, Z_value, cmap=cm.coolwarm,linewidth=0, antialiased=False)
    # plt.pause(1)
    # plt.pause(1)
    ax.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))
    fig.colorbar(surf, shrink=1.0, aspect=5)
    end_time = time.time()
    print "end time is", end_time
    plt.show()

    # print Z_value
    # Z_int = np.rint(Z_value)
    # result = np.uint8(Z_int)
    # result = cv2.cvtColor(result, cv2.COLOR_GRAY2RGB)
    # cv2.imwrite("god_bless.jpg", result)


complex_dft_result,result_magnititude = fft()
# print result_magnititude
start_time = time.time()
print "start time is", start_time
visualize(complex_dft_result, result_magnititude)




