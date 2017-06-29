#-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.
 #File Name :
 #Creation Date :
 #Created By : Rui An  
#_._._._._._._._._._._._._._._._._._._._._.

import numpy as np
import cv2
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter

#First we generate an image with a big white cube in the middle
img = np.zeros((256,256,3), np.uint8)
cv2.rectangle(img,(78,78),(178,178),(255,255,255),-1)
cv2.imwrite("white_cube.jpg", img)


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
    img = cv2.imread('white_cube.jpg', 0)
    print img
    #This dft has all the values we need for the recovery of the photo, I hope
    dft = cv2.dft(np.float32(img),flags = cv2.DFT_COMPLEX_OUTPUT)
    # dft = np.fft.fftshift(dft)
    # I actually don't want to have it shift
    # dft_shift = np.fft.fftshift(dft)
    result_magnititude = 20*np.log(cv2.magnitude(dft[:,:,0],dft[:,:,1]))
    result_magnititude = np.uint8(result_magnititude)
    result_color_magnititude = cv2.cvtColor(result_magnititude, cv2.COLOR_GRAY2RGB)
    cv2.imwrite("wc_frequency_magnitude.jpg", result_color_magnititude)
    return dft, result_magnititude

def synthesis(x_freq, y_freq, complex_a, complex_b, row_size, col_size):
    X_data = np.arange(row_size)
    Y_data = np.arange(col_size)
    X_data, Y_data = np.meshgrid(X_data, Y_data)
    Z_data = complex_a*(np.cos(x_freq*X_data))*(np.cos(y_freq*Y_data)) + \
            complex_b*(np.sin(x_freq*X_data))*(np.sin(y_freq*Y_data))
    return Z_data
    

def visualize(complex_dft_result, result_magnititude):
    '''
    In here I scaled down the magnitude
    Using the previous results
    Shit, I think I will follow the path from the signal
    Processing book, separate down the wave into two forms
    one cos and one sin, and added them back together
    '''
    x,y,_= complex_dft_result.shape
    #Also I need to build up the range for drawing functions
    X_value = np.arange(x) 
    Y_value = np.arange(y)
    Z_value = np.zeros((x,y))
    # plt.ion()
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    for i in range(x):
        x_freq = 2.0*np.pi*(i/float(x))
        progress = 100*(float(i*x)/float(x*y))
        print "Finished", progress,"%"
        for j in range(y):
            # if result_magnititude[i][j] < 50:
                # continue
            y_freq = 2.0*np.pi*(j/float(y))
            complex_a = complex_dft_result[i][j][0]
            complex_b = complex_dft_result[i][j][1]
            #In here I have to scale it down a little bit
            # if complex_a != 0: 
                # if complex_a < 0:
                    # complex_a = -20*np.log(np.abs(complex_a))
                # else:
                    # complex_a = 20*np.log(np.abs(complex_a))
            # if complex_b != 0:
                # if complex_b < 0:
                    # complex_b = -20*np.log(np.abs(complex_b))
                # else:
                    # complex_b = 20*np.log(np.abs(complex_b))

            temp_result = synthesis(x_freq, y_freq, complex_a, complex_b, x, y)
            Z_value = Z_value + temp_result
    print Z_value
    X_value, Y_value = np.meshgrid(X_value, Y_value)
    # surf = ax.plot_surface(X_value, Y_value, np.abs(Z_value), cmap=cm.coolwarm,linewidth=0, antialiased=False)
    surf = ax.plot_surface(X_value, Y_value, Z_value, cmap=cm.coolwarm,linewidth=0, antialiased=False)
    # plt.pause(1)
    ax.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))
    fig.colorbar(surf, shrink=0.5, aspect=5)
    plt.show()

    # print Z_value
    # Z_int = np.rint(Z_value)
    # result = np.uint8(Z_int)
    # result = cv2.cvtColor(result, cv2.COLOR_GRAY2RGB)
    # cv2.imwrite("god_bless.jpg", result)


complex_dft_result,result_magnititude = fft()
# print result_magnititude
visualize(complex_dft_result, result_magnititude)



