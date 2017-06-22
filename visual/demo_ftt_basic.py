#-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.
 #File Name : demo_ftt_basic.py
 #Creation Date : 22-06-2017
 #Created By : Rui An  
#_._._._._._._._._._._._._._._._._._._._._.
'''
This is just a simple program to demonstrate the fft   
'''

import numpy as np
import cv2
import time
from matplotlib import pyplot as plt

'''
I am just going to use the numpy for this one
But I do think I might need to use cv2 version to 
get more info out of the fft
'''
def fft(img):
    f = np.fft.fft2(img)
    #We need to shift the zero frequency from top left to the center
    fshift = np.fft.fftshift(f)
    magnitude_spectrum = 20*np.log(np.abs(fshift))
    magnitude_spectrum = np.rint(magnitude_spectrum)
    result = np.uint8(magnitude_spectrum)
    # print magnitude_spectrum
    result = cv2.cvtColor(result, cv2.COLOR_GRAY2RGB)
    # plt.subplot(122),plt.imshow(magnitude_spectrum, cmap = 'gray')
    # plt.title('Magnitude Spectrum'), plt.xticks([]), plt.yticks([])
    # plt.show()
    # cv2.imshow("result", result)
    # cv2.waitKey(0)
    return result 

    

# Static image testing
# img = cv2.imread("./src_picture/exp.jpg")
# grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# x,y,_ = img.shape
# start_time = time.time()
# result = fft(grey)
# cv2.imshow("result", result)
# end_time = time.time()
# print end_time - start_time 


camera = cv2.VideoCapture("./src_video/test_clip.mp4")
fourcc = cv2.VideoWriter_fourcc(*'XVID')
frame_rate = 24 
resolution = (1280, 720)
out = cv2.VideoWriter("basic_fft.avi" ,fourcc, frame_rate, resolution)
start_time = time.time()


while True:
    grabbed, frame = camera.read()
    if grabbed:
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        x,y = img.shape
        ftt_result = fft(img)
        out.write(ftt_result)
        print "finished on frame"
    else:
        print("No video feed available")
        break
end_time = time.time()
print end_time - start_time 
camera.release()
out.release()
cv2.destroyAllWindows()

