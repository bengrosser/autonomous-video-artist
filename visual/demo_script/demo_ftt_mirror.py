#-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.
 #File Name : demo_ftt_mirror.py
 #Creation Date : 24-07-2017
 #Created By : Rui An  
#_._._._._._._._._._._._._._._._._._._._._.

'''
FFT mirror
'''



import numpy as np
import cv2
import time
from matplotlib import pyplot as plt



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

    result = cv2.resize(result, (500,500), interpolation=cv2.INTER_LINEAR)
    return result 


camera = cv2.VideoCapture(0)
# fourcc = cv2.VideoWriter_fourcc(*'XVID')
# frame_rate = 24 
# resolution = (1280, 720)
# out = cv2.VideoWriter("basic_fft.avi" ,fourcc, frame_rate, resolution)
start_time = time.time()


while True:
    grabbed, frame = camera.read()
    if grabbed:
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        x,y = img.shape
        ftt_result = fft(img)
        print "wtf"
        cv2.imshow("mirror", ftt_result)
    else:
        print("No video feed available")
        break
end_time = time.time()
print end_time - start_time 
camera.release()
out.release()
cv2.destroyAllWindows()
