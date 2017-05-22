import cv2
import numpy as np
from matplotlib import pyplot as plt

#Now All I can do is just settle down with plt and not mess with the window functions
#Totally pain in the ass

global p0 
global old_img 
global prvs  
global hsv 

def showPic(img):
    plt.imshow(img)
    plt.axis("off")
    plt.show()


def detectLines(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 150, apertureSize = 3)
    lines = cv2.HoughLines(edges, 1, np.pi/180, 200)
    for rho, theta in lines[0]:
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a*rho
        y0 = b*rho
        x1 = int(x0 + 1000*(-b))
        y1 = int(y0 + 1000*(a))
        x2 = int(x0 - 1000*(-b))
        y2 = int(y0 - 1000*(a))
    cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)
    return img
    

def detectCircles(img):
    img = cv2.medianBlur(img,5)
    cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
    circles = cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,1,20,
                                param1=50,param2=30,minRadius=0,maxRadius=0)
    circles = np.uint16(np.around(circles))
    for i in circles[0,:]:
        # draw the outer circle
        cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
        # draw the center of the circle
        cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)
    return img


#Checkout the optic flow functions
#This is the light version of optic flow
#TODO: Further testing
def light_motionEffects(img, which_frame):
    feature_params = dict( maxCorners = 100,
                       qualityLevel = 0.3,
                       minDistance = 7,
                       blockSize = 7 )
    lk_params = dict( winSize  = (15,15),
                      maxLevel = 2,
                      criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
    color = np.random.randint(0,255,(100,3))
    global p0
    global old_img
    
    if(which_frame == 0):
        old_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        p0 = cv2.goodFeaturesToTrack(old_img, mask = None, **feature_params)
    else: 
        mask = np.zeros_like(old_img)
        frame_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        p1, st, err = cv2.calcOpticalFlowPyrLK(old_img, frame_gray, p0, None, **lk_params)
        # Select good points
        good_new = p1[st==1]
        good_old = p0[st==1]
        # draw the tracks
        for i,(new,old) in enumerate(zip(good_new,good_old)):
            a,b = new.ravel()
            c,d = old.ravel()
            mask = cv2.line(mask, (a,b),(c,d), color[i].tolist(), 2)
            frame = cv2.circle(img,(a,b),5,color[i].tolist(),-1)
        # img = cv2.add(img,mask)
        showPic(img)
        old_img = frame_gray.copy()
        p0 = good_new.reshape(-1,1,2)
    return img


#TODO: Fine Tune those parameters, go read the documents for specific effects
def dense_montionEffects(img, which_frame, out): 
    global hsv
    global prvs
    if(which_frame == 0):
        prvs = cv2.cvtColor(img ,cv2.COLOR_BGR2GRAY)
        hsv = np.zeros_like(img)
        hsv[...,1] = 255

    else:
        frame2 = img 
        next = cv2.cvtColor(frame2,cv2.COLOR_BGR2GRAY)
        flow = cv2.calcOpticalFlowFarneback(prvs,next, None, 0.5, 3, 15, 3, 5, 1.2, 0)
        mag, ang = cv2.cartToPolar(flow[...,0], flow[...,1])
        hsv[...,0] = ang*180/np.pi/2
        hsv[...,2] = cv2.normalize(mag,None,0,255,cv2.NORM_MINMAX)
        rgb = cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)
        prvs = next
        out.write(rgb)
        # showPic(rgb)
        # cv2.imshow('frame2',rgb)
        # k = cv2.waitKey(30) & 0xff
        # if k == 27:
            # break
        # elif k == ord('s'):
            # cv2.imwrite('opticalfb.png',frame2)
            # cv2.imwrite('opticalhsv.png',rgb)


def feature_detection(img):
    orb = cv2.ORB_create()
    kp = orb.detect(img,None)
    kp, des = orb.compute(img, kp)
    img2 = np.empty_like(img)
    cv2.drawKeypoints(img,kp,img2, color=(0,255,0))
    showPic(img2)

#First parameter for frame, second for the comparing image
#TODO: Figure out a way in Ubuntu to install SURF and SIFT for scale invariant feature mapping
def feature_matching(img1, img2):
    gray1 = cv2.cvtColor(img1.copy(), cv2.COLOR_BGR2GRAY)
    gray2 = cv2.cvtColor(img2.copy(), cv2.COLOR_BGR2GRAY)
    detector = cv2.AKAZE_create()
    kp1, des1 = detector.detectAndCompute(img1,None)
    kp2, des2 = detector.detectAndCompute(img2,None)
    bf = cv2.BFMatcher()
    matches = bf.knnMatch(des1,des2, k=2)
    good = []
    for m,n in matches:
        if m.distance < 0.75*n.distance:
            good.append([m])
    img3 = cv2.drawMatchesKnn(img1,kp1,img2,kp2,good,None,flags=2)
    showPic(img3)
    # out.write(img3)



camera = cv2.VideoCapture("walk.mp4")
counter = 0
fourcc = cv2.VideoWriter_fourcc(*'avc1')
out = cv2.VideoWriter('output.avi',fourcc, 25.0, (480,270))



while True:
    grabbed, frame = camera.read()
    print np.shape(frame)
    if grabbed:
        # cv2.imshow('image', frame)
        # detectLines(frame.copy())
        dense_montionEffects(frame, counter, out)
        counter += 1
        # feature_detection(frame)
        # img1 = cv2.imread("exp.jpg")TypeError: Required argument 'outImg' (pos 6) not found
        # img2 = cv2.imread("exp1.jpg")
        # feature_matching(img1, img2)
    else:
        print("No video feed available")

    
    
