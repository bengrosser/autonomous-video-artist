#-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.
 #File Name : exp_alpha.py
 #Creation Date : 14-06-2017
 #Last Modified : Wed Jun 14 13:21:47 2017
 #Created By : Rui An  
#_._._._._._._._._._._._._._._._._._._._._.
import numpy as np
import cv2 

img1 = cv2.imread("./src_picture/exp.jpg")
img2 = np.full((500,1000,3), 255, np.uint8)
img3 = cv2.ellipse(img2, (200,200), (150,200), 0, 0, 360, (0,0,0), 5)
img4 = cv2.ellipse(img2, (200,200), (150,200), 0, 0, 360, (200,200,200), 5)
# cv2.imshow("shit",img4)
# cv2.waitKey(0)

alpha = 0.5 
# img1_1 = alpha*img3
# print img1_1
# "------------------------------------------------"
# img2_2 = (1-alpha)*img1
# print img2_2
# "------------------------------------------------"
# img3_3 = img1_1 + img2_2
# print img3_3
x,y,z = img1.shape
for i in range(x):
    for j in range(y):
        if(img3[i][j][0] != 255): 
            result_color = alpha*img3[i][j] + (1-alpha)*img1[i][j]
            img1[i][j] = result_color
            
            # continue
            # print img3_3[i][j]
# "------------------------------------------------"
# result = img3_3.astype(np.uint8)
# result = cv2.cvtColor(result, cv2.COLOR_BGR2RGB)
# result = cv2.cvtColor(result,cv2.COLOR_RGB2BGR)
cv2.imshow("result", img1)
cv2.waitKey(0)







