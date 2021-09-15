import cv2
import numpy as np
import os

# print(os.getcwd())
print(cv2.__file__)
img = cv2.imread('src/thesis_drone/src/Drone_Pose_Estimation/test_photos/CV_1.jpg')
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
edges = cv2.Canny(gray,threshold1=10,threshold2=200,apertureSize = 3)
# cv2.imshow("edges",edges)
# cv2.waitKey(0)

lines = cv2.HoughLines(edges,rho=1,theta=np.pi/180,threshold=1)
print(lines[0])
for rho,theta in lines[0]:
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a*rho
    y0 = b*rho
    x1 = int(x0 + 1000*(-b))
    y1 = int(y0 + 1000*(a))
    x2 = int(x0 - 1000*(-b))
    y2 = int(y0 - 1000*(a))

    cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)

cv2.imshow("img",img)
cv2.waitKey(0)