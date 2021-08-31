import numpy as np
import cv2
import glob
from imutils.video import VideoStream
import time
import os
import sys

# getting the name of the directory
# where the this file is present.
current = os.path.dirname(os.path.realpath(__file__))
  
# Getting the parent directory name
# where the current directory is present.
parent = os.path.dirname(current)
  
# adding the parent directory to 
# the sys.path.
sys.path.append(parent)
  
# now we can import the module in the parent
# directory.
from  utilities import getVideoCap
USE_USB_CAMERA=True

def getCalibrationPhotos(numOfPhotos=25):

    # For webcam input:
    cap = getVideoCap(usb=USE_USB_CAMERA)

    time.sleep(2.0)
    
    success, image = cap.read()
    count = 0
    while success and count<numOfPhotos:
        cv2.imshow("Calibration photo",image)
        cv2.imwrite("test_photos/%d.jpg" % (count+1), image)     # save frame as JPEG file
        time.sleep(2.0)
        #cv2.waitKey(0)#wait key to get next photo      
        success,image = cap.read()
        print('Read a new frame: ', success)
        count += 1


if __name__=="__main__":
    getCalibrationPhotos(numOfPhotos=5)