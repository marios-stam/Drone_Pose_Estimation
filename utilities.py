import cv2
from imutils.video import VideoStream

def getVideoCap(usb=True):
    int_camera_index=0
    usb_camera_index=2
    if(usb):
        cap = cv2.VideoCapture(usb_camera_index)
        cap.set(cv2.CAP_PROP_FPS, 30)
    else:
        cap = cv2.VideoCapture(int_camera_index)
    
    return cap