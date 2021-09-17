import numpy as np
import cv2
from cv2 import bgsegm
import os,sys
import camera_model
from os.path import dirname, abspath

def add_parent_folder_to_path():
    print ("current file:",cv2.__file__)
    # getting the name of the directory
    # where the this file is present.
    current = os.path.dirname(os.path.realpath(__file__))
    
    # Getting the parent directory name
    # where the current directory is present.
    parent = os.path.dirname(current)
    
    # adding the parent directory to 
    # the sys.path.
    sys.path.append(parent)

add_parent_folder_to_path()

from utilities import getVideoCap 
from calibration import calibration

#cap = cv2.VideoCapture('people-walking.mp4')
#cap = cv2.VideoCapture('Ball_Bouncing.mp4')
cap = cv2.VideoCapture('/home/marios/catkin_ws/src/thesis_drone/src/Drone_Pose_Estimation/test_videos/phone_edited.mp4')
# cap = cv2.VideoCapture(0)

AREA_THRESHOLD=400

def get_max_contour_area(contours):
    max_area=0
    max_i=0
    for i,c in enumerate(contours):
        if cv2.contourArea(c) >max_area :
            max_area=cv2.contourArea(c)
            max_i=i

    return max_area, max_i

def draw_bounding_box(max_contour,frame):
    x,y,w,h = cv2.boundingRect(max_contour)
    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255,0), 2)
    center = ( x+w/2 ,y+h/2 )
    
    max_area=cv2.contourArea(max_contour)
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(frame, "area:%d"%(max_area),(450,30) ,font, 1, (0, 0, 255),2, cv2.FILLED)
    cv2.putText(frame, "x: %d"%(center[0]), (450,60) ,font, 1, (0, 0, 255),2, cv2.FILLED)
    cv2.putText(frame, "y: %d"%(center[1]), (450,90) ,font, 1, (0, 0, 255),2, cv2.FILLED)
    
    return center

def calculate_pol_for_area_to_depth():
    actual_depths=[1,2]
    areas=[4000,8000]
    p = np.polyfit(x=areas, y=actual_depths, deg=1)
    return p

def save_pol_for_area_to_depth(path,pol):
    """ Save the polynomial of the area estimation to given path/file. """
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
    cv_file.write("pol", pol)
    # note you *release* you don't close() a FileStorage object
    cv_file.release()

def load_pol_for_area_to_depth(path):
    """ Loads camera matrix and distortion coefficients. """
    # FILE_STORAGE_READ
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)

    # note we also have to specify the type to retrieve other wise we only get a
    # FileNode object back instead of a matrix
    pol = cv_file.getNode("pol").mat()
    
    cv_file.release()
    return pol

def estimate_z_based_on_contour(contour,p:list)->float:
    area = cv2.contourArea(contour)
    depth=np.polyval(p,area)

    return depth

def get_center_of_contour(contour)->list:
    # compute the center of the contour
    M = cv2.moments(contour)
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    center = [cx,cy]

    return center

class pose_extractor_CV_techniques:
    def __init__(self,USB_cam=1):
        # self.cap = getVideoCap(usb=USB_cam)
        self.cap=cv2.VideoCapture('/home/marios/catkin_ws/src/thesis_drone/src/Drone_Pose_Estimation/test_videos/phone_edited.mp4')
        parent_dir_path = dirname(dirname(abspath(__file__)))
        params=calibration.load_coefficients(parent_dir_path+"/calibration/cameraCoeffs.yml")
        print(params)
        self.matrix_coefficients,self.distortion_coefficients=params[0],params[1]
        self.background_subtract = cv2.createBackgroundSubtractorMOG2(history = 100, varThreshold = 16, detectShadows =False)
        self.z_estimation_polynomial=load_pol_for_area_to_depth('pol_area_depth.yml')

    def visualise(self,frame,gblur):
        cv2.imshow('original', frame)
        cv2.imshow('first frame', gblur)

        k = cv2.waitKey(30) & 0xff
        if k == 27:
            self.kill()
    
    def estimate_xy_based_on_z(self,point,z):
        xy = camera_model.Unproject(np.float32( np.array([point,]) ),z,self.matrix_coefficients,self.distortion_coefficients)
        print("xy from estimation:",xy[0])
        return xy[0]

    def getPose(self):
        ret, frame = self.cap.read()

        first_frame = self.background_subtract.apply(frame)
        gblur = cv2.GaussianBlur(first_frame, (5, 5), 0)

        #contouring
        #threshold : Separate out regions of an image corresponding to objects which we want to analyze.
        ret, threshold = cv2.threshold(gblur, 200, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        # Thresholding Technique - cv2.THRESH_BINARY,cv2.THRESH_BINARY_INV,cv2.THRESH_TRUNC,cv2.THRESH_TOZERO,cv2.THRESH_TOZERO_INV etc
        contours,_ = cv2.findContours(threshold, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        if len(contours)==0:
            return None,None,False

        found_pose=False
        max_area, max_i = get_max_contour_area(contours)
        max_area_criterion = max_area > AREA_THRESHOLD
        if not max_area_criterion:
            return None,None,False

        # cv2.drawContours(frame, contours, -1, (0, 0, 255), 6)
        max_contour=contours[max_i]
        draw_bounding_box(max_contour,frame)
        z = estimate_z_based_on_contour(max_contour,self.z_estimation_polynomial)
        contour_center = get_center_of_contour(max_contour)
        xy= self.estimate_xy_based_on_z(contour_center,z)
        
        tvec = [ xy[0], xy[1], z]
        rpy=[0,0,0]

        self.visualise(frame,gblur)
        
        return tvec,rpy,True
    
    def kill(self):
        # When everything done, release the capture
        self.cap.release()
        cv2.destroyAllWindows()

pose_estimator=pose_extractor_CV_techniques()
while (1):
    pose_estimator.getPose()

if __name__=="__main__":
    p = calculate_pol_for_area_to_depth()
    # save_pol_for_area_to_depth('pol_area_depth.yml',p)
    p=load_pol_for_area_to_depth('pol_area_depth.yml')
    print(p)