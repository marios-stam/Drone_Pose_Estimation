from numpy.lib.function_base import extract
from utilities import getVideoCap
from tracking import pose_estimation_simple
from calibration import calibration
import cv2

class pose_extractor:
    def __init__(self,USB_cam=1):
        self.cap = getVideoCap(usb=USB_cam)
        self.matrix_coefficients,self.distortion_coefficients=calibration.load_coefficients("calibration/cameraCoeffs.yml")

    def getPose(self):
        ret, frame = self.cap.read()

        matrix_coefficients, distortion_coefficients=self.matrix_coefficients,self.distortion_coefficients
        frame,tvec,rpy= pose_estimation_simple(frame, matrix_coefficients, distortion_coefficients)
        frame=cv2.flip(frame, 1) #flip frame to make it easier for user to test it live

        cv2.imshow('frame', frame)
        key = cv2.waitKey(3) & 0xFF
        if key == ord('q'):  # Quit
            self.kill()
            return None,None

        if len(tvec)>0:
            return tvec,rpy
        else:
            return None,None

    def kill(self):
        # When everything done, release the capture
        self.cap.release()
        cv2.destroyAllWindows()

if __name__=="__main__":
    pose_estimator=pose_extractor(USB_cam=0)

    while True:
        pose_estimator.getPose()