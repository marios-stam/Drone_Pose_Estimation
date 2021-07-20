import numpy as np
import cv2
import cv2.aruco as aruco
import calibration

def track(matrix_coefficients, distortion_coefficients):
    CAMERA_INDEX=0
    cap = cv2.VideoCapture(CAMERA_INDEX)
    while True:
        ret, frame = cap.read()
        # operations on the frame come here
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Change grayscale
        # aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)  # Use 5x5 dictionary to find markers
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)  # Use 5x5 dictionary to find markers
        parameters = aruco.DetectorParameters_create()  # Marker detection parameters
        # lists of ids and the corners beloning to each id
        
        corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict,
                                                                parameters=parameters,
                                                                cameraMatrix=matrix_coefficients,
                                                                distCoeff=distortion_coefficients)

        #if no calibration has been done
        #corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict,
        #                                                        parameters=parameters)
        
        if np.all(ids is not None):  # If there are markers found by detector
            for i in range(0, len(ids)):  # Iterate in markers
                # Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients

                rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients,
                                                                           distortion_coefficients)

                (rvec - tvec).any()  # get rid of that nasty numpy value array error
                aruco.drawDetectedMarkers(frame, corners)  # Draw A square around the markers
                aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)  # Draw Axis
                # Display the resulting frame
                # cv2.imshow('frame', frame)
                # Wait 3 milisecoonds for an interaction. Check the key and do the corresponding job.
               
        else:
            pass
            #print("mlkia")
            
        cv2.imshow('frame', frame)
        key = cv2.waitKey(3) & 0xFF
        if key == ord('q'):  # Quit
            break
                
    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
        
if __name__=="__main__":
    camera_matrix, dist_matrix=calibration.load_coefficients("cameraCoeffs.yml")
    track(camera_matrix,dist_matrix)