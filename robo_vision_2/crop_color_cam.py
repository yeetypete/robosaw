# Calibrate the camera for cropping the frame
# press 's' key to save the calibration
# press 'esc' key to exit

import cv2
import numpy as np
import time
# A required callback method that goes into the trackbar function.
def nothing(x):
    pass

# Initializing the camera feed.
cam_id_array = np.load('__calibrate__/color_angle_center_cam_id_array.npy')
color_cam_id = int(cam_id_array[0])
cap = cv2.VideoCapture(color_cam_id)

width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
# Create a window named trackbars.
cv2.namedWindow("Trackbars", cv2.WINDOW_AUTOSIZE)

# trackbars for each edge
cv2.createTrackbar("left", "Trackbars", 0, int(width/2) - 1, nothing)
cv2.createTrackbar("right", "Trackbars", 0, int(width/2) - 1, nothing)
cv2.createTrackbar("top", "Trackbars", 0, int(height/2) - 1, nothing)
cv2.createTrackbar("bottom", "Trackbars", 0, int(height/2) - 1, nothing)

while True:
    
    # Start reading the cam feed frame by frame.
    ret, frame = cap.read()
    if not ret:
        break
    
    
    # Get the new values of the trackbar in real time as the user changes 
    # them
    left = cv2.getTrackbarPos("left", "Trackbars")
    right = cv2.getTrackbarPos("right", "Trackbars")
    top = cv2.getTrackbarPos("top", "Trackbars")
    bottom = cv2.getTrackbarPos("bottom", "Trackbars")
 
    cropped_image = frame[top:height-bottom, left:width-right]
    
    #cv2.imshow('Trackbars',cv2.resize(cropped_image,None,fx=0.5,fy=0.5))
    cv2.imshow('Preview',cropped_image)
    
    # If the user presses ESC then exit the program
    key = cv2.waitKey(1)
    if key == 27:
        break
    
    # If the user presses `s` then print this array and save it to the saw
    if key == ord('s'):
        
        thearray = [[top,height-bottom], [left,width-right]]
        print(thearray)
        
        # Also save this array as penval.npy
        np.save('__calibrate__/color_cam_top_bottom_left_right',thearray)
        break
    
# Release the camera & destroy the windows.    
cap.release()
cv2.destroyAllWindows()
