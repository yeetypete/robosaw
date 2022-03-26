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
center_cam_id = int(cam_id_array[2])
cap = cv2.VideoCapture(center_cam_id)

width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
# Create a window named trackbars.
cv2.namedWindow("Trackbars", cv2.WINDOW_AUTOSIZE)

# trackbars for each edge
cv2.createTrackbar("left-right", "Trackbars", int(width/2), int(width), nothing)
cv2.createTrackbar("up-down", "Trackbars", int(height/2), int(height), nothing)
cv2.createTrackbar("Radius", "Trackbars", int(width/3), int(width/2) - 1, nothing)

while True:
    
    # Start reading the cam feed frame by frame.
    ret, frame = cap.read()
    if not ret:
        break
    
    
    # Get the new values of the trackbar in real time as the user changes 
    # them
    

    center_coordinates = (cv2.getTrackbarPos("left-right", "Trackbars"),cv2.getTrackbarPos("up-down", "Trackbars"))
    radius = cv2.getTrackbarPos("Radius", "Trackbars")
    circle_mask = np.zeros(shape=frame.shape, dtype=np.uint8)
    color = 255
    thickness = -1
    circle_mask = cv2.circle(circle_mask, center_coordinates, radius, color, thickness)
    circle_mask = cv2.inRange(circle_mask, 254, 256)
    cropped_image = cv2.bitwise_and(frame, frame, mask=circle_mask)

    # make crosshairs
    pt1 = (center_coordinates[0],center_coordinates[1]-radius)
    pt2 = (center_coordinates[0],center_coordinates[1]+radius)
    cv2.line(cropped_image,pt1,pt2,(255,0,50),2, cv2.LINE_AA)
    pt1 = (center_coordinates[0]-radius,center_coordinates[1])
    pt2 = (center_coordinates[0]+radius,center_coordinates[1])
    cv2.line(cropped_image,pt1,pt2,(255,0,50),2, cv2.LINE_AA)

    #cv2.imshow('Trackbars',cv2.resize(cropped_image,None,fx=0.5,fy=0.5))
    cv2.imshow('Preview',cropped_image)
    
    # If the user presses ESC then exit the program
    key = cv2.waitKey(1)
    if key == 27:
        break
    
    # If the user presses `s` then print this array and save it to the saw
    if key == ord('s'):
        
        thearray = [[center_coordinates[0],center_coordinates[1]], [radius]]
        print(thearray)
        
        # Also save this array as penval.npy
        np.save('__calibrate__/center_cam_x_y_radius',thearray)
        break
    
# Release the camera & destroy the windows.    
cap.release()
cv2.destroyAllWindows()


