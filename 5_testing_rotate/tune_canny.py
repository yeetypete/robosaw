#finding hsv range of target object
# press 's' key to save the calibration
# press 'esc' key to exit
import cv2
import argparse
import numpy as np
import time
# A required callback method that goes into the trackbar function.
def nothing(x):
    pass

# Initializing the cam feed.
cam_id_array = np.load('__calibrate__/color_angle_center_cam_id_array.npy')
center_cam_id = int(cam_id_array[2])
print("Color cam ID: " + str(center_cam_id))
cap = cv2.VideoCapture(center_cam_id)
#time.sleep(2)
#cap.set(3,1280)
#cap.set(4,720)

# Create a window named trackbars.
cv2.namedWindow("Trackbars", cv2.WINDOW_AUTOSIZE)

# Now create 6 trackbars that will control the lower and upper range of 
# H,S and V channels. The Arguments are like this: Name of trackbar, 
# window name, range,callback function. For Hue the range is 0-179 and
# for S,V its 0-255.
try:
    thresh_ratio = np.load('__calibrate__/canny_tresh_ratio.npy')
    low_threshold = thresh_ratio[0]
    ratio = thresh_ratio[1]
    cv2.createTrackbar("Threshold", "Trackbars", low_threshold, 100, nothing)
    cv2.createTrackbar("Ratio", "Trackbars", ratio, 5, nothing)
except:
    cv2.createTrackbar("Threshold", "Trackbars", 0, 100, nothing)
    cv2.createTrackbar("Ratio", "Trackbars", 0, 5, nothing)

 
while True:
    
    # Start reading the webcam feed frame by frame.
    ret, frame = cap.read()
    if not ret:
        break
    # Flip the frame horizontally (Not required)
    #frame = cv2.flip( frame, 1 ) 
    #frame = cv2.GaussianBlur(frame,(5,5),cv2.BORDER_DEFAULT)
    # Convert the BGR image to HSV image.
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Get the new values of the trackbar in real time as the user changes 
    # them
    low_threshold = cv2.getTrackbarPos("Threshold", "Trackbars")
    ratio = cv2.getTrackbarPos("Ratio", "Trackbars")
    
    frame1 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame1 = cv2.GaussianBlur(frame1, (3, 3), 0)
    frame1 = cv2.GaussianBlur(frame1,(5,5),cv2.BORDER_DEFAULT)
    detected_edges = cv2.Canny(frame1,low_threshold, low_threshold*ratio ,apertureSize = 3)

    
 
    
    # Converting the binary mask to 3 channel image, this is just so 
    # we can stack it with the others
    mask_3 = cv2.cvtColor(detected_edges, cv2.COLOR_GRAY2BGR)
    
    # stack the mask, orginal frame and the filtered result
    stacked = np.hstack((mask_3,frame))
    
    # Show this stacked frame at 40% of the size.
    cv2.imshow('Set Top Color Mask, press ESC to close s to save',cv2.resize(stacked,None,fx=0.8,fy=0.8))
    
    # If the user presses ESC or q then exit the program
    key = cv2.waitKey(1)
    if key == 27:
        break
    if key == ord('q'):
        break
    
    # If the user presses `s` then print this array.
    if key == ord('s'):
        
        thearray = [low_threshold,ratio]
        print(thearray)
        
        # Also save this array as __calibrate__/hsv_value.npy
        np.save('__calibrate__/canny_tresh_ratio',thearray)
        break
    
# Release the camera & destroy the windows.    
cap.release()
cv2.destroyAllWindows()