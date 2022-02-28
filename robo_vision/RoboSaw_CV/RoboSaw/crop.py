# Cropping an image
import cv2
import numpy as np
import time
# A required callback method that goes into the trackbar function.
def nothing(x):
    pass

# Initializing the webcam feed.
cap = cv2.VideoCapture(1)
#cap.set(3,1280)
#cap.set(4,720)
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
# Create a window named trackbars.
cv2.namedWindow("Trackbars")

# Now create 6 trackbars that will control the lower and upper range of 
# H,S and V channels. The Arguments are like this: Name of trackbar, 
# window name, range,callback function. For Hue the range is 0-179 and
# for S,V its 0-255.
cv2.createTrackbar("left", "Trackbars", 0, int(width/2) - 1, nothing)
cv2.createTrackbar("right", "Trackbars", 0, int(width/2) - 1, nothing)
cv2.createTrackbar("top", "Trackbars", 0, int(height/2) - 1, nothing)
cv2.createTrackbar("bottom", "Trackbars", 0, int(height/2) - 1, nothing)

while True:
    
    # Start reading the webcam feed frame by frame.
    ret, frame = cap.read()
    if not ret:
        break
    # Flip the frame horizontally (Not required)
    #frame = cv2.flip( frame, 1 )
    
    # Get the new values of the trackbar in real time as the user changes 
    # them
    left = cv2.getTrackbarPos("left", "Trackbars")
    right = cv2.getTrackbarPos("right", "Trackbars")
    top = cv2.getTrackbarPos("top", "Trackbars")
    bottom = cv2.getTrackbarPos("bottom", "Trackbars")
 
    cropped_image = frame[top:height-bottom, left:width-right]
    
    #cv2.imshow('Trackbars',cv2.resize(cropped_image,None,fx=0.5,fy=0.5))
    cv2.imshow('Trackbars',cropped_image)
    
    # If the user presses ESC then exit the program
    key = cv2.waitKey(1)
    if key == 27:
        break
    
    # If the user presses `s` then print this array.
    if key == ord('s'):
        
        thearray = [[top,height-bottom], [left,width-right]]
        print(thearray)
        
        # Also save this array as penval.npy
        np.save('left,right,top,bottom',thearray)
        break
    
# Release the camera & destroy the windows.    
cap.release()
cv2.destroyAllWindows()