#finding hsv range of target object
# press 's' key to save the calibration
# press 'esc' key to exit
import cv2
import numpy as np
import time
# A required callback method that goes into the trackbar function.
def nothing(x):
    pass

# Initializing the cam feed.
cam_id_array = np.load('__calibrate__/color_angle_center_cam_id_array.npy')
color_cam_id = int(cam_id_array[0])
print("Color cam ID: " + str(color_cam_id))
cap = cv2.VideoCapture(color_cam_id)
time.sleep(2)
#cap.set(3,1280)
#cap.set(4,720)


print("Loading...")

# Create a window named trackbars.
cv2.namedWindow("Trackbars", cv2.WINDOW_AUTOSIZE)

try:
    hsv = np.load('__calibrate__/hsv_value1.npy')
    h_lower_thresh1 = hsv[0][0]
    h_upper_thresh1 = hsv[1][0]
    s_lower_thresh1 = hsv[0][1]
    s_upper_thresh1 = hsv[1][1]
    v_lower_thresh1 = hsv[0][2]
    v_upper_thresh1 = hsv[1][2]
except:
    print("No values found, using defaults.")
    h_lower_thresh1 = 0
    h_upper_thresh1 = 179
    s_lower_thresh1 = 0
    s_upper_thresh1 = 255
    v_lower_thresh1 = 0
    v_upper_thresh1 = 255

# Now create 6 trackbars that will control the lower and upper range of 
# H,S and V channels. The Arguments are like this: Name of trackbar, 
# window name, range,callback function. For Hue the range is 0-179 and
# for S,V its 0-255.
cv2.createTrackbar("L - H", "Trackbars", h_lower_thresh1, 179, nothing)
cv2.createTrackbar("L - S", "Trackbars", s_lower_thresh1, 255, nothing)
cv2.createTrackbar("L - V", "Trackbars", v_lower_thresh1, 255, nothing)
cv2.createTrackbar("U - H", "Trackbars", h_upper_thresh1, 179, nothing)
cv2.createTrackbar("U - S", "Trackbars", s_upper_thresh1, 255, nothing)
cv2.createTrackbar("U - V", "Trackbars", v_upper_thresh1, 255, nothing)
 


while cap.isOpened():
    
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
    l_h = cv2.getTrackbarPos("L - H", "Trackbars")
    l_s = cv2.getTrackbarPos("L - S", "Trackbars")
    l_v = cv2.getTrackbarPos("L - V", "Trackbars")
    u_h = cv2.getTrackbarPos("U - H", "Trackbars")
    u_s = cv2.getTrackbarPos("U - S", "Trackbars")
    u_v = cv2.getTrackbarPos("U - V", "Trackbars")
 
    # Set the lower and upper HSV range according to the value selected
    # by the trackbar
    lower_range = np.array([l_h, l_s, l_v])
    upper_range = np.array([u_h, u_s, u_v])
    
    # Filter the image and get the binary mask, where white represents 
    # your target color
    mask = cv2.inRange(hsv, lower_range, upper_range)
 
    # You can also visualize the real part of the target color (Optional)
    res = cv2.bitwise_and(frame, frame, mask=mask)
    
    # Converting the binary mask to 3 channel image, this is just so 
    # we can stack it with the others
    mask_3 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    
    # stack the mask, orginal frame and the filtered result
    stacked = np.hstack((mask_3,frame,res))
    
    # Show this stacked frame at 40% of the size.
    cv2.imshow('Set Top Color Mask, press ESC to close s to save',cv2.resize(stacked,None,fx=0.7,fy=0.7))
    
    # If the user presses ESC or q then exit the program
    key = cv2.waitKey(1)
    if key == 27:
        break
    if key == ord('q'):
        break

    # If the user presses `s` then print this array.
    if key == ord('s'):
        
        thearray = [[l_h,l_s,l_v],[u_h, u_s, u_v]]
        print(thearray)
        
        # Also save this array as __calibrate__/hsv_value.npy
        np.save('__calibrate__/hsv_value1',thearray)
        break
    
# Release the camera & destroy the windows.    
cap.release()
cv2.destroyAllWindows()