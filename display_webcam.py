# Python program to illustrate HoughLine
# method for line detection
import cv2
import numpy as np
  
cap = cv2.VideoCapture(0)
# The device number might be 0 or 1 depending on the device and the webcam
#cap.open(0, cv2.CAP_DSHOW)
while(True):
    # Capture a frame from the webcam
    ret , frame = cap.read()

    # Convert the capture to grayscale
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    # detect edges using canny edge detection
    edges = cv2.Canny(gray,50,150,apertureSize = 3)

    # find lines and get the equations for them
    lines = cv2.HoughLines(edges,1,np.pi/180,200)
    if lines is not None:
        for rho,theta in lines[0]:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))
            # Save the new image over frame to save space since it is already allocated.
            cv2.line(frame,(x1,y1),(x2,y2),(0,0,255),2)

    cv2.imshow('demo - press "q" to quit application', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()



# Reading the required image in 
# which operations are to be done. 
# Make sure that the image is in the same 
# directory in which this python program is

  

