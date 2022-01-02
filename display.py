import cv2
import numpy as np
from Model import Model

def disp_model(model):
    WIDTH = 666
    HEIGHT = 420
    black_image = np.zeros(shape=[HEIGHT,WIDTH,3], dtype = np.uint8)
    rho = model.detected_rho
    theta = model.detected_theta

    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a*rho
    y0 = b*rho
    x1 = int(x0 + 2*np.maximum(HEIGHT,WIDTH)*(-b))    
    y1 = int(y0 + 2*np.maximum(HEIGHT,WIDTH)*(a))
    x2 = int(x0 - 2*np.maximum(HEIGHT,WIDTH)*(-b))
    y2 = int(y0 - 2*np.maximum(HEIGHT,WIDTH)*(a))

    # Draw the coordinate lines centered at pivot point
    cv2.line(black_image,(0,int(model.y_off)),(int(WIDTH),int(model.y_off)),(255,100,0),2) # top line intersects the pivot point of the saw
    cv2.line(black_image,(int(model.x_off),int(model.y_off-5)),(int(model.x_off),int(model.y_off+5)),(255,100,0),2) # centerline of pivot point
    
    # Draw the detected line
    cv2.line(black_image,(x1,y1),(x2,y2),(0,255,0),4) # detected line
    #cv2.line(black_image,(0,0),(int(x0),int(y0)),(0,255,100),1) # guide line
    #cv2.line(black_image,(int(x0),int(y0)),(int(x0),int(model.y_off)),(0,255,100),1) # guide line

    # Draw the intersection point and find distance to pivot point
    offset = model.find_offset()
    black_image = cv2.circle(black_image, (int(offset),int(model.y_off)), radius=1, color=(0, 0, 255), thickness=2)
    #print("Offset: " + str(offset))

    #Draw the saw blade
    radius = 50
    black_image = cv2.circle(black_image, (int(model.x_off),int(model.y_off)), radius=radius, color=(0, 0, 200), thickness=1)
    cv2.line(black_image,(int(model.x_off),int(model.y_off)),(int(-(radius+20)*np.sin(model.blade_theta)+model.x_off), int((radius+20)*np.cos(model.blade_theta)+model.y_off)),(0,0,200),4)

    cv2.imshow("Robo-Vision", black_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def disp_current_view():
    angle = 0
    cap = cv2.VideoCapture(1) # 1 for webacam 0 for built in camera
    max_angle = 0.9
    min_angle = 0
    threshold = 200
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
        lines = cv2.HoughLines(edges,1,np.pi/180,threshold)
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
                if (theta < max_angle and theta > min_angle):
                    angle = theta
                    cv2.line(frame,(x1,y1),(x2,y2),(200,0,200),5)
            
             

        font                   = cv2.FONT_HERSHEY_SIMPLEX
        bottomLeftCornerOfText = (10,200)
        fontScale              = 1
        fontColor              = (255,255,255)
        thickness              = 3
        lineType               = 2

        cv2.putText(frame,'Angle: '+str(angle)  + '   Threshold: ' + str(threshold), 
        bottomLeftCornerOfText, 
        font, 
        fontScale,
        fontColor,
        thickness,
        lineType)
        cv2.imshow('Robo-preview: press "q" to quit application', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    disp_current_view()