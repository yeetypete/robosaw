import cv2
import numpy as np
from Model import Model

def disp_img_processing(model):
    cap = cv2.VideoCapture(1) # Change this depending on device and camera used
    while (True):
        ret , frame = cap.read()

        #image processing:
        grey = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(grey,(5,5),cv2.BORDER_DEFAULT)
        edges = cv2.Canny(blur,50,150,apertureSize = 3)
        lines = cv2.HoughLines(edges,1,np.pi/180,150)

        #checking for best line:
        if lines is not None:
            for i in range(0,len(lines)):
                rho = lines[i][0][0]
                theta = lines[i][0][1]
                if model.theta_is_valid(theta):
                    #adding line to image
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a * rho
                    y0 = b * rho
                    pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
                    pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
                    cv2.line(frame,pt1,pt2,(255,100,200),5, cv2.LINE_AA)
                    break
        


        cv2.imshow('RoboVision: press "q" to quit application', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    disp_current_view()