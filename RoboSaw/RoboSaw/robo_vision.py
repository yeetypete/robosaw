import cv2
import numpy as np
#from Model import Model

def disp_img_processing():
    cap = cv2.VideoCapture(0,cv2.CAP_DSHOW) # 0 for webacam 1 for built in camera
    while (True):
        ret , frame = cap.read()
        grey = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(grey,(5,5),cv2.BORDER_DEFAULT)
        edges = cv2.Canny(blur,50,150,apertureSize = 3)
        lines = cv2.HoughLines(edges,1,np.pi/180,150)
        if lines is not None:
            rho = lines[0][0][0]
            theta = lines[0][0][1]
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a * rho
            y0 = b * rho
            pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
            pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
            cv2.line(blur,pt1,pt2,(255,100,200),5, cv2.LINE_AA)

        cv2.imshow('RoboVision: press "q" to quit application', blur)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    disp_current_view()