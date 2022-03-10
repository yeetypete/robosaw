import cv2
import numpy as np
from Model import Model

def open_cameras(model):
    """ Initializes the three camrea feeds
            returns array of captures: [color, angle, center]"""
    color_cap = cv2.VideoCapture(model.color_cam_id)
    angle_cap = cv2.VideoCapture(model.angle_cam_id)
    center_cap = cv2.VideoCapture(model.center_cam_id)
    if (not color_cap.isOpened() or not angle_cap.isOpened() or not center_cap.isOpened()):
        print("Cannot open camera")
        exit()
    return [color_cap, angle_cap, center_cap]

def display_cap(model, cap):
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
 
        # Display the resulting frame
        cv.imshow("Press 'q' to quit", frame)
        if cv.waitKey(1) == ord('q'):
            break
    # When everything done, release the capture
    cap.release()
    cv.destroyAllWindows()

