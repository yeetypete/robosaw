import cv2
import numpy as np
from Model import Model

def start_camera_feed(camera_id):
    """ returns caprture """
    cap = cv2.VideoCapture(camera_id) # Change this depending on device and camera used
    return cap

def lines_are_close(angle1,dist1, angle2,dist2):
    if (abs(angle1 - angle2) < Model.min_angle_diff and abs(dist1 - dist2) < Model.min_dist_diff):
        return True
    else:
        return False

def check_for_line(cap, model): 
    ret , frame = cap.read()
    line_detected = False
    out_angle = None
    out_distance = None
    #image processing:
    grey = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(grey,50,150,apertureSize = 3)
    lines = cv2.HoughLines(edges,1,np.pi/180,model.line_detection_threshold)

    #checking for best line:
    #line with higest accumulator value is first in the list of lines[]
    if lines is not None:
        line_detected = True
        
        # Get rho and theta from the one most prominent line
        rho = lines[0][0][0]
        theta = lines[0][0][1]
        angle = np.degrees(abs(theta))

        # correct for weird angle issue
        if angle > 180-model.max_angle:
            angle = (angle - 180)

        if (abs(angle) < model.max_angle):
            out_angle = angle
        
        # Now find the distance of the line from the left edge of frame
        if theta == 0:
            out_distance =  abs(rho)
        if model.y_off == 0:
            out_distance =  rho/np.cos(theta)
        if rho > 0:
            out_distance =  (rho/np.cos(theta)-model.y_off*np.tan(theta))
        if rho < 0:
            out_distance =  (rho/np.cos(theta)+model.y_off*np.tan(theta))
        if rho == 0:
            if theta > np.pi/2:
                out_distance =  (-model.y_off*np.tan(theta))
            if theta < np.pi/2:
                out_distance =  (model.y_off/np.tan(theta))
        else:
            return None

    return [line_detected, out_distance, out_angle]

def check_for_edge(cap, model):
    ret , frame = cap.read()
    line_detected = False
    out_angle = None
    out_distance = None

    #image processing:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) #color space transformation to hsv
    lower_green = np.array([model.h_lower_thresh,model.s_lower_thresh,model.v_lower_thresh])
    upper_green = np.array([model.h_upper_thresh,model.s_upper_thresh,model.v_upper_thresh])
    mask = cv2.inRange(hsv, lower_green, upper_green) #threshold the image to only show green pixels
    mask = cv2.GaussianBlur(mask,(5,5),0)
    edges = cv2.Canny(mask,50,150,apertureSize = 3) #find edges with canny
    lines = cv2.HoughLines(edges,1,np.pi/180,model.line_detection_threshold - 100)

    #checking for best line:
    #line with higest accumulator value is first in the list of lines[]
    if lines is not None:
        line_detected = True
        
        # Get rho and theta from the one most prominent line
        rho = lines[0][0][0]
        theta = lines[0][0][1]
        angle = np.degrees(abs(theta))

        # correct for weird angle issue
        if angle > 180-model.max_angle:
            angle = (angle - 180)

        if (abs(angle) < model.max_angle):
            out_angle = angle
        
        # Now find the distance of the line from the left edge of frame
        if theta == 0:
            out_distance =  abs(rho)
        if model.y_off == 0:
            out_distance =  rho/np.cos(theta)
        if rho > 0:
            out_distance =  (rho/np.cos(theta)-model.y_off*np.tan(theta))
        if rho < 0:
            out_distance =  (rho/np.cos(theta)+model.y_off*np.tan(theta))
        if rho == 0:
            if theta > np.pi/2:
                out_distance =  (-model.y_off*np.tan(theta))
            if theta < np.pi/2:
                out_distance =  (model.y_off/np.tan(theta))
        else:
            return None

    return [line_detected, out_distance, out_angle]

if __name__ == "__main__":
    check_for_line(cap,model)
    start_camera_feed(camera_id)