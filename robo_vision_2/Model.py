import numpy as np
import cv2
class Model(object):

    """ RoboSaw environment model """

    #### Functions ####
    def __init__(self, MAX_ANGLE):
        self.max_angle = MAX_ANGLE
        print("RoboSaw initializing Model")

    def img_proc_line_detect(self,frame):
        """ Image processing """
        #grey = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        #grey = cv2.GaussianBlur(grey,(5,5),cv2.BORDER_DEFAULT)
        filter = np.array([[0,0,-1,0,0],[0,-1,-2,-1,0],[-1,-2,16,-2,-1],[0,-1,-2,-1,0],[0,0,-1,0,0]])
        filter_frame=cv2.filter2D(frame,-1,filter)
        edges = cv2.Canny(filter_frame,50,150,apertureSize = 3)
        lines = cv2.HoughLines(edges,1,np.pi/180,self.line_detection_threshold)
        return lines

    def get_best_line(self,lines):
        """ Returns line = [rho,theta] of best line detected """
        #line with higest accumulator value is first in the list of lines[]
        if lines is not None:

            # Start with highest accumulator value and iterate until 
            # one is found within the max_angle range
            for i in range(0,len(lines)):
                rho = lines[i][0][0]
                theta = lines[i][0][1]
                angle = np.degrees(abs(theta))

                # correct for weird angle issue
                if angle > 180-self.max_angle:
                    angle = (angle - 180)

                # Ifthe line is in range return
                if (abs(angle) < self.max_angle):
                    return [rho,theta]

            # If no line is found return None
            return [None,None]
                
       
    def get_saw_angle(self,line):
        """ Given line = [rho,theta] returns the angle that the saw must rotate to """
        if line is None:
            return None
        rho = line[0]
        theta = line[1]
        out_angle = None

        if ((rho is not None) and (theta is not None)):
            angle = np.degrees(abs(theta))
            # correct for angle geometry
            if angle > 180-self.max_angle:
                angle = (angle - 180)
            if (abs(angle) < self.max_angle):
                out_angle = angle
            return out_angle
        else:
            return None

    def show_line(self,frame,line):
        rho = line[0]
        theta = line[1]
        angle = np.degrees(abs(theta))
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a * rho
        y0 = b * rho
        pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
        pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
        cv2.line(frame,pt1,pt2,(255,100,200),5, cv2.LINE_AA)

        # format text to show angle
        font                   = cv2.FONT_HERSHEY_SIMPLEX
        bottomLeftCornerOfText = (10,200)
        fontScale              = 1
        fontColor              = (255,255,255)
        thickness              = 3
        lineType               = 2

        cv2.putText(frame,'Angle: '+str(angle), 
        bottomLeftCornerOfText, 
        font, 
        fontScale,
        fontColor,
        thickness,
        lineType)

        return frame
        
    #### Model constants ####
    line_detection_threshold = 40 #100

    # Set the camera indexes
    cam_id_array = np.load('__calibrate__/color_angle_center_cam_id_array.npy')

    color_cam_id = cam_id_array[0]
    center_cam_id = cam_id_array[1]
    angle_cam_id = cam_id_array[2]

    # HSV colorspace threshold for wood edge detection
    hsv = np.load('__calibrate__/hsv_value.npy')

    h_lower_thresh = hsv[0][0]
    h_upper_thresh = hsv[1][0]
    s_lower_thresh = hsv[0][1]
    s_upper_thresh = hsv[1][1]
    v_lower_thresh = hsv[0][2]
    v_upper_thresh = hsv[1][2]

    # Crop color_cam
    crop_vals_color_cam = np.load('__calibrate__/color_cam_top_bottom_left_right.npy')

    top_color_cam = crop_vals_color_cam[0][0]
    bottom_color_cam = crop_vals_color_cam[0][1]
    left_color_cam = crop_vals_color_cam[1][0]
    right_color_cam = crop_vals_color_cam[1][1]

    # Crop angle_cam
    crop_vals_angle_cam = np.load('__calibrate__/angle_cam_top_bottom_left_right.npy')

    top_angle_cam = crop_vals_angle_cam[0][0]
    bottom_angle_cam = crop_vals_angle_cam[0][1]
    left_angle_cam = crop_vals_angle_cam[1][0]
    right_angle_cam = crop_vals_angle_cam[1][1]

    # Crop center_cam
    crop_vals_center_cam = np.load('__calibrate__/center_cam_top_bottom_left_right.npy')

    top_center_cam = crop_vals_center_cam[0][0]
    bottom_center_cam = crop_vals_center_cam[0][1]
    left_center_cam = crop_vals_center_cam[1][0]
    right_center_cam = crop_vals_center_cam[1][1]

    pass




