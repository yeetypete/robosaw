import numpy as np
import cv2
class Model(object):
    """RoboSaw environment model"""
    detected_rho = None
    detected_theta = None
    dist_from_blade = None
    blade_theta = None
    crop_x1 = None
    line_detection_threshold = 80 #100
    line_detect_camera_id = 0
    min_angle_diff = 1
    min_dist_diff = 10


    ###########################
    camera_id = 1
    ###########################


    #colorspace threshold for edge detection
    #green_h = 90
    #green_buff = 3
    # red isolation [[0, 104, 179], [179, 255, 255]]
    # for green background [[69, 33, 57], [98, 211, 218]] __calibrate__/hsv_value.npy
    hsv = np.load('__calibrate__/hsv_value.npy')
    h_lower_thresh = hsv[0][0]
    h_upper_thresh = hsv[1][0]
    s_lower_thresh = hsv[0][1]
    s_upper_thresh = hsv[1][1]
    v_lower_thresh = hsv[0][2]
    v_upper_thresh = hsv[1][2]

    # cropping values
    # 4x4 use [[19, 399], [82, 596]]
    crop_vals = np.load('__calibrate__/top_bottom_left_right.npy')
    top = crop_vals[0][0]
    bottom = crop_vals[0][1]
    left = crop_vals[1][0]
    right = crop_vals[1][1]

    # constructor
    def __init__(self,initial_angle,MAX_ANGLE,Y_OFFSET,X_OFFSET,DETECTION_THRESHOLD):
        self.blade_angle = initial_angle
        self.blade_theta = np.radians(self.blade_angle)
        self.blade_init_angle = initial_angle
        self.max_angle = MAX_ANGLE
        self.y_off = Y_OFFSET
        self.x_off = X_OFFSET
        self.line_detection_threshold = DETECTION_THRESHOLD

    
    # methods
    def img_proc_line_detect(self,frame):
        #image processing:
        grey = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        #grey = cv2.GaussianBlur(grey,(5,5),cv2.BORDER_DEFAULT)
        edges = cv2.Canny(grey,50,150,apertureSize = 3)
        lines = cv2.HoughLines(edges,1,np.pi/180,self.line_detection_threshold)
        return lines

    def set_line_detect_camera_id(self, cam_id):
        self.line_detect_camera_id = cam_id

    def set_line_detection_threshold(self, thresh):
        self.line_detection_threshold = thresh

    def set_blade_angle(self, angle):
        if self.angle_is_valid(angle):
            self.blade_angle = angle
            self.blade_theta = np.radians(angle)

    def update(self):
        self.blade_theta = np.radians(self.blade_angle)

    def greet(self):
        print("Starting...")

    def start(self):
        self.greet()

    def angle_is_valid(self, angle):
        """ returns true iff the angle is in bounds """ 
        if np.radians(angle) < np.radians(self.max_angle):
            #print("Angle is valid")
            return True
        if self.detected_theta < np.radians(self.max_angle):
            #print("Angle is valid")
            return True
        if self.blade_theta < np.radians(self.max_angle):
            #print("Angle is valid")
            return True
        else:
            #print("Angle is NOT valid")
            return False

    def theta_is_valid(self, theta):
        """ returns true iff the angle is in bounds """ 
        if abs(theta) < abs(np.radians(self.max_angle)):
            #print("Angle is valid")
            return True
        else:
            #print("Angle is NOT valid")
            return False

    def find_angle(self):
        """ returns angle in degrees that the saw should move to
        -- positive angle -> rotate blade counter-clockwise
        -- negative angle -> rotate blade clockwise
        -- zero angle -> center the blade for a cross-cut """

        #print("Degrees: " + str(-(np.degrees((rho*theta)/abs(rho)))))
        return -(np.degrees((self.detected_rho*self.detected_theta)/abs(self.detected_rho)))

    def offset_from_center(self): # TODO
        return None

    def find_x_offset_of_detected_line(self):
        """ finds distance from offset point and returns the x_offset_distance
        -- the offset poit is adjustable to calibrate the saw
        -- the offset point is the pivot point of the saw's turntable
        -- returns how far the line intersection point is from the offset point in pixels """ 

        if self.detected_theta == 0:
            return abs(self.detected_rho)
        if self.y_off == 0:
            return self.detected_rho/np.cos(self.detected_theta)
        if self.detected_rho > 0:
            return (self.detected_rho/np.cos(self.detected_theta)-self.y_off*np.tan(self.detected_theta))
        if self.detected_rho < 0:
            return (self.detected_rho/np.cos(self.detected_theta)+self.y_off*np.tan(self.detected_theta))
        if self.detected_rho == 0:
            if self.detected_theta > np.pi/2:
                return (-self.y_off*np.tan(self.detected_theta))
            if self.detected_theta < np.pi/2:
                return (self.y_off/np.tan(self.detected_theta))
        else:
            return None

    def set_detected_rho(self,rho):
        """ Use this to set detected rho manually """
        self.detected_rho = rho

    def set_detected_theta(self,theta):
        """ Use this to set detected theta manually """
        self.detected_theta = theta

    def print_blade_angle(self):
        """ prints the blade angle to console """
        print(self.blade_angle)

    def check_for_hand(self):
        """ returns true if there is a hand present in the danger zone """
        # TODO: add a check that sets self.hand_detected == True or False
        #   using a hand tracking/ detecting algorithm
        self.hand_detected = False

        if self.hand_detected == True:
            print("Hand Detected")
            return True
        else:
            print("No Hand Detected")
            return False # TODO


