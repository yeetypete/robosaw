import numpy as np

class Model(object):
    """RoboSaw environment model"""
    detected_rho = None
    detected_theta = None
    dist_from_blade = None
    blade_theta = None
    crop_x1 = None
    # constructor
    def __init__(self,initial_angle,MAX_ANGLE,Y_OFFSET,X_OFFSET):
        self.blade_angle = initial_angle
        self.blade_theta = np.radians(self.blade_angle)
        self.blade_init_angle = initial_angle
        self.max_angle = MAX_ANGLE
        self.y_off = Y_OFFSET
        self.x_off = X_OFFSET

    
    # methods
    def set_blade_angle(self, angle):
        if self.angle_is_valid(angle):
            self.blade_angle = angle
            self.blade_theta = np.radians(angle)

    def update(self):
        self.blade_theta = np.radians(self.blade_angle)

    def greet(self):
        print("Model created")

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

    def find_offset(self):
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
            return 0

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


