from Model import Model
import robo_vision as rv
import display
import numpy as np
import cv2

MAX_ANGLE = 52
Y_OFFSET = 50 # pixels from top of frame: negative -> above the border | positive -> below the border
X_OFFSET = 500 # pixels from left edge of frame: negative -> left of the edge | positive -> to the right of the edge

initial_angle = -45 # angle of the blade at startup: get this from Dylan
model = Model(initial_angle,MAX_ANGLE,Y_OFFSET,X_OFFSET)

#display.disp_current_view() # Displays camera feed
model.check_for_hand()
if not model.hand_detected: 
    model.start()
model.set_detected_rho(100)
model.set_detected_theta(-np.pi/4)
model.set_blade_angle(model.blade_angle)
rv.disp_img_processing()
#display.disp_model(model)

