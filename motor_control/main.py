from Model import Model
import robo_vision as rv
import display
import numpy as np
import cv2

MAX_ANGLE = 52 # maximum angle that the blade can rotate for a miter cut
CAMERA_ID = 1 # change this depending on which camera to use, default to zero
Y_OFFSET = 50 # pixels from top of frame: negative -> above the border | positive -> below the border
X_OFFSET = 500 # pixels from left edge of frame: negative -> left of the edge | positive -> to the right of the edge
DETECTION_THRESHOLD = 180 # threshold for line detection -> minimum accumulator value

initial_angle = -45 # angle of the blade at startup: get this from Dylan




if __name__ == "__main__":
	# generate a model based on initial conditions
	model = Model(initial_angle,MAX_ANGLE,Y_OFFSET,X_OFFSET,DETECTION_THRESHOLD)

	# start camera feed
	cap = rv.start_camera_feed(CAMERA_ID)

	model.check_for_hand()
	if not model.hand_detected: 
    	model.start()

	model.set_detected_theta(-np.pi/4)
	model.set_blade_angle(model.blade_angle)
	#display.disp_img_processing(model, CAMERA_ID)
	display.disp_edge_detection(model, CAMERA_ID)
	line = rv.check_for_line(cap,model)

