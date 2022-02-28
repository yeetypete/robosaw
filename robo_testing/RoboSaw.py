from Model import Model
import robo_vision as rv
import display
import numpy as np
import cv2
import asyncio

def main():
    MAX_ANGLE = 10 # maximum angle that the blade can rotate for a miter cut
    CAMERA_ID = 0 # change this depending on which camera to use, default to zero, 1 for external usb camera
    Y_OFFSET = 50 # pixels from top of frame: negative -> above the border | positive -> below the border
    X_OFFSET = 500 # pixels from left edge of frame: negative -> left of the edge | positive -> to the right of the edge
    LINE_DETECTION_THRESHOLD = 50 # threshold for line detection -> minimum accumulator value for Hough Lines algo.

    initial_angle = -45 # angle of the blade at startup: get this from Dylan

    # generate a model based on initial conditions
    model = Model(initial_angle,MAX_ANGLE,Y_OFFSET,X_OFFSET,LINE_DETECTION_THRESHOLD)

    model.check_for_hand()
    if not model.hand_detected: 
        model.start()

    model.set_detected_theta(-np.pi/4)
    model.set_blade_angle(model.blade_angle)

    # open a camera feed
    cap = cv2.VideoCapture(CAMERA_ID)
    if not cap.isOpened():
        raise IOError("Cannot open webcam")

    while True:
        line = rv.check_for_line(model, cap)
        if(line[0]):
            print("Distance: [" + str(line[1]) + "] pixels")


if __name__ == "__main__":
    import time
    s = time.perf_counter()
    main()
    elapsed = time.perf_counter() - s
    print(f"{__file__} executed in {elapsed:0.2f} seconds.")