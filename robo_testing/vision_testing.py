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

    # start camera feed
    #cap = rv.start_camera_feed(CAMERA_ID)

    model.check_for_hand()
    if not model.hand_detected: 
        model.start()

    model.set_detected_theta(-np.pi/4)
    model.set_blade_angle(model.blade_angle)

    display.disp_final(model, CAMERA_ID)
    #await asyncio.gather(display.disp_img_processing(model, CAMERA_ID), display.disp_edge_detection(model, CAMERA_ID))
    
    #display.disp_img_processing(model, cap)
    #display.disp_edge_detection(model, CAMERA_ID)

    cap = cv2.VideoCapture(CAMERA_ID)
    if not cap.isOpened():
        raise IOError("Cannot open webcam")

    while True:
        line = rv.check_for_line(model, cap)
        if(line[0]):
            print("Distance: [" + str(line[1]) + "] pixels")
            #print("Angle: [" + str(line[2]) + "] degrees")
        c = cv2.waitKey(1)
        if c == 27:
            break
            cap.release()
            cv2.destroyAllWindows()


if __name__ == "__main__":
    import time
    s = time.perf_counter()
    #asyncio.run(main())
    main()
    elapsed = time.perf_counter() - s
    print(f"{__file__} executed in {elapsed:0.2f} seconds.")