import cv2
import numpy as np
from Model import Model
import time

def open_cameras(model):
    """ Initializes the three camrea feeds
            returns array of captures: [color, angle, center]"""
    print("Warming up the cameras...")
    s = time.perf_counter()

    color_cap = cv2.VideoCapture(model.color_cam_id)
    angle_cap = cv2.VideoCapture(model.angle_cam_id)
    center_cap = cv2.VideoCapture(model.center_cam_id)
    if not color_cap.isOpened():
        print("Cannot open color camera")
        exit()
    if not angle_cap.isOpened():
        print("Cannot open angle camera")
        exit()
    if not center_cap.isOpened():
        print("Cannot open center camera")
        exit()

    elapsed = time.perf_counter() - s
    print(f"All cameras opened in {elapsed:0.3f} seconds.")
    
    return [color_cap, angle_cap, center_cap]

def display_cap(model, cap):
    print("Displaying cap #" +str(cap) + "\nPress 'q' to close")
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
 
        # Display the resulting frame
        cv2.imshow("Press 'q' to quit", frame)
        if cv2.waitKey(1) == ord('q'):
            break
    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

def find_angle_display(model,cap):
    while True:
        # get frame
        ret , frame = cap.read()
        if not ret:
            print("No frame captured: ret is False")
        # crop
        #frame = frame[model.top_angle_cam:model.bottom_angle_cam, model.left_angle_cam:model.right_angle_cam]

        line_detected = False
        out_angle = None

        lines = model.img_proc_line_detect(frame)
        line = model.get_best_line(lines)
        angle = model.get_saw_angle(line)

        if angle is not None:
            frame = model.show_line(frame,line)

        print("Angle: " + str(angle))

        cv2.imshow('RoboVision: press "q" to quit', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            break

