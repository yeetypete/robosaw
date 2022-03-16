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

def display_color_cap(model, cap):
    print("Displaying cap #" +str(cap) + "\nPress 'q' to close")
    time.sleep(1)
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
 
        # Display the resulting frame
        frame = frame[model.top_color_cam:model.bottom_color_cam, model.left_color_cam:model.right_color_cam]
        cv2.imshow("Press 'q' to quit", frame)
        if cv2.waitKey(1) == ord('q'):
            break
    # When everything done, release the capture
    #cap.release()
    cv2.destroyAllWindows()

def display_center_cap(model, cap):
    print("Displaying cap #" +str(cap) + "\nPress 'q' to close")
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        # Crop circle
        frame = model.crop_circle(frame)
        # Display the resulting frame
        cv2.imshow("Press 'q' to quit", frame)
        if cv2.waitKey(1) == ord('q'):
            break
    # When everything done, release the capture
    #cap.release()
    cv2.destroyAllWindows()

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
    #cap.release()
    cv2.destroyAllWindows()

def find_angle_display(model,cap):
    while True:
        # get frame
        ret , frame = cap.read()
        if not ret:
            print("No frame captured: ret is False")
        # crop
        #frame = frame[model.top_angle_cam:model.bottom_angle_cam, model.left_angle_cam:model.right_angle_cam]

        lines = model.img_proc_angle_detect(frame)
        line = model.get_best_line(lines)
        
        if line is not None:
            frame = model.add_line_angle(frame,line)


        cv2.imshow('RoboVision: press "q" to quit', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            #cap.release()
            cv2.destroyAllWindows()
            break

def find_center_display(model,cap):
    while True:
        # get frame
        ret , frame = cap.read()
        if not ret:
            print("No frame captured: ret is False")
        # crop
        frame = model.crop_circle(frame)

        # find line
        lines = model.img_proc_line_detect_center(frame)
        line = model.get_best_center_line(lines)
        
        if line is not None:
            frame = model.add_line_distance(frame,line)
            

        cv2.imshow('RoboVision: press "q" to quit', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            #cap.release()
            cv2.destroyAllWindows()
            break

def find_angle(model,cap):
    """ If an angle is detected returns the angle that the saw should rotate to 
    Else returns None """
    ret , frame = cap.read()
    if not ret:
            print("No frame captured: ret is False")
            return None
    lines = model.img_proc_angle_detect(frame)
    line = model.get_best_line(lines)
    angle = model.get_saw_angle(line)
    cv2.destroyAllWindows()
    return angle

def find_distance(model,cap):
    """ If edge is detected return its distance from the blade
    Else return None """
    ret , frame = cap.read()
    if not ret:
            print("No frame captured: ret is False")
            return None
    lines = model.img_proc_line_detect_center(frame)
    line = model.get_best_center_line(lines)
    distance = model.find_dist_from_center(line)
    cv2.destroyAllWindows()
    return distance

def img_proc_display(model,cap):
    while True:
        # get frame
        ret , frame = cap.read()
        if not ret:
            print("No frame captured: ret is False")
            return None
        # crop
        #frame = frame[model.top_angle_cam:model.bottom_angle_cam, model.left_angle_cam:model.right_angle_cam]

        frame = model.img_proc(frame)

        cv2.imshow('RoboVision: press "q" to quit', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            #cap.release()
            cv2.destroyAllWindows()
            break

def wood_is_loaded(model,cap):
    ret , frame = cap.read()
    if not ret:
            print("No frame captured: ret is False")
            return False
    frame1 = frame[model.top_color_cam1:model.bottom_color_cam1, model.left_color_cam1:model.right_color_cam1]
    hsv1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2HSV) #color space transformation to hsv
    lower_green = np.array([model.h_lower_thresh1,model.s_lower_thresh1,model.v_lower_thresh1])
    upper_green = np.array([model.h_upper_thresh1,model.s_upper_thresh1,model.v_upper_thresh1])
    mask1 = cv2.inRange(hsv1, lower_green, upper_green) #threshold the image to only show green pixels
    number_of_white_pix1 = np.sum(mask1 == 255)
    if number_of_white_pix1 < model.color_thresh_wood_detection:
        print("Wood is loaded")
        return True
    else:
        print("Waiting for you to give me the wood...")
        return False