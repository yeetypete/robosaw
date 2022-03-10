import cv2
import numpy as np
import time
import sys

################################################
color_cam_id = 4
angle_cam_id = 0
center_cam_id = 2
thearray = [color_cam_id, angle_cam_id, center_cam_id]

print(thearray)
key = input()
if key == 27:
    print("Exiting without saving...")
    sys.exit()
if key == 's':
    thearray = [color_cam_id, angle_cam_id, center_cam_id] 
    # Also save this array as .npy
    np.save('__calibrate__/color_angle_center_cam_id_array',thearray)
    print(str(thearray) + ": Saved")
    sys.exit()
################################################


def display_by_index(cam_id):
    name = "Camera ID: " + str(cam_id)
    cap = cv2.VideoCapture(cam_id)
    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    print("\nPress 'q' to close display window")

    while True:
        ret,frame = cap.read()
        cv2.imshow(name, frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            break;
    time.sleep(1)

print("How many cameras are connected?: ")
#count = int(input()) # Use this for windows
count = 2*int(input()) # Use this for pi

for cam_id in range(count,2): # Use this for pi
#for cam_id in range(count): # Use this for windows
    print("Opening camera "+ str(cam_id)+"...")
    display_by_index(cam_id)

print("Assign each camera the correct index.\n")
print("Enter the index ID for color_cam: ")
color_cam_id = int(input())
print("The index for color_cam is " + str(color_cam_id))


print("\nEnter the index ID for angle_cam: ")
angle_cam_id = int(input())
print("The index for angle_cam is " + str(angle_cam_id))


print("\nEnter the index ID for center_cam: ")
center_cam_id = int(input())
print("The index for center_cam is " + str(center_cam_id))

print("\nPress the 's' key to save and apply these settings: ")

thearray = [color_cam_id, angle_cam_id, center_cam_id]

print(thearray)
key = input()
if key == 27:
    print("Exiting without saving...")
    sys.exit()
if key == 's':
    thearray = [color_cam_id, angle_cam_id, center_cam_id] 
    # Also save this array as .npy
    np.save('__calibrate__/color_angle_center_cam_id_array',thearray)
    print(str(thearray) + ": Saved")
    sys.exit()