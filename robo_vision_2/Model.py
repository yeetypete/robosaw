import numpy as np
class Model(object):
    """RoboSaw environment model"""
    line_detection_threshold = 80 #100

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




