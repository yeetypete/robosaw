<p align="center" width="100%">
    <img width="50%" src="./readme_media/logo.png">
</p>

# **RoboSaw Software Guide**

## **Calibration Scripts**

All these scripts are used to calibrate certain parts of the RoboSaw's
vision system. The outputs are saved in a local \_\_calibrate\_\_ folder
as .npy flies. This allows the calibration to be persistent after
turning on and off the RoboSaw. To calibrate, please navigate to the
RoboSaw folder in the terminal. If your RoboSaw has desktop applications
already set up, then there is no need to run these from the terminal.
However, running these will always work from the terminal.

IMPORTANT: If calibrating for the first time, run set_cam_index.py
before any of the other calibration scripts. Otherwise the scripts will
not know which cameras to use.

The RoboSaw comes fully calibrated. However, there may be times when it
needs to be recalibrated. Calibration issues could occur if a camera is
moved. 

The calibration process is a bit involved. It is strongly recommended
for this process to be performed by a RoboSaw technician. 

1.  Plug in a usb mouse and keyboard to the usb type-A ports on the
    RoboSaw.

2.  Connect a monitor using the HDMI port.

3.  Turn on the RoboSaw.

4.  You should see the Raspberry Pi boot up on the screen.

5.  Open the terminal and enter `cd /pi/robosaw`

6.  This directory is where the calibration scripts are located. Run
    these in this order to recalibrate the RoboVision system.

    a.  Run `python3 set_cam_index.py` first to tell RoboVision which
         camera is used for each task. The color-cam is the one facing
         the fence, the angle-cam is the one that is closest to the
         intake side of the saw, and the center-cam is the one attached
         to the blade guard.

> Follow the prompts to save the camera indexes.

b.  Run `python3 tune_color_key_top.py` to isolate the green color. Adjust
     the sliders until only the green is visible.

c.  Run `python3 tune_color_key_bottom.py`  to isolate the green color.
     Adjust the sliders until only the green is visible.

> Tuning the color keys:
> Default view ![tune_color_zeroed](https://user-images.githubusercontent.com/55928366/165641691-36a1e34b-166a-4378-a07f-d77d559c408e.png)
> Next adjust the Hue to isolate the green pixels as well as possible ![tune_color_H_set](https://user-images.githubusercontent.com/55928366/165641937-a7e04ee1-fa39-446e-9a7a-69f4d28be160.png)
> Finally adjust the Saturation and Brightness to cut out extranious pixels ![tune_color_set_all](https://user-images.githubusercontent.com/55928366/165642008-b26f4294-e927-4a1b-87e8-1a1ad6900818.png)


d.  Run `python3 crop_angle_cam.py` to crop the area viewed by the angle
     detector. Make sure that the camera only sees the wood when the
     wood is loaded.

> Cropping the angle camera: 
> Default view with no wood loaded ![crop_angle_cam_no_wood](https://user-images.githubusercontent.com/55928366/165642387-3bf7e3cd-598b-444d-aa97-264a7b09e994.png)
> Next load a 2x4 under the canera and make sure it is against the fence ![crop_angle_cam_wood_not_cropped](https://user-images.githubusercontent.com/55928366/165642485-0b5c48ff-c43f-46d7-804a-257504b0155c.png)
> Finally adjust the **Top** and **Bottom** sliders to cut out any background around the wood. In rare cases it is possible that the roller is in view on the left side. If so, crop that out with the **Left** slider ![crop_angle_cam_wood_cropped](https://user-images.githubusercontent.com/55928366/165642715-33e845a5-1fbb-47a4-b0c4-798b6be57283.png)



e.  Run `python3 crop_center_cam.py` to crop the area viewed by the center
     camera which positions the wood under the blade. Make sure the
     centerline is in the middle of the blade slot. As a rule of thumb,
     try to make the cropped circle the same radius as the circular
     section of the table.

> Cropping and aligning the center camera: ![crop_center](https://user-images.githubusercontent.com/55928366/165642981-54499a2c-464a-47d3-882c-3f51c775c87f.png)


f.  Run `python3 crop_color_cam_bottom.py` to isolate a small area on the
     green fence which, when blocked by a piece of wood, will tell the
     RoboVision program that the wood is past the angle detector. 

g.  Run `python3 crop_color_cam_top.py` to isolate a small area on the
     green fence which, when blocked by a piece of wood, will tell the
     RoboVision program that the wood is under the blade. It is possible to accidentally crop the image in a direction that is not allowable. If this happens the ROI will turn red and you will not be able to save the selection. It works best to make this ROI in a way that it captured both angled sides of the fence in the center.

> Cropping the color camera regions of interest: 
> Valid selection ![crop_color_region_valid](https://user-images.githubusercontent.com/55928366/165643034-238fe004-b55f-4405-a3f1-a043612bc343.png)
> Invalid selection ![crop_color_region_invalid](https://user-images.githubusercontent.com/55928366/165643078-d6696757-dcac-4f77-94a5-24bdb098dbb3.png)



h.  Run `python3 tune_canny.py` to tune the edge detection. Place a sample
     piece of wood with a line on it under the center camera. Adjust
     the sliders until the desired line is visible. It is ok if the
     detector picks up other edges and wood grains as these will be
     ignored by the RoboSaw. The most important thing is to adjust it
     so that your line is clearly visible in the preview. Try to stick
     to similar lighting conditions and darkness of wood for each
     calibration.

> Tuning the edge detection: 
> Default values ![tune_canny_zeroed](https://user-images.githubusercontent.com/55928366/165643245-6eeb7756-28f1-46ad-b1d0-098e5bab01cb.png)
> Marker only ![tune_canny_sharpie](https://user-images.githubusercontent.com/55928366/165643284-4c07346b-4b27-4028-8e60-6eca2f58a80a.png)
> Pencil and marker ![tune_canny_pencil_and_sharpie](https://user-images.githubusercontent.com/55928366/165643342-0a93542c-3527-4ece-a048-dbe6dad3c36d.png)
> Darker wood ![tune_canny_darker_wood](https://user-images.githubusercontent.com/55928366/165643389-731fc2e2-deaf-4842-b45a-509aa17fa6b8.png)



# **Model.py**

This is a class which stores all the variables related to the current
state of the RoboSaw. As sensor readings are processed, an instance of
Model called 'model' gets updated to reflect the current state of the
system. Only one instance of 'model' is created globally which allows
processes on stearate threads to pull data from the model. The Model
class also defines many functions defining the mathematical operations
used in robo_vision2 and elsewhere throughout the program. Although
these functions aren't used to store information about the RoboSaw's
state, it was a design choice to include them here for convenience.

## class Model(object)

### \_\_init\_\_(self, MAX_ANGLE)

This is the constructor. It initializes the maximum angle that the saw
is able to cut. In the case of the saw, we used it is set to 52 degrees.

### best_angle(self, arr)

Returns the trimmed mean of an array of angle samples. When the line
first passes under the angle detection camera, each sample is
accumulated into an array until the array is the same size as the
desired number of samples. It then trims all datapoints outside of two
standard deviations from the mean and then takes the mean of that
trimmed dataset. This is done to eliminate outliers if there are any.

### img_proc(self,frame)

Given an image frame it returns a processed version of that frame of the
same size as the input frame. Here it is used to convert to grayscale,
add cv2.GaussianBlur to reduce high frequency noise from the image, then
finally apply a cv2.Canny edge detection. The returned frame is a binary
edge map showing all the detected edges present in the input frame.

### img_proc_angle_detect(self,frame)

Given binary edge map returned from img_proc(frame) it applies a Hough
transform to detect lines from the edges. Each line is represented by a
rho & theta pair. This has been optimized to detect lines from the
camera which detects angles.

### img_proc_line_detect_center(self,frame)

Given binary edge map returned from img_proc(frame) it applies a Hough
transform to detect lines from the edges. Each line is represented by a
rho & theta pair. This has been optimized to detect lines from the
center camera which detects a lines distance from the center of the
blade.

### get_best_line(self,lines)

Given an accumulator array created by running a Hough transform on an
edge map, this trims all the lines which are outside the maximum cutting
angle of the saw. This is to reduce the likelihood of a false detection
since no line intended to be cut will be outside of this range. This
eliminates a lot of the lines captured by strongly colored grains in the
wood. Then from the remaining lines, the one with the highest
accumulator value wins and is selected as the most likely line.

### get_saw_angle(self,line)

Given a line, returns the angle which the saw must rotate to in order to
make the cut. The angle is first converted to degrees and then it is
signed to show if the angle is to the left or right of zero degrees.

### find_dist_from_center(self, line)

Given a line detected by the center camera, returns the distance of that
line from the center of the blade. The distance is in pixels. Positive
values mean the line is to the right of the blade and negative values
mean it is to the left of the blade. A zero value means the line is
directly under the blade.

*Uses numpy and cv2 libraries.*

# **Robo_vision2.py**

This is the main processing module used for computer vision in this
project. This is the second version. Robo_vision was originally only
designed to use one camera, but an updated version allowed for much more
robust image processing. Therefore, we rebuilt robo_vision in favor of
robo_vision2.

Here is a breakdown of the functions defined within this module:

### open_cameras(model)

This opens all three cameras and returns an array of the captures.

### find_angle(model,cap)

This uses functions from Model to pull a frame from the angle camera,
detect lines, pick the best line, and then find the trimmed-mean best
angle sample from the array of angle samples. Angle returned is in
degrees.

### find_distance(model,cap)

This uses functions from Model to pull a frame from the center camera,
detect lines, determine the best line, and then find the distance from
the blade in pixels.

### wood_is_4x4(model,cap)

This uses functions from Model to pull a frame from the color camera,
threshold the frame based on the calibrations in Model, and then count
the number of pixels detected in the region of interest. When a piece of
wood is tall enough to cover this region, all pixels turn off and it is
assumed that the wood is 4 inches tall rather than 2 inches. Returns
true if the wood is 4x4 and false if it is not.

### wood_is_2x6(model,cap)

This uses functions from Model to pull a frame from the color camera,
threshold the frame based on the calibrations in Model, and then count
the number of pixels detected in the region of interest. When a piece of
wood is wide enough to cover this region, all pixels turn off and it is
assumed that the wood is 6 inches wide rather than 4 inches. Returns
true if the wood is 2x6 and false if it is not.

### wood_is_loaded(model,cap)

This uses functions from Model to pull a frame from the color camera,
threshold the frame based on the calibrations in Model, and then count
the number of pixels detected in the region of interest. This will
return true if something is covering the intake fence over the entire
region of interest.

### wood_is_under(model,cap)

This uses functions from Model to pull a frame from the color camera,
threshold the frame based on the calibrations in Model, and then count
the number of pixels detected in the region of interest. This will
return true if something is covering the center fence over the entire
region of interest.

*Uses cv2, numpy, Model, and time.*

# **RoboSaw.py**

### class DriverFault(Exception)

Defines a custom exception to raise if a fault is detected.

## class Actuator(object)

Defines pins and enables drivers to run the linear actuator.

-   \_\_init\_\_(self, pwm_pin, dir_pin, en_pin, flt_pin, \_pi)
    Constructor.

-   setSpeed(self, speed) Sets the speed of the actuator. Range from
    -480 -- 480.

-   enable(self) Enables the actuator.

-   disable(self) Disables the actuator.

-   getFault(self) Error handling.

-   forceStop(self) Stops the actuator.

### init_gpio()

Pigpio initialization for GPIO pins

### init_args()

Argument parser for testing

### feed(distance, speed)

Feeds the wood by a given distance.

Uses print_function from \_\_future\_\_, Model, pigpio, argparse, and
dual_g2_hpmd_rpi.

# **ButtonHandler.py**

## class ButtonHandler(threading.Thread)

Listens to each button. This handles button debouncing. In order to
properly debounce the buttons the callback function must return quickly.
Therefore, the callback returns quickly after starting the desired
process in a new thread.

*Uses RPi.GPIO and threading.*

# **Main**

This is where the main function of the program lives. Functions defined
in Main are:

### initialize()

This initializes an instance of Model using the desired maximum angle.
Returns that instance of Model called 'model'.

### close_caps(caps)

Given an array of camera captures; this closes all the captures in the
array. Used after each process is done using the cameras.

### eject()

Ejects the wood from the RoboSaw.

### stop(secs)

Stops the wood from feeding for given amount of seconds.

### raise_blade()

Raises the blade. Used in case the blade is stuck down after an
emergency stop was used.

### run(model)

This is the process that intakes the wood and centers the line under the
blade. First it starts the intake wheels and waits for wood to be
loaded. Once it sees a piece of wood is loaded it starts to look for a
line and grabs the angle of that line. Then it tells the RoboSaw to
rotate the blade to the correct angle and awaits the users confirmation
that this is a valid line to cut. If the user presses "SKIP" the saw
will not cut that line and will continue to the next line. If the user
selects "RUN" the RoboSaw will center that line under the blade. To
center the line quickly and precisely under the blade we use a PID loop.
This PID loop runs continuously until the user presses the "CUT" button
or the "SKIP" button. Continuously running the PID loop up until the
last second allows the wood to recover in the case that it gets bumped
out of place by the user or other external events.

### cut(model)

This is the process that cuts the wood when the user presses the "CUT"
button on the pendant. It checks first that the model is ready to make a
cut. If a cut is not ready to be made the function simply returns. It
then does a visual check to see if the wood is loaded properly and is
under the blade. If the wood is in place, it turns on the blade, lowers
the blade, then raises it and stops the blade. Finally, it closes the
camera captures and returns.
<br>*Uses Model, robo_vision2, RoboSaw, ButtonHandler, time, cv2, sys, signal, pigpio, RPi.GPIO, numpy, simple_pid, and matplotlib.pyplot.*
<br>
# Dependencies
![Untitled Diagram](https://user-images.githubusercontent.com/55928366/165645983-85dbdfad-5c0d-4d2c-a9f0-030f3f8a3b02.jpg)


