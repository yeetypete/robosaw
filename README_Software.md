**RoboSaw Software Guide**

**Model**

This is a class which stores all the variables related to the current
state of the RoboSaw. As sensor readings are processed, an instance of
Model called 'model' gets updated to reflect the current state of the
system. Only one instance of 'model' is created globally which allows
processes on stearate threads to pull data from the model. The Model
class also defines many functions defining the mathematical operations
used in robo_vision2 and elsewhere throughout the program. Although
these functions aren't used to store information about the RoboSaw's
state, it was a design choice to include them here for convenience.

\_\_init\_\_(self, MAX_ANGLE) This is the constructor. It initializes
the maximum angle that the saw is able to cut. In the case of the saw,
we used it is set to 52 degrees.

best_angle(self, arr) Returns the trimmed mean of an array of angle
samples. When the line first passes under the angle detection camera,
each sample is accumulated into an array until the array is the same
size as the desired number of samples. It then trims all datapoints
outside of two standard deviations from the mean and then takes the mean
of that trimmed dataset. This is done to eliminate outliers if there are
any.

img_proc(self,frame) Given an image frame it returns a processed version
of that frame of the same size as the input frame. Here it is used to
convert to grayscale, add cv2.GaussianBlur to reduce high frequency
noise from the image, then finally apply a cv2.Canny edge detection. The
returned frame is a binary edge map showing all the detected edges
present in the input frame.

img_proc_angle_detect(self,frame) Given binary edge map returned from
img_proc(frame) it applies a Hough transform to detect lines from the
edges. Each line is represented by a rho & theta pair. This has been
optimized to detect lines from the camera which detects angles.

img_proc_line_detect_center(self,frame) Given binary edge map returned
from img_proc(frame) it applies a Hough transform to detect lines from
the edges. Each line is represented by a rho & theta pair. This has been
optimized to detect lines from the center camera which detects a lines
distance from the center of the blade.

get_best_line(self,lines) Given an accumulator array created by running
a Hough transform on an edge map, this trims all the lines which are
outside the maximum cutting angle of the saw. This is to reduce the
likelihood of a false detection since no line intended to be cut will be
outside of this range. This eliminates a lot of the lines captured by
strongly colored grains in the wood. Then from the remaining lines, the
one with the highest accumulator value wins and is selected as the most
likely line.

get_saw_angle(self,line) Given a line, returns the angle which the saw
must rotate to in order to make the cut. The angle is first converted to
degrees and then it is signed to show if the angle is to the left or
right of zero degrees.

find_dist_from_center(self, line) Given a line detected by the center
camera, returns the distance of that line from the center of the blade.
The distance is in pixels. Positive values mean the line is to the right
of the blade and negative values mean it is to the left of the blade. A
zero value means the line is directly under the blade.

**Robo_vision2**

This is the main processing module used for computer vision in this
project. This is the 2^nd^ version. Robo_vision was originally only
designed to use one camera, but an updated version allowed for much more
robust image processing. Therefore, we rebuilt robo_vision in favor of
robo_vision2.

Here is a breakdown of the functions defined within this module:

open_cameras(model) This opens all three cameras and returns an array of
the captures.

return \[color_cap, angle_cap, center_cap\]

find_angle(model,cap) This uses functions from Model to pull a frame
from the angle camera, detect lines, pick the best line, and then find
the trimmed-mean best angle sample from the array of angle samples.
Angle returned is in degrees.

return angle

find_distance(model,cap) This uses functions from Model to pull a frame
from the center camera, detect lines, determine the best line, and then
find the distance from the blade in pixels.

return distance

wood_is_4x4(model,cap) This uses functions from Model to pull a frame
from the color camera, threshold the frame based on the calibrations in
Model, and then count the number of pixels detected in the region of
interest. When a piece of wood is tall enough to cover this region, all
pixels turn off and it is assumed that the wood is 4 inches tall rather
than 2 inches. Returns true if the wood is 4x4 and false if it is not.

wood_is_2x6(model,cap) This uses functions from Model to pull a frame
from the color camera, threshold the frame based on the calibrations in
Model, and then count the number of pixels detected in the region of
interest. When a piece of wood is wide enough to cover this region, all
pixels turn off and it is assumed that the wood is 6 inches wide rather
than 4 inches. Returns true if the wood is 2x6 and false if it is not.

wood_is_loaded(model,cap) This uses functions from Model to pull a frame
from the color camera, threshold the frame based on the calibrations in
Model, and then count the number of pixels detected in the region of
interest. This will return true if something is covering the intake
fence over the entire region of interest.

wood_is_under(model,cap) This uses functions from Model to pull a frame
from the color camera, threshold the frame based on the calibrations in
Model, and then count the number of pixels detected in the region of
interest. This will return true if something is covering the center
fence over the entire region of interest.
