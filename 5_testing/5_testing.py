from Model import Model
import robo_vision2 as rv
import RoboSaw as robosaw
import time
import cv2
import sys
import signal
import pigpio
import RPi.GPIO as GPIO
import os
import numpy as np
from simple_pid import PID

### USED GPIO PINS ###

# 4, 5, 6, 7, 8, 9, 10, 12, 13, 16, 17, 20, 22, 23, 24, 25, 26, 27

### AVAILABLE PINS ###

# 0, 1, 3, 14, 19

# flags
run_flag = True

# Button pin assignments
run_btn = 9
eject_btn = 8
cut_btn = 10

# linear actuator pin assignments
_pin_M3DIR = 27
_pin_M3EN = 17
_pin_M3PWM = 20
_pin_M3FLT = 7

# rotator pin assignments

_pin_M4DIR = 11
_pin_M4EN = 15
_pin_M4PWM = 21
_pin_M4FLT = 2

blade_relay_pin = 26
limit_home_pin = 4
limit_max_pin = 16



def initialize():
    """ Run this first. Returns (model,caps)"""

    # Initialize Model object 'model'
    MAX_ANGLE = 52
    model = Model(MAX_ANGLE)

    # Initialize the camera captures
    #caps = rv.open_cameras(model)

    return model
    
def close_caps(caps):
    """ Close the captures before terminating """
    try:
        for cap in caps:
            print("\nReleasing capture: " + str(cap))
            cap.release()
    except Exception as e: 
        print(e)
        robosaw.motors.forceStop()
    finally:
        robosaw.motors.forceStop()

def eject():
    """ Ejects the wood. Pushes all wood out and raises the blade to default height """
    try:
        args = robosaw.init_args()
        robosaw.motors.setSpeeds(args.speed, args.speed)
        time.sleep(10)
        robosaw.motors.setSpeeds(0, 0)
        print("\nEjecting wood")
    except Exception as e: 
        print(e)
        robosaw.motors.forceStop()
        print("\nUnable to eject")
    finally:
        return

def bump():
    """ bumps the wood for a short period of time """
    try:
        speed = 300
        robosaw.motors.setSpeeds(0,0)
        robosaw.motors.setSpeeds(speed,speed)
        time.sleep(0.01)
        robosaw.motors.setSpeeds(0,0)
        time.sleep(0.08)
    except Exception as e: 
        print(e)
        print("Cannot bump the wood")
        robosaw.motors.setSpeeds(0,0)
        return

def bump_prop(dist):
    """ bumps the wood for a short period of time """
    try:
        time = dist*0.001
        robosaw.motors.setSpeeds(0,0)
        time.sleep(time)
        robosaw.motors.setSpeeds(300,300)
        time.sleep(time)
        robosaw.motors.setSpeeds(0,0)
        time.sleep(0.08)
    except Exception as e: 
        print(e)
        print("Cannot bump the wood")
        robosaw.motors.setSpeeds(0,0)
        return

def smart_bump(dist):
    """ bumps the wood for a short period of time """
    try:
        dist = -dist
        speed = np.interp(dist,[0,100],[0,100])
        robosaw.motors.setSpeeds(speed,speed)
    except Exception as e: 
        print(e)
        print("Cannot bump the wood")
        robosaw.motors.setSpeeds(0,0)
        return

def hard_stop(secs):
    """ bumps the wood for a short period of time """
    try:
        robosaw.motors.setSpeeds(-200,-200)
        time.sleep(0.005)
        robosaw.motors.setSpeeds(0,0)
        time.sleep(secs)
    except Exception as e: 
        print(e)
        print("Cannot stop the wood")
        robosaw.motors.setSpeeds(0,0)
        pass

def stop(secs):
    """ bumps the wood for a short period of time """
    try:
        robosaw.motors.setSpeeds(0,0)
        time.sleep(secs)
    except Exception as e: 
        print(e)
        print("Cannot stop the wood")
        robosaw.motors.setSpeeds(0,0)
        pass

def cut(model):
    try:
        caps = rv.open_cameras(model)
        _pi = robosaw.init_gpio()
        args = robosaw.init_args()
        motor3 = robosaw.Actuator(_pin_M3PWM, _pin_M3DIR, _pin_M3EN, _pin_M3FLT, _pi)
        _pi.set_mode(limit_home_pin, pigpio.INPUT)
        _pi.set_mode(limit_max_pin, pigpio.INPUT)
        _pi.set_mode(blade_relay_pin, pigpio.OUTPUT)
        _pi.set_pull_up_down(limit_home_pin, pigpio.PUD_UP)
        _pi.set_pull_up_down(limit_max_pin, pigpio.PUD_UP)

        #print("\nPress 'cut' button to make the cut.") # Alternatively use buttons or both for redundancy
        #GPIO.wait_for_edge(cut_btn, GPIO.FALLING) # Blocking statement that waits for user to press the cut button before proceeding to make the cut
        
        

        #not GPIO.input(cut_btn)
        #if not GPIO.input(cut_btn): # indent until Eject if uncommented later
        print("\nCut initiated. GTFO!")

        caps[0] = cv2.VideoCapture(model.color_cam_id)######################
        while not caps[0].isOpened():###
            print("Cannot open color camera")####
            caps[0] = cv2.VideoCapture(model.color_cam_id)######

        if not rv.wood_is_under(model,caps[0]): # check for end of wood
            robosaw.motors.setSpeeds(args.speed, args.speed) # Set new speed
            time.sleep(5)
            return

        if rv.wood_is_under(model,caps[0]): # Final check to make sure something is actually under the blade
            # Spin the blade
            _pi.write(blade_relay_pin, 1) # spin up the blade

            # Lower the blade as it spins
                act_time = 7
                motor3.setSpeed(-480)
                #time.sleep(act_time)
                t_end = time.time() + act_time
                while time.time() < t_end:
                    if(rv.wood_is_under(model,caps[0])):
                        rv.show(model)
                    else:
                        print("Wood not under blade. Stopping...")
                        return
                    

                # Raise blade again
                # ... TODO ...
                motor3.setSpeed(480)
                t_end = time.time() + act_time/2
                while time.time() < t_end:
                    if(rv.wood_is_under(model,caps[0])):
                        rv.show(model)

                _pi.write(blade_relay_pin, 0) # Stop the blade

                t_end = time.time() + act_time/2
                while time.time() < t_end:
                    if(rv.wood_is_under(model,caps[0])):
                        rv.show(model)

                motor3.setSpeed(0)
            else:
                # TODO - lower only a little for 4x4
                
        else:
            print("\nWood is not propperly in place to make the cut")
        # indent until Eject if uncommented later
        
        caps[0].release() #########################
        close_caps(caps) # Always close captures after
    except Exception as e: 
        print(e)
        close_caps(caps) # Close caps if program fails or gets cancelled
    finally:
        close_caps(caps)

def run(model):
    """ Intake at idle speed until wood is detected, 
    find the angle of the line, 
    rotate blade to match angle, 
    move line under blade and center it, 
    wait for confirmation button,
    make cut, raise blade """
    
    
    try:
        _pi = robosaw.init_gpio()
        args = robosaw.init_args()
        motor3 = robosaw.Actuator(_pin_M3PWM, _pin_M3DIR, _pin_M3EN, _pin_M3FLT, _pi)
        _pi.set_mode(limit_home_pin, pigpio.INPUT)
        _pi.set_mode(limit_max_pin, pigpio.INPUT)
        _pi.set_mode(blade_relay_pin, pigpio.OUTPUT)
        _pi.set_pull_up_down(limit_home_pin, pigpio.PUD_UP)
        _pi.set_pull_up_down(limit_max_pin, pigpio.PUD_UP)

        ############# PID ###############
        center_pid = PID(0, 0, 0, setpoint=0)
        center_pid.setpoint = 0
        center_pid.proportional_on_measurement = False
        center_pid.tunings = (2.45, 0.03, 0.15)
        center_pid.sample_time = 0.07 # Get this from measuting the line distance capture time
        center_pid.output_limits = (-100, 200)
        #################################

        caps = rv.open_cameras(model)

        # Check if wood is loaded
        while not rv.wood_is_loaded(model,caps[0]): # wait for the wood
            rv.show(model)
            robosaw.motors.setSpeeds(args.speed, args.speed) # idle
        
        if rv.wood_is_4x4():
            model.4x4_detected = True
        else:
            model.4x4_detected = False

        if rv.wood_is_2x6():
            model.max_angle = 10
        else:
            model.max_angle = 52
        caps[0].release()


        # Once wood is loaded accumlate angle samples
        angles = []
        while len(angles) < model.num_angle_samples:
            # Get angles as it moves and save to angle[] array
            robosaw.motors.setSpeeds(args.speed, args.speed) # idle
            angle = rv.find_angle(model,caps[1])
            rv.show(model)
            if angle is not None:
                print("Angle: " + str(angle))
                angles.append(angle)
        # find most likely angle based on removing outliers and taking the mean
        blade_angle = model.best_angle(angles)
        
        # Stop or slow the wood
        # ... TODO ...
        

        # Rotate the blade to correct angle
        
        #blade_angle = 0 # override the angle if its a 2x6
        print("\nRotate blade to " + str(blade_angle) + " degrees.\n")
        caps[1].release() # Close the angle camera

        #caps = rv.open_cameras(model) # Open caps again
        #caps[1].release()
        # Move the line close to the center and slow down
        caps[0] = cv2.VideoCapture(model.color_cam_id)
        while not caps[0].isOpened():
            print("Cannot open color camera")
            caps[0] = cv2.VideoCapture(model.color_cam_id)

        while not rv.wood_is_under(model,caps[0]):
            rv.show(model)
            continue # Kepps moving the wood until it is under the center camera
        
        # Slowdown the wood now that it is looking for a center line

        slowdown = 0.40 # value between 0 and 1: slowdown factor ####################################### Slowdown factor

        robosaw.motors.setSpeeds(args.speed*(1-slowdown), args.speed*(1-slowdown)) # Set new speed

        #time.sleep(0.1)
        """
        if not rv.wood_is_loaded(model,caps[0]): # check for end of wood
            robosaw.motors.setSpeeds(args.speed, args.speed) # Set new speed
            time.sleep(8)
            return
        """
        caps[0].release()
        # Stop the line under the blade
        
        while True:
            stop_dist = rv.find_distance(model,caps[2])
            rv.show(model)
            if stop_dist is not None:
                print("Line detected.")
                hard_stop(0.1)
                break
        

        print("Start PID.")
        t_end = time.time() + 5 # run PID loop for specified time after the line is detected
        while time.time() < t_end:
            dist = rv.find_distance(model,caps[2])
            if (dist is not None):
                print("Distance: " + str(dist))
                
                speed = center_pid(dist)
                robosaw.motors.setSpeeds(speed,speed)

        robosaw.motors.setSpeeds(0,0)
        # Calculate overshoot from stop point
        t_end = time.time() + 0.5
        while time.time() < t_end:
            dist = rv.find_distance(model,caps[2])
            if dist is not None:
                rv.show(model)
        
        #time.sleep(0.5) # wait half a second to see if the wood oversoots
        dist = rv.find_distance(model,caps[2])
        rv.show(model)
        print("\nOvershoot/undershoot distance: " + str(dist))
        

        # Close the center capture
        caps[2].release()
 
    except Exception as e: 
        print(e)
        close_caps(caps)
    finally:
        close_caps(caps)

def run_btn_callback(channel):
    print("Run button pressed")
    run(model)
    

def eject_btn_callback(channel):
    print("Eject button pressed")
    eject()

def cut_btn_callback(channel):
    #cut(model)
    print("cut button pressed")
    

    


def signal_handler(sig, frame):
    GPIO.cleanup()
    sys.exit(0)

model = initialize() # global model
if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(run_btn, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(eject_btn, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(cut_btn, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    rv.show(model)

    """
    run_flag = 0
    cut_flag = 0
    center_flag = 0
    cut_flag = 0
    """

    # interrupt to run the saw
    GPIO.add_event_detect(run_btn, GPIO.FALLING, 
            callback=run_btn_callback, bouncetime=300)

    GPIO.add_event_detect(cut_btn, GPIO.FALLING, 
            callback=cut_btn_callback, bouncetime=300)

    # interrupt to feed the wood manually
    GPIO.add_event_detect(eject_btn, GPIO.FALLING, 
            callback=eject_btn_callback, bouncetime=300)


    signal.signal(signal.SIGINT, signal_handler)
    signal.pause()
    

    try:
        #run()
        robosaw.motors.forceStop()
    except Exception as e: 
        print(e)
        robosaw.motors.forceStop()
    finally:
        robosaw.motors.forceStop()

    """
    while(1):
        try:
            run()
            robosaw.motors.forceStop()
        except Exception as e: 
            print(e)
            robosaw.motors.forceStop()
        finally:
            robosaw.motors.forceStop()
    """

    