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
import matplotlib.pyplot as plt
from ButtonHandler import ButtonHandler
from rotate import rotate

### USED GPIO PINS ###

# 4, 5, 6, 7, 8, 9, 10, 12, 13, 16, 17, 20, 22, 23, 24, 25, 26, 27

### AVAILABLE PINS ###

# 0, 1, 3, 14, 19
 
# PID
max_pid_speed = 480
kP = 3.0
kI = 0.001
kD = 0.15

kP_a = 4
kI_a = 0.001
kD_a = 0.01
"""
kP = 1.5
kI = 0.4
kD = 0.36
"""

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
        robosaw.motors.setSpeeds(-480,-480)
        time.sleep(0.1)
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

def raise_blade():
    _pi = robosaw.init_gpio()
    motor3 = robosaw.Actuator(_pin_M3PWM, _pin_M3DIR, _pin_M3EN, _pin_M3FLT, _pi)
    motor3.setSpeed(480)
    time.sleep(7)
    motor3.setSpeed(0)

def cut(model):
    if model.cut_ready == False:
        return
    model.cut_initiated = True
    try:
        rv.show(model)
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
        print("\nCut initiated.")

        caps[0] = cv2.VideoCapture(model.color_cam_id)######################
        while not caps[0].isOpened():###
            print("Cannot open color camera")####
            caps[0] = cv2.VideoCapture(model.color_cam_id)######


        if not rv.wood_is_under(model,caps[0]): # check for end of wood
            return
            #robosaw.motors.setSpeeds(args.speed, args.speed) # Set new speed
            #time.sleep(5) # Eject for 5 seconds if end is detected

        if rv.wood_is_under(model,caps[0]): # Final check to make sure something is actually under the blade
            # Spin the blade
            print("Blade ON. CAUTION!")
            _pi.write(blade_relay_pin, 1) # spin up the blade

            # Lower the blade as it spins
            act_time = 6.5
            motor3.setSpeed(-480)
            #time.sleep(act_time)
            t_end = time.time() + act_time/4
            while time.time() < t_end:
                if(rv.wood_is_under(model,caps[0])):
                    #print("Chop... \n\n")
                    time.sleep(0.001)
                    #rv.show(model)
                else:
                    print("Wood not under blade")

            t_end = time.time() + 3*act_time/4
            while time.time() < t_end:
                if(rv.wood_is_under(model,caps[0])):
                    rv.show(model)
                else:
                    print("Wood not under blade")
                    
                    

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
            #else:
                # TODO - lower only a little for 4x4    
        else:
            print("\nWood is not propperly in place to make the cut")
            return
        # indent until Eject if uncommented later
        
        model.cut_initiated = False
        #caps[0].release() #########################
        close_caps(caps) # Always close captures after
        
    except Exception as e: 
        print(e)
        close_caps(caps) # Close caps if program fails or gets cancelled
        model.cut_initiated = False
        
    finally:
        _pi.write(blade_relay_pin, 0) # Always turn off the blade
        model.cut_initiated = False
        model.cut_ready = False

def run(model):
    """ Intake at idle speed until wood is detected, 
    find the angle of the line, 
    rotate blade to match angle, 
    move line under blade and center it, 
    wait for confirmation button,
    make cut, raise blade """
    if model.cut_initiated == True:
        return
    model.cut_ready = False

    try:
        _pi = robosaw.init_gpio()
        args = robosaw.init_args()
        motor3 = robosaw.Actuator(_pin_M3PWM, _pin_M3DIR, _pin_M3EN, _pin_M3FLT, _pi)
        motor4 = robosaw.Actuator(_pin_M4PWM, _pin_M4DIR, _pin_M4EN, _pin_M4FLT, _pi)
        _pi.set_mode(limit_home_pin, pigpio.INPUT)
        _pi.set_mode(limit_max_pin, pigpio.INPUT)
        _pi.set_mode(blade_relay_pin, pigpio.OUTPUT)
        _pi.set_pull_up_down(limit_home_pin, pigpio.PUD_UP)
        _pi.set_pull_up_down(limit_max_pin, pigpio.PUD_UP)

        ############# PID ###############
        center_pid = PID(0, 0, 0, setpoint=0)
        center_pid.setpoint = 0
        center_pid.proportional_on_measurement = False
        center_pid.tunings = (kP, kI, kD)
        center_pid.sample_time = 0.045 # Get this from measuting the line distance capture time
        center_pid.output_limits = (-max_pid_speed,max_pid_speed)

        ang_pid = PID(0, 0, 0, setpoint=0)
        ang_pid.setpoint = 0
        ang_pid.proportional_on_measurement = False
        ang_pid.tunings = (kP_a, kI_a, kD_a)
        ang_pid.sample_time = 0.075 # Get this from measuting the line distance capture time
        ang_pid.output_limits = (-max_pid_speed,max_pid_speed)
        #################################

        caps = rv.open_cameras(model)
        #GPIO.setup(run_btn, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Check if wood is loaded
        if rv.wood_is_loaded(model,caps[0]):
            rv.show(model)
            time.sleep(0.5)
            rv.show(model)
        else:
            rv.show(model)
            time.sleep(0.5)
            rv.show(model)
        while not rv.wood_is_loaded(model,caps[0]): # wait for the wood
            rv.show(model)
            robosaw.motors.setSpeeds(args.speed, args.speed) # idle
        """
        if rv.wood_is_4x4():
            model._4x4_detected = True
        else:
            model._4x4_detected = False

        if rv.wood_is_4x4():
            model.max_angle = 10
        else:
            model.max_angle = 52
        """

        caps[0].release()


        # Once wood is loaded accumlate angle samples
        angles = []
        print("\nLooking for angle...")
        while len(angles) < model.num_angle_samples:
            # Get angles as it moves and save to angle[] array
            robosaw.motors.setSpeeds(args.speed, args.speed) # idle
            angle = rv.find_angle(model,caps[1])
            rv.show(model)
            if angle is not None:
                #print("Angle: " + str(angle))
                angles.append(angle)
        # find most likely angle based on removing outliers and taking the mean
        blade_angle = model.best_angle(angles)
        
        # Stop or slow the wood
        # ... TODO ...
        robosaw.motors.setSpeeds(0, 0) # idle

        # Rotate the blade to correct angle
        
            
        #blade_angle = 0 # override the angle if its a 4x4
        print("\nAngle found. Rotate blade to " + str(blade_angle) + " degrees.\n")
        caps[1].release() # Close the angle camera
        if (model.home == 0):
            model.desired_angle = 52
            rotate(model)
        else:
            model.desired_angle = blade_angle
            rotate(model)
        robosaw.motors.setSpeeds(args.speed, args.speed) # idles

        """
        # Move the line close to the center and slow down
        caps[0] = cv2.VideoCapture(model.color_cam_id)
        while not caps[0].isOpened():
            print("Cannot open color camera")
            caps[0] = cv2.VideoCapture(model.color_cam_id)

        while not rv.wood_is_under(model,caps[0]):
            #rv.show(model)
            continue # Kepps moving the wood until it is under the center camera
        """

        # Slowdown the wood now that it is looking for a center line

        slowdown = 0.0 # value between 0 and 1: slowdown factor ####################################### Slowdown factor

        robosaw.motors.setSpeeds(args.speed*(1-slowdown), args.speed*(1-slowdown)) # Set new speed

        """
        if not rv.wood_is_loaded(model,caps[0]): # check for end of wood
            robosaw.motors.setSpeeds(args.speed, args.speed) # Set new speed
            time.sleep(8)
            return
        """
        #caps[0].release()
        # Stop the line under the blade
        
        """
        while True:
            dist = rv.find_distance(model,caps[2])
            #rv.show(model)
            if dist is not None:
                print("Line detected.")
                #hard_stop(0.5)
                break
        """
        # Plotting #
        setpoint, y, x = [], [], []
        start_time = time.time()

        #angle = rv.find_angle(model,caps[2])
        dist = rv.find_distance(model,caps[2])
        
        #print("First dist: " + str(dist))
        
        
        while dist == None or dist > 0:
            dist = rv.find_distance(model,caps[2])
            #angle = rv.find_angle(model,caps[2])
        #stop(0.01)
        print("PID start dist: " + str(dist))
        #print("PID start angle: " + str(angle))
        
        t_end = time.time() + 10 # run PID loop for specified time after the line is detected
        
        #while time.time() < t_end: 
        print(" \n-- Press 'RUN' to preview the cut.\n\n-- Tap 'RUN' to skip without moving to next line.\n   or press 'CUT' to make the cut.")
        while True:
            sample_time_start = time.time()

            
            if model.cut_initiated == True or model.stop_pid == True:
            #if model.cut_initiated == True:
                # Quit #
                #close_caps(caps)
                #model.cut_initiated = False
                #model.cut_ready = False
                #model.stop_pid = False
                return
            

            if GPIO.input(run_btn) == GPIO.LOW:
                #time.sleep(0.1)
                robosaw.motors.setSpeeds(0,0)
                motor4.setSpeed(0)
                model.cut_ready = True
                rv.show(model)
                return

            """
            if angle is not None and dist is not None:
                ang_speed = -ang_pid(angle)
                if dist < 10:
                    motor4.setSpeed(ang_speed)
                else:
                    motor4.setSpeed(0)
            else:
                motor4.setSpeed(0)
            """

            if (dist is not None):
                model.dist = dist
                #print("Distance: " + str(dist))
                speed = center_pid(dist)
                robosaw.motors.setSpeeds(speed,speed)

                
                
                
                x += [time.time() - start_time]
                y += [dist]
                setpoint += [center_pid.setpoint]
                
                
                model.cut_ready = True
            
            #angle = rv.find_angle(model,caps[2])
            dist = rv.find_distance(model,caps[2])
            
            center_pid.sample_time = time.time() - sample_time_start
            #print("Sample time: " + str(time.time() - sample_time_start))

        robosaw.motors.setSpeeds(0,0)
        # Show #
        rv.show(model)
        plt.plot(x, y, label='measured')
        plt.plot(x, setpoint, label='target')
        plt.xlabel('time (seconds)')
        plt.ylabel('Distance from center (pixels)')
        plt.legend()
        plt.savefig("PID_plot.png")
        # dictionary of lists  
        #dict = {'Time (seconds)': x, 'Distance (pixels)': y, 'setpoint': setpoint} 
       
        rows = [x,y,setpoint]

        np.savetxt("PID_plot.csv", 
                   rows,
                   delimiter =", ", 
                   fmt ='% s')
    
        plt.show()

        """
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
        """

        # Close the center capture
        caps[2].release()
 
    except Exception as e: 
        print(e)
        close_caps(caps)
        #model.cut_ready = False
    finally:
        rv.show(model)
        close_caps(caps)
        robosaw.motors.setSpeeds(0,0)
        model.cut_ready = True

def run_btn_callback(channel):
    print("Run button pressed")
    run(model)
    robosaw.motors.forceStop()
    

def eject_btn_callback(channel):
    print("Eject button pressed")
    #eject()
    robosaw.motors.forceStop()

def cut_btn_callback(channel):
    #model.stop_pid = True
    #model.cut_initiated = True
    robosaw.motors.forceStop()
    print("Cut button pressed")
    cut(model)
    robosaw.motors.forceStop()
    
    
# Handle keyboard interrupts
def signal_handler(sig, frame):
    GPIO.cleanup()
    sys.exit(0)

model = initialize() # global model
if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(run_btn, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(eject_btn, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(cut_btn, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    raise_blade()

    """
    run_flag = 0
    cut_flag = 0
    center_flag = 0
    cut_flag = 0
    """

    """
    GPIO.setup(4, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    cb = ButtonHandler(4, real_cb, edge='falling', bouncetime=100)
    cb.start()
    GPIO.add_event_detect(4, GPIO.FALLING, callback=cb)
    """

    # interrupt to run the saw
    cb_run = ButtonHandler(run_btn, run_btn_callback, edge='falling', bouncetime=10)
    cb_run.start()
    GPIO.add_event_detect(run_btn, GPIO.FALLING, 
            callback=cb_run)

    # interrupt to cut
    cb_cut = ButtonHandler(cut_btn, cut_btn_callback, edge='falling', bouncetime=10)
    cb_cut.start()
    GPIO.add_event_detect(cut_btn, GPIO.FALLING, 
            callback=cb_cut)

    # interrupt to feed the wood manually
    cb_eject = ButtonHandler(eject_btn, eject_btn_callback, edge='falling', bouncetime=10)
    cb_eject.start()
    GPIO.add_event_detect(eject_btn, GPIO.FALLING, 
            callback=cb_eject)


    print("Ready.")

    signal.signal(signal.SIGINT, signal_handler)
    signal.pause()
    

    try:
        robosaw.motors.forceStop()
    except Exception as e: 
        print(e)
        robosaw.motors.forceStop()
    finally:
        robosaw.motors.forceStop()

    
