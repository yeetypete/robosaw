from Model import Model
from __future__ import print_function
import robo_vision as rv
import display
import numpy as np
import cv2
import asyncio
import time
import pigpio
import argparse
from dual_g2_hpmd_rpi import motors, MAX_SPEED

# Define a custom exception to raise if a fault is detected.
class DriverFault(Exception):
    def __init__(self, driver_num):
        self.driver_num = driver_num

def raiseIfFault():
    if motors.motor1.getFault():
        raise DriverFault(1)
    if motors.motor2.getFault():
        raise DriverFault(2)

class Actuator(object):
    MAX_SPEED = _max_speed

    def __init__(self, pwm_pin, dir_pin, en_pin, flt_pin):
        self.pwm_pin = pwm_pin
        self.dir_pin = dir_pin
        self.en_pin = en_pin
        self.flt_pin = flt_pin

        _pi.set_pull_up_down(flt_pin, pigpio.PUD_UP) # make sure FLT is pulled up
        _pi.write(en_pin, 1) # enable driver by default

    def setSpeed(self, speed):
        if speed < 0:
            speed = -speed
            dir_value = 1
        else:
            dir_value = 0

        if speed > MAX_SPEED:
            speed = MAX_SPEED

        _pi.write(self.dir_pin, dir_value)
        #_pi.hardware_PWM(self.pwm_pin, 20000, int(speed * 6250 / 3));
        _pi.set_PWM_range(self.pwm_pin, 480)
        _pi.set_PWM_frequency(self.pwm_pin, 20000)
        _pi.set_PWM_dutycycle(self.pwm_pin, speed)
          # 20 kHz PWM, duty cycle in range 0-1000000 as expected by pigpio

    def enable(self):
        _pi.write(self.en_pin, 1)

    def disable(self):
        _pi.write(self.en_pin, 0)

    def getFault(self):
        return not _pi.read(self.flt_pin)

    def forceStop(self):
        global _pi
        _pi.stop()
        _pi = pigpio.pi()
        self.setSpeed(0)

def main():
    MAX_ANGLE = 10 # maximum angle that the blade can rotate for a miter cut
    CAMERA_ID = 0 # change this depending on which camera to use, default to zero, 1 for external usb camera
    Y_OFFSET = 50 # pixels from top of frame: negative -> above the border | positive -> below the border
    X_OFFSET = 500 # pixels from left edge of frame: negative -> left of the edge | positive -> to the right of the edge
    LINE_DETECTION_THRESHOLD = 50 # threshold for line detection -> minimum accumulator value for Hough Lines algo.

    initial_angle = -45 # angle of the blade at startup: get this from Dylan

    _max_speed = 480

    # Motor encoder pin assignments (still not assigned)
    #_pin_ENC1A =
    #_pin_ENC1B =
    #_pin_ENC2A =
    #_pin_ENC2B =  

    # linear actuator pin assignments
    _pin_M3DIR = 27
    _pin_M3EN = 17
    _pin_M3PWM = 20
    _pin_M3FLT = 7

    # PID parameters
    TARGET = 45
    KP = 0.02
    KD = 0.01
    KI = 0.0
    SAMPLETIME = 1

    posi = 0
    tprev = 0
    eprev = 0
    eintegral = 0

    # Argument parser for testing
    parser = argparse.ArgumentParser(description='set speed and ramp period parameter')
    parser.add_argument('speed', metavar='N', type=int, help='speed (-480, 480)')
    #parser.add_argument('period', metavar='t', type=float, help='period (seconds)')
    #parser.add_argument('motor', metavar='M', type=int, help='motor (1,2,3)')
    args = parser.parse_args()

    # generate a model based on initial conditions
    model = Model(initial_angle,MAX_ANGLE,Y_OFFSET,X_OFFSET,LINE_DETECTION_THRESHOLD)

    model.check_for_hand()
    if not model.hand_detected: 
        model.start()

    model.set_detected_theta(-np.pi/4)
    model.set_blade_angle(model.blade_angle)

    # open a camera feed 
    # do this before the motor control loop
    cap = cv2.VideoCapture(CAMERA_ID)
    if not cap.isOpened():
        raise IOError("Cannot open webcam")

    # pigpio initialization for GPIO pins
    _pi = pigpio.pi()
    if not _pi.connected:
        raise IOError("Can't connect to pigpio")

    # Define a custom exception to raise if a fault is detected.
    class DriverFault(Exception):
        def __init__(self, driver_num):
            self.driver_num = driver_num

    def raiseIfFault():
        if motors.motor1.getFault():
            raise DriverFault(1)
        if motors.motor2.getFault():
            raise DriverFault(2)

    # def readEncoder():
    #     b = _pi.read(_pin_ENC1B)
    #     if (b > 0):
    #         posi += 1
    #     else:
    #         posi -= 1

    # motor control can go in here

    # Initalize linear actuator
    motor3 = Actuator(_pin_M3PWM, _pin_M3DIR, _pin_M3EN, _pin_M3FLT)

    # PROTOTYPE TESTING SETUP - feed at constant speed, stop when line detected, actuate saw, stop
    # Uncomment for constant feed at 300 speed (speed: 0-480)
    # motors.setSpeeds(args.speed, args.speed)

    while True:
        try:
            line = rv.check_for_line(model, cap) # line contains [Line detected T/F , Distance of line from center of camera frame , angle of that line]
            if(line[0]):
                print("Distance: [" + str(line[1]) + "] Angle: [" + str(line[2]) + "]")
                motors.setSpeeds(0,0)
                raiseIfFault()
                # motor3.setSpeed(480)
                # time.sleep(3)
                # motor3.setSpeed(-480)
                # time.sleep(3)
                # motor3.setSpeed(0)
                break
        except DriverFault as e:
            print("Driver %s fault!" % e.driver_num)

    motors.forceStop()
    motor3.forceStop()


if __name__ == "__main__":
    import time
    s = time.perf_counter()
    main()
    
    elapsed = time.perf_counter() - s
    print(f"{__file__} executed in {elapsed:0.2f} seconds.")