import RoboSaw as robosaw
import RPi.GPIO as GPIO
from simple_pid import PID
import time

def rotate(model):
    count_events_per_rev = 2786.0
    table_ratio = 9.18
    _pin_M4DIR = 11
    _pin_M4EN = 15
    _pin_M4PWM = 21
    _pin_M4FLT = 2
    
    limit_max_pin = 16
    
    ENCA = 14
    ENCB = 19
    outcome = [0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0]
    last_AB = 0b00
    counter = 0.0
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(ENCA, GPIO.IN)
    GPIO.setup(ENCB, GPIO.IN)
    GPIO.setup(limit_max_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    _pi = robosaw.init_gpio()
    motor4 = robosaw.Actuator(_pin_M4PWM, _pin_M4DIR, _pin_M4EN, _pin_M4FLT, _pi)
    angle = model.desired_angle
    if model.home == 0:
        model.home = 1
        print("homing")
        while (True):
            A = GPIO.input(ENCA)
            B = GPIO.input(ENCB)
            curret_AB = (A << 1) | B
            position = (last_AB << 2) | curret_AB
            counter += outcome[position]
            print(counter)
            last_AB = curret_AB
            if (GPIO.input(limit_max_pin) == 0):
                print("home found")
                motor4.setSpeed(0)
                counter = 0
                break
            #speed = rotate_pid(counter)
            # set motor speed
            motor4.setSpeed(400)
            #rotate_pid.sample_time = time.time() - sample_time_start
        print("moving to angle")
        while (True):
            #sample_time_start = time.time()
            A = GPIO.input(ENCA)
            B = GPIO.input(ENCB)
            curret_AB = (A << 1) | B
            position = (last_AB << 2) | curret_AB
            counter += outcome[position]
            last_AB = curret_AB
            tablepos = -counter / count_events_per_rev * 360.0 / table_ratio
            if (tablepos > angle - 0.1 and tablepos < angle + 0.1 ):
                motor4.setSpeed(0)
                break
            print(angle - tablepos)
            #speed = rotate_pid(2 *(angle - tablepos))
            if (tablepos < angle):
                motor4.setSpeed(-200)
            if (tablepos > angle):
                motor4.setSpeed(200)
            # set motor speed
            #motor4.setSpeed(speed)
            #rotate_pid.sample_time = time.time() - sample_time_start
        motor4.forceStop()
        model.last_angle = 0
    else:
        tablepos = model.last_angle
        dest = angle - model.last_angle
        counter = 0;
        while (True):
            #sample_time_start = time.time()
            A = GPIO.input(ENCA)
            B = GPIO.input(ENCB)
            curret_AB = (A << 1) | B
            position = (last_AB << 2) | curret_AB
            counter += outcome[position]
            last_AB = curret_AB
            tablepos = -counter / count_events_per_rev * 360.0 / table_ratio
            if (tablepos > dest - 0.1 and tablepos < dest + 0.1 ):
                motor4.setSpeed(0)
                break
            print(angle - tablepos)
            #speed = rotate_pid(2 *(angle - tablepos))
            if (tablepos < dest):
                motor4.setSpeed(-200)
            if (tablepos > dest):
                motor4.setSpeed(200)
            # set motor speed
            #motor4.setSpeed(speed)
            #rotate_pid.sample_time = time.time() - sample_time_start
        model.last_angle = angle
    
