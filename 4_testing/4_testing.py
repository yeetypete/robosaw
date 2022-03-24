from Model import Model
import robo_vision2 as rv
import RoboSaw as robosaw
import time
import sys
import signal
import pigpio
import RPi.GPIO as GPIO

### USED GPIO PINS ###

# _3_, 4, 5, 6, 7, _8_, 12, 13, 16, 17, 20, 22, 23, 24, 25, 26, 27


# Button pin assignments
run_btn = 3
eject_btn = 8

# linear actuator pin assignments
_pin_M3DIR = 27
_pin_M3EN = 17
_pin_M3PWM = 20
_pin_M3FLT = 7

blade_relay_pin = 26
limit_home_pin = 4
limit_max_pin = 16

def initialize():
    """ Run this first. Returns (model,caps)"""

    # Initialize Model object 'model'
    MAX_ANGLE = 50
    model = Model(MAX_ANGLE)

    # Initialize the camera captures
    caps = rv.open_cameras(model)

    return model,caps
    
def close_caps(caps):
    """ Close the captures before terminating """
    for cap in caps:
        print("\nReleasing capture: " + str(cap))
        cap.release()
    robosaw.motors.forceStop()

def eject():
    """ Ejects the wood. Pushes all wood out and raises the blade to default height """
    print("\nEjecting wood")
    try:
        args = robosaw.init_args()
        robosaw.motors.setSpeeds(args.speed, args.speed)
        time.sleep(10)
        robosaw.motors.setSpeeds(0, 0)
    except:
        print("\nUnable to eject")
    return

def run():
    """ Intake at idle speed until wood is detected, 
    find the angle of the line, 
    rotate blade to match angle, 
    move line under blade and center it, 
    wait for confirmation button,
    make cut, raise blade """
    try:
        model,caps = initialize()
    except:
        print("\nUnable to initialize model and caps")

    try:
        _pi = robosaw.init_gpio()
        args = robosaw.init_args()
        motor3 = robosaw.Actuator(_pin_M3PWM, _pin_M3DIR, _pin_M3EN, _pin_M3FLT, _pi)
        _pi.set_mode(limit_home_pin, pigpio.INPUT)
        _pi.set_mode(limit_max_pin, pigpio.INPUT)
        _pi.set_mode(blade_relay_pin, pigpio.OUTPUT)
        _pi.set_pull_up_down(limit_home_pin, pigpio.PUD_UP)
        _pi.set_pull_up_down(limit_max_pin, pigpio.PUD_UP)

        # Check if wood is loaded
        while not rv.wood_is_loaded(model,caps[0]): # wait for the wood
            robosaw.motors.setSpeeds(args.speed, args.speed) # idle
        #caps[0].release()

        # Once wood is loaded accumlate angle samples
        angles = []
        while len(angles) < model.num_angle_samples:
            # Get angles as it moves and save to angle[] array
            robosaw.motors.setSpeeds(args.speed, args.speed) # idle
            angle = rv.find_angle(model,caps[1])
            if angle is not None:
                print("Angle: " + str(angle))
                angles.append(angle)
        # find most likely angle based on removing outliers and taking the mean
        blade_angle = model.best_angle(angles)
        caps[1].release() # Close the angle camera

        # Stop or slow the wood
        # ... TODO ...
        robosaw.motors.setSpeeds(args.speed - 50, args.speed - 50)

        # Rotate the blade to correct angle
        # ... TODO ...
        print("\nRotate blade to " + str(blade_angle) + " degrees.\n")

        # Move the line close to the center and slow down
        # ... TODO ...
        while not rv.wood_is_under(model,caps[0]):
            continue # Kepps moving the wood until it is under the center camera

        # Stop the wood under the blade
        while True:
            dist = rv.find_distance(model,caps[2])
            if (dist is not None):
                if (dist < 50):
                    print("Distance: " + str(dist))
                    robosaw.motors.setSpeeds(100,100)
                    if (dist < 0):
                        print("Distance: " + str(dist))
                        robosaw.motors.setSpeeds(0,0)
                        break
                robosaw.feed(abs(dist), int(args.speed - 50))
                print("Distance: " + str(dist))

        # Calculate overshoot from stop point
        time.sleep(1) # wait a second to see if the wood oversoots
        dist = rv.find_distance(model,caps[2])
        print("\nOvershoot/undershoot distance: " + str(-dist))

        if rv.wood_is_under: # Final check to make sure something is actually under the blade
            # Spin the blade
            # ... TODO ...
            _pi.write(blade_relay_pin, 1)

            # Lower the blade as it spins
            # ... TODO ...

            motor3.setSpeed(480)
            time.sleep(1)

            # Raise blade again
            # ... TODO ...
            motor3.setSpeed(-480)
            time.sleep(1)
            motor3.setSpeed(0)

            # Stop the blade
            # ... TODO ...

            _pi.write(blade_relay_pin, 0)
        else:
            print("\nWood is not propperly in place to make the cut")
        # Eject the wood
        # ... TODO ...

        #robosaw.motors.motor2.setSpeed(args.speed)
        #time.sleep(2)

        close_caps(caps) # Always close captures after
    except:
        close_caps(caps) # Close caps if program fails or gets cancelled

def run_btn_callback(channel):
    print("Run button pressed")
    run()


def eject_btn_callback(channel):
    print("Eject button pressed")
    eject()

def signal_handler(sig, frame):
    GPIO.cleanup()
    sys.exit(0)

if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(run_btn, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    s = time.perf_counter()

    # interrupt to run the saw
    GPIO.add_event_detect(run_btn, GPIO.FALLING, 
            callback=run_btn_callback, bouncetime=100)

    # interrupt to feed the wood manually
    GPIO.add_event_detect(eject_btn, GPIO.FALLING, 
            callback=eject_btn_callback, bouncetime=100)

    signal.signal(signal.SIGINT, signal_handler)
    signal.pause()

    elapsed = time.perf_counter() - s
    print(f"{__file__} executed in {elapsed:0.2f} seconds.")