from Model import Model
import robo_vision2 as rv
import RoboSaw as robosaw
import time

# linear actuator pin assignments
_pin_M3DIR = 27
_pin_M3EN = 17
_pin_M3PWM = 20
_pin_M3FLT = 7

blade_relay_pin = 26

def initialize():
    """ Run this first. Returns (model,caps)"""

    # Initialize Model object 'model'
    MAX_ANGLE = 50
    model = Model(MAX_ANGLE)

    # Initialize the camera captures
    caps = rv.open_cameras(model)

    return model,caps
    

def eject(model,caps):
    """ Ejects the wood. Pushes all wood out and raises the blade to default height """
    print("Ejecting wood")
    robosaw.motors.setSpeeds(args.speed, args.speed)
    time.sleep(10)
    robosaw.motors.setSpeeds(0, 0)
    return

def run(model,caps):
    """ Intake at idle speed until wood is detected, 
    find the angle of the line, 
    rotate blade to match angle, 
    move line under blade and center it, 
    wait for confirmation button,
    make cut, raise blade """

    _pi = robosaw.init_gpio()
    args = robosaw.init_args()
    motor3 = robosaw.Actuator(_pin_M3PWM, _pin_M3DIR, _pin_M3EN, _pin_M3FLT)

    # Check if wood is loaded
    while not rv.wood_is_loaded(model,caps[0]): # wait for the wood
        robosaw.motors.setSpeeds(args.speed, args.speed) # idle
    caps[0].release()

    # Once wood is loaded accumlate angle samples
    #rv.find_angle_display(model,caps[1])
    angles = []
    while len(angles) < model.num_angle_samples:
        # Get angles as it moves and save to angle[] array
        robosaw.motors.setSpeeds(args.speed, args.speed) # idle
        angle = rv.find_angle(model,caps[1])
        print("Angle: " + str(angle))
        if angle is not None:
            #blade_angle = angle
            angles.append(angle)
    # find most likely angle based on removing outliers and taking the mean
    blade_angle = model.best_angle(angles)
    caps[1].release()

    # Stop or slow the wood
    # ... TODO ...
    robosaw.motors.setSpeeds(args.speed - 50, args.speed - 50)

    # Rotate the blade to correct angle
    # ... TODO ...
    print("\nRotate blade to " + str(blade_angle) + " degrees.")

    # Move the line close to the center and slow down
    # ... TODO ...
    # Stop the wood under the blade
    #rv.find_center_display(model,caps[2])
    while True:
        dist = rv.find_distance(model,caps[2])
        if (dist is not None):
            if (dist < 0):
                print("Distance: " + str(dist))
                robosaw.motors.setSpeeds(0,0)
                break
            robosaw.feed(abs(dist), int(args.speed - 50))
            print("Distance: " + str(dist))

    # Calculate overshoot from stop point
    time.sleep(1) # wait a second to see if the wood oversoots
    dist = rv.find_distance(model,caps[2])
    print("\nOvershoot/undershoot distance: " + str(dist))

    # Spin the blade
    # ... TODO ...
    _pi.write(blade_relay_pin, 1)

    # Lower the blade as it spins
    # ... TODO ...

    motor3.setSpeed(480)
    time.sleep(4)

    # Raise blade again
    # ... TODO ...
    motor3.setSpeed(-480)
    time.sleep(4)

    # Stop the blade
    # ... TODO ...

    _pi.write(blade_relay_pin, 0)

    # Eject the wood
    # ... TODO ...

    robosaw.motor2.setSpeed(args.speed)
    time.sleep(2)

def close_caps(caps):
    """ Close the captures before terminating """
    for cap in caps:
        print("\n\nReleasing capture: " + str(cap))
        cap.release()

    robosaw.motors.forceStop()


if __name__ == "__main__":
    s = time.perf_counter()
    model,caps = initialize()
    #eject(model,caps) # Runs the feeding mechanism as long as 'feed' button is pressed
    run(model,caps) # Once 'Run' button pressed, waits for wood, finds angle, centers under the blade and makes the cut
    close_caps(caps) # Close cameras before shutting off
    elapsed = time.perf_counter() - s
    print(f"{__file__} executed in {elapsed:0.2f} seconds.")