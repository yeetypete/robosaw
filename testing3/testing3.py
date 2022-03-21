from Model import Model
import robo_vision2 as rv
import RoboSaw as robosaw
import time

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
    return

def run(model,caps):
    """ Intake at idle speed until wood is detected, 
    find the angle of the line, 
    rotate blade to match angle, 
    move line under blade and center it, 
    wait for confirmation button,
    make cut, raise blade """

    ### constants ####
    MAX_ANGLE = 50
    ##################

    _pi = robosaw.init_gpio()
    args = robosaw.init_args()

    # Check if wood is loaded
    while not rv.wood_is_loaded(model,caps[0]): # wait for the wood
        robosaw.motors.setSpeeds(args.speed, args.speed) # idle
    #caps[0].release()

    # Once wood is loaded accumlate angle samples
    angles = []
    while len(angles) < model.max_angle_samples:
        # Get angles as it moves and save to angle[] array
        robosaw.motors.setSpeeds(args.speed, args.speed) # idle
        angle = rv.find_angle(model,caps[1])
        angles.append(angle)
    # find most likely angle based on removing outliers and taking the mean
    final_angle = model.best_angle(angles)

    # Pause the wood from moving
    # ... TODO ...

    # Rotate the blade to correct angle
    # ... TODO ...
    print("Rotate blade to " + str(final_angle) + "degrees.")

    # Move the line close to the center and slow down
    # ... TODO ...

    # Find the distance of the line from the blade's plane of intersection
    while True:
        dist = rv.find_distance(model,caps[2])
        if (dist is not None):
            if (dist > 0):
                print("Distance: " + str(dist))
                break
            robosaw.feed(abs(dist), int(args.speed))
            print("Distance: " + str(dist))

    # Calculate overshoot
    time.sleep(0.5) # wait half a second to see how far the wood oversoots
    dist = rv.find_distance(model,caps[2])
    print("Overshoot distance: " + str(abs(dist)))

    # Start the blade spinning
    # ... TODO ...

    # Lower the blade as it spins
    # ... TODO ...

    # Raise blade again
    # ... TODO ...

    # Stop the blade spinning
    # ... TODO ...

    # Eject the wood
    # ... TODO ...

    # Close the captures before terminating!
    for cap in caps:
        print("\nReleasing: " + str(cap) + "...\n")
        cap.release()

    robosaw.motors.forceStop()


if __name__ == "__main__":
    import time
    s = time.perf_counter()
    model,caps = initialize()
    #eject(model,caps)
    run(model,caps)
    elapsed = time.perf_counter() - s
    print(f"{__file__} executed in {elapsed:0.2f} seconds.")