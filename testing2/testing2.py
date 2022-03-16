from Model import Model
import robo_vision2 as rv
import RoboSaw as robosaw

def main():
    """ Main fn for testing robo vision v2 using multiple cameras """
    ### constants ####
    MAX_ANGLE = 50
    ##################

        # Initialize Model object 'model'
    model = Model(MAX_ANGLE)

        # Initialize the camera captures
    caps = rv.open_cameras(model)
    caps[1].release() # ignore angle for this test


    _pi = robosaw.init_gpio()
    args = robosaw.init_args()

        # Check if wood is loaded
    wood_loaded = False
    while not wood_loaded: # wait for the wood
        wood_loaded = rv.wood_is_loaded(model,caps[0])
    caps[0].release()


        # Find the distance of the line from the blade's plane of intersection
    dist = rv.find_distance(model,caps[2])
    while (dist is not None and dist < 0):
        dist = -(rv.find_distance(model,caps[2]))
        robosaw.feed(dist, args.speed)
        print("Distance: " + str(dist))

        # Close the captures before terminating!
    for cap in caps:
        print("\nReleasing: " + str(cap) + "...\n")
        cap.release()

    robosaw.motors.forceStop()


if __name__ == "__main__":
    import time
    s = time.perf_counter()
    main()
    elapsed = time.perf_counter() - s
    print(f"{__file__} executed in {elapsed:0.2f} seconds.")