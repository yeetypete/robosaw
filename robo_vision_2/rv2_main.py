from Model import Model
import robo_vision2 as rv
import cv2

def main():
        """ Main fn for testing robo vision v2 using multiple cameras """
        ### constants ####
        MAX_ANGLE = 50
        ##################

            # Initialize Model object 'model'
        model = Model(MAX_ANGLE)

            # Initialize the camera captures
        caps = rv.open_cameras(model)

            # Check if wood is loaded
        #rv.display_color_cap(model,caps[0])
        wood_loaded = False
        while not wood_loaded: # wait for the wood
            wood_loaded = rv.wood_is_loaded(model,caps[0])
        caps[0].release()

        # Once wood is loaded accumlate angle samples
        angles = []
        while len(angles) < model.num_angle_samples:
            # Get angles as it moves and save to angle[] array
            #robosaw.motors.setSpeeds(args.speed, args.speed) # idle
            angle = rv.find_angle(model,caps[1])
            if angles is not None:
                angles.append(angle)
        # find most likely angle based on removing outliers and taking the mean
        blade_angle = model.best_angle(angles)
        print("\n\nBest angle:" + str(blade_angle))
        caps[1].release()
            # Find the distance of the line from the blade's plane of intersection
        #rv.display_center_cap(model,caps[2])
        rv.find_center_display(model,caps[2])
        while True:
            dist = rv.find_distance(model,caps[2])
            # If the user presses ESC then exit the program
            key = cv2.waitKey(1)
            if key == 27:
                break
            if (dist is not None):
                dist = -dist
                #robosaw.feed(dist, args.speed)
                print("Distance: " + str(dist))

            # Close the captures and windows before terminating!
        cv2.destroyAllWindows()
        for cap in caps:
            print("\nReleasing: " + str(cap) + "...\n")
            cap.release()

if __name__ == "__main__":
    import time
    s = time.perf_counter()
    main()
    elapsed = time.perf_counter() - s
    print(f"{__file__} executed in {elapsed:0.2f} seconds.")