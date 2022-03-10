from Model import Model
import robo_vision2 as rv

def main():
        """ Main fn for testing robo vision v2 using multiple cameras """

        # Initialize Model object 'model'
        model = Model(45)

        # Initialize the camera captures
        caps = rv.open_cameras(model)

        # Check if wood is loaded
        #rv.display_cap(model,caps[0])

        # Find the angle
        print(str(model.top_angle_cam))
        #rv.display_cap(model,caps[1])
        rv.find_angle_display(model,caps[2])

        # Find the distance of the line from the blade's plane of intersection
        #rv.display_cap(model,caps[2])

if __name__ == "__main__":
    import time
    s = time.perf_counter()
    main()
    elapsed = time.perf_counter() - s
    print(f"{__file__} executed in {elapsed:0.2f} seconds.")