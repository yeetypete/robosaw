
def main():
        """ Main fn for testing robo vision v2 using multiple cameras """

        # Initialize Model object 'model'


        # Initialize the camera captures


        # Check if wood is loaded


        # Find the angle


        # Find the distance of the line from the blade's plane of intersection


if __name__ == "__main__":
    import time
    s = time.perf_counter()
    main()
    elapsed = time.perf_counter() - s
    print(f"{__file__} executed in {elapsed:0.2f} seconds.")