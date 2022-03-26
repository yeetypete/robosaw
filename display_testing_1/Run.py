from threading import Thread
import cv2
import robo_vision2 as rv

class Run:
    """
    Class that runs RoboVision in a seperate thread.
    """

    def __init__(self, model, frame=None):
        self.frame = frame
        self.model = model
        self.stopped = False

    def start(self):
        Thread(target=self.show, args=()).start()
        return self

    def show(self):
        while not self.stopped:
            cv2.imshow("Video Run", self.frame)
            if cv2.waitKey(1) == ord("q"):
                self.stopped = True

    def run(self):
        """ Intake at idle speed until wood is detected, 
        find the angle of the line, 
        rotate blade to match angle, 
        move line under blade and center it, 
        wait for confirmation button,
        make cut, raise blade """
        frame = self.frame
        model = self.model
        while not self.stopped:
            dist = rv.find_distance(model,frame)
            print(str(dist))
            if (dist is not None):
                print("Distance: " + str(dist))
                print("Close")
                if (dist < 100):
                    #robosaw.motors.setSpeeds(100,100)
                    if (dist <= 0):
                        print("Distance: " + str(dist))
                        print("Center")
                        

    def stop(self):
        self.stopped = True