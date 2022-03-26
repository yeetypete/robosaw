from Model import Model
import robo_vision2 as rv
import time
import argparse
import os
import cv2
from Run import Run
from CountsPerSec import CountsPerSec
from VideoGet import VideoGet
from VideoShow import VideoShow

def putIterationsPerSec(frame, iterations_per_sec):
    """
    Add iterations per second text to lower-left corner of a frame.
    """

    cv2.putText(frame, "{:.0f} iterations/sec".format(iterations_per_sec),
        (10, 450), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255))
    return frame

def threadVideoGet(source=0):
    """
    Dedicated thread for grabbing video frames with VideoGet object.
    Main thread shows video frames.
    """

    video_getter = VideoGet(source).start()
    cps = CountsPerSec().start()

    while True:
        if (cv2.waitKey(1) == ord("q")) or video_getter.stopped:
            video_getter.stop()
            break

        frame = video_getter.frame
        frame = putIterationsPerSec(frame, cps.countsPerSec())
        cv2.imshow("Video", frame)
        cps.increment()

def noThreading(source=0):
    """Grab and show video frames without multithreading."""

    cap = cv2.VideoCapture(source)
    cps = CountsPerSec().start()

    while True:
        grabbed, frame = cap.read()
        if not grabbed or cv2.waitKey(1) == ord("q"):
            break

        frame = putIterationsPerSec(frame, cps.countsPerSec())
        cv2.imshow("Video", frame)
        cps.increment()


def threadBoth(model,source=0):
    """
    Dedicated thread for grabbing video frames with VideoGet object.
    Dedicated thread for showing video frames with VideoShow object.
    Main thread serves only to pass frames between VideoGet and
    VideoShow objects/threads.
    """
    #source = caps[2]
    video_getter = VideoGet(source).start()
    video_shower = VideoShow(video_getter.frame).start()
    cps = CountsPerSec().start()
    runner = Run(model,video_getter.frame).start()

    while True:
        if video_getter.stopped or video_shower.stopped:
            video_shower.stop()
            video_getter.stop()
            runner.stop()
            break

        frame = video_getter.frame
        frame = putIterationsPerSec(frame, cps.countsPerSec())
        video_shower.frame = frame
        runner.frame = frame
        
        cps.increment()


def threadVideoShow(model,caps):
    """
    Dedicated thread for showing video frames with VideoShow object.
    Main thread grabs video frames.
    """

    cap = caps[2]
    (grabbed, frame) = cap.read()
    video_shower = VideoShow(frame).start()
    cps = CountsPerSec().start()
    runner = Run(model,frame).start()
    while True:
        (grabbed, frame) = cap.read()
        if not grabbed or video_shower.stopped:
            video_shower.stop()
            runner.stop()
            break

        frame = putIterationsPerSec(frame, cps.countsPerSec())
        video_shower.frame = frame
        runner.frame = frame
        cps.increment()
        

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", "-s", default=0,
        help="Path to video file or integer representing webcam index"
            + " (default 0).")
    ap.add_argument("--thread", "-t", default="none",
        help="Threading mode: get (video read in its own thread),"
            + " show (video show in its own thread), both"
            + " (video read and video show in their own threads),"
            + " none (default--no multithreading)")
    args = vars(ap.parse_args())

    # If source is a string consisting only of integers, check that it doesn't
    # refer to a file. If it doesn't, assume it's an integer camera ID and
    # convert to int.
    if (
        isinstance(args["source"], str)
        and args["source"].isdigit()
        and not os.path.isfile(args["source"])
    ):
        args["source"] = int(args["source"])

    if args["thread"] == "both":
        threadBoth(args["source"])
    elif args["thread"] == "get":
        threadVideoGet(args["source"])
    elif args["thread"] == "show":
        threadVideoShow(args["source"])
    else:
        noThreading(args["source"])


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
    print("Releasing capture")
    caps[2].release()



def run(model,frame):
    """ Intake at idle speed until wood is detected, 
    find the angle of the line, 
    rotate blade to match angle, 
    move line under blade and center it, 
    wait for confirmation button,
    make cut, raise blade """
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
    



if __name__ == "__main__":
    s = time.perf_counter()
    #eject(model,caps) # Runs the feeding mechanism as long as 'feed' button is pressed
    model,caps = initialize()
    #threadVideoShow(model,caps)
    threadBoth(model,caps)
    #run(model,caps) # Once 'Run' button pressed, waits for wood, finds angle, centers under the blade and makes the cut
    close_caps(caps)

    #main()

    elapsed = time.perf_counter() - s
    print(f"{__file__} executed in {elapsed:0.2f} seconds.")