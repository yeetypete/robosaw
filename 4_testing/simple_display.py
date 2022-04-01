import cv2

logo = cv2.imread(cv2.samples.findFile("RoboSaw_logo.jpg"))

vc = cv2.VideoCapture(0)

if vc.isOpened(): # try to get the first frame
    rval, frame = vc.read()
else:
    rval = False

while rval:
    cv2.namedWindow("preview",cv2.WINDOW_NORMAL)
    cv2.resizeWindow("preview", 600,400)
    width = int(frame.shape[1])
    height = int(frame.shape[0])
    dim = (width, height)
  
    # resize image
    resized = cv2.resize(logo, dim, interpolation = cv2.INTER_AREA)
    added_image = cv2.addWeighted(frame,0.5,resized,0.15,0)

    
    cv2.imshow("preview", added_image)
    rval, frame = vc.read()
    key = cv2.waitKey(20)
    if key == 27: # exit on ESC
        break

cv2.destroyWindow("preview")
vc.release()
