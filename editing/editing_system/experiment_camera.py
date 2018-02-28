import cv2

def read_one_frame(camera):
    grabbed, frame = camera.read()
    if grabbed:
        return frame
    else:
        return None


camera = cv2.VideoCapture('./test/429.mp4')
frame = read_one_frame(camera)
counter = 0
while frame is not None:
    frame = read_one_frame(camera)
    if frame is not None:
        print "Got one frame"
    counter += 1

