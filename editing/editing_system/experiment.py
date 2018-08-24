import cv2
import editing

def get_length():
    vid_path = "/Users/frankshammer42/Documents/CS/autonomous-video-artist/editing/editing_system/editing_result/2018_5_5_13-56-2.mp4"
    vid_generator = editing.load_generator(vid_path)
    counter = 0
    length = int(vid_generator.get(cv2.CAP_PROP_FRAME_COUNT))
    while True:
        grabbed, frame = vid_generator.read()
        if grabbed:
            counter += 1
        else:
            break



get_length()





