import gradient

base_address = "/Users/frankshammer42/Documents/CS/autonomous-video-artist/editing/editing_system/integrated_test/clips/"
vid_name = "2018_5_5_13-56-2.mp4"
vid_address = base_address + vid_name

res1 = 640
res2 = 480
out_put = "gradient_debug.mp4"

gradient.produce_gradient_video(vid_address, out_put, 30, res1, res2)
