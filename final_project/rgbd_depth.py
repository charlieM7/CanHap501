import cv2
import pyrealsense2 as rs
import numpy as np
import os

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

try:
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        colour_frame = frames.get_color_frame()

        if not depth_frame or not colour_frame:
            continue

        # convert depth frame to numpy array
        depth_img = np.asanyarray(depth_frame.get_data())
        colour_img = np.asanyarray(colour_frame.get_data())

        # apply colormap
        colourmap = cv2.applyColorMap(cv2.convertScaleAbs(depth_img, alpha=0.2), cv2.COLORMAP_JET)
        corrected_colour = np.clip((colourmap / 255.0) ** 0.5 * 255.0, 0, 255).astype(np.uint8)
        grayscale_img = cv2.cvtColor(corrected_colour, cv2.COLOR_BGR2GRAY)

        # apply smoothing
        depth_smoothed = cv2.GaussianBlur(corrected_colour, (3, 3), 0)
        cv2.imshow("Depth Image", depth_smoothed)

        #normalize grayscale for texture 
        norm_img = (grayscale_img - grayscale_img.min()) / (grayscale_img.max() - grayscale_img.min())
        
        # create folder
        save_folder = "depth_images"
        if not os.path.exists(save_folder):
            os.makedirs(save_folder)

        # save images
        cv2.imwrite(os.path.join(save_folder, "depth_image.png"), depth_smoothed)
        cv2.imwrite(os.path.join(save_folder, "regular_image.png"), colour_img)
        cv2.imwrite(os.path.join(save_folder, "grayscale_image.png"), grayscale_img)

        # exit with q key press 
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()
