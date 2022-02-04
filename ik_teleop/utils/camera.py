# Standard imports
import numpy as np

# Image based imports
import cv2
import pyrealsense2 as rs

def create_realsense_pipeline(camera_serial_number, resolution, fps):
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(camera_serial_number)
    config.enable_stream(rs.stream.color, resolution[0], resolution[1], rs.format.bgr8, fps)

    return pipeline, config

def create_realsense_rgb_depth_pipeline(camera_serial_number, resolution, fps):
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(camera_serial_number)
    config.enable_stream(rs.stream.color, resolution[0], resolution[1], rs.format.bgr8, fps)
    config.enable_stream(rs.stream.depth, resolution[0], resolution[1], rs.format.z16, fps)

    return pipeline, config

def getting_image_data(pipeline):
    # Creating a frame object to get the RGB data
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()

    # Storing the frame data as an image
    image = np.asanyarray(color_frame.get_data())

    # Checking if the frame produced an image or not
    if image is None:
        print('Ignoring empty camera frame!')
        return None

    # Converting the image from BGR to RGB
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    return image

def getting_depth_data(pipeline):
    # Creating a frame object to get the Depth data
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()

    # Storing the frame data as an image
    depth_image = np.asanyarray(depth_frame.get_data())

    # Checking if the frame produced an image or not
    if depth_image is None:
        print('Ignoring empty camera frame!')
        return None

    return depth_image

def rotate_image(image, angle):
    if angle == 90:
        image = cv2.rotate(image, cv2.ROTATE_90)
    elif angle == 180:
        image = cv2.rotate(image, cv2.ROTATE_180)
    elif angle == 270:
        image = cv2.rotate(image, cv2.ROTATE_270)

    return image    

def create_contours(image, circle, knuckle_bound, thumb_bound, thickness):
    # Creating the wrist bound
    image = cv2.circle(image, (circle.center[0], circle.center[1]), circle.radius, (255, 0, 0), thickness)

    # Creating the pinky knuckle bound
    image = cv2.drawContours(image, [np.array(knuckle_bound)], 0, (0, 0, 255), thickness)

    # Creating the thumb tip bound
    image = cv2.drawContours(image, [np.array(thumb_bound)], 0, (0, 255, 0), thickness)

    return image

