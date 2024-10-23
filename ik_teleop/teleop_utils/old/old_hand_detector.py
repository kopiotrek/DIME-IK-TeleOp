import time
import numpy as np

import mediapipe

import cv2

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import ik_teleop.utils.camera as camera
import ik_teleop.utils.joint_handling as joint_handlers

from hydra import initialize, compose

ABSOLUTE_POSE_COORD_TOPIC = '/absolute_mediapipe_joint_pixels'
MEDIAPIPE_RGB_IMG_TOPIC = '/mediapipe_rgb_image'
TRANFORMED_POSE_COORD_TOPIC = '/transformed_mediapipe_joint_coords'

MOVING_AVERAGE_LIMIT = 2

class MediapipeJoints(object):
    def __init__(self, display_image = True, cfg = None, rotation_angle = 0, moving_average = True, normalize = True):
        try:
            rospy.init_node('teleop_camera')
        except:
            pass

        # Getting the configurations
        if cfg is None:
            initialize(config_path = "../parameters/")
            self.cfg = compose(config_name = "teleop")
        else:
            self.cfg = cfg

        # Creating a realsense pipeline
        self.pipeline, config = camera.create_realsense_rgb_depth_pipeline(self.cfg.realsense.serial_numbers[0], self.cfg.realsense.resolution, self.cfg.realsense.fps)

        self.pipeline.start(config)

        self.rotation_angle = rotation_angle
        self.normalize = normalize

        # Creating mediapipe objects
        self.mediapipe_drawing = mediapipe.solutions.drawing_utils
        self.mediapipe_hands = mediapipe.solutions.hands

        self.absolute_coord_publisher = rospy.Publisher(ABSOLUTE_POSE_COORD_TOPIC, Float64MultiArray, queue_size = 1)
        self.trans_coord_publisher = rospy.Publisher(TRANFORMED_POSE_COORD_TOPIC, Float64MultiArray, queue_size = 1)

        self.rgb_image_publisher = rospy.Publisher(MEDIAPIPE_RGB_IMG_TOPIC, Image, queue_size = 1)

        self.bridge = CvBridge()

        self.display_image = display_image

        self.moving_average = moving_average
        if self.moving_average is True:
            self.moving_average_queue = []

    def transform_coords(self, wrist_position, thumb_knuckle_position, index_knuckle_position, middle_knuckle_position, ring_knuckle_position, pinky_knuckle_position, finger_tip_coords, mirror_points = True):
        joint_coords = np.vstack([
            wrist_position, 
            thumb_knuckle_position,
            index_knuckle_position, 
            middle_knuckle_position, 
            ring_knuckle_position,
            pinky_knuckle_position, 
            np.array([finger_tip_coords[key] for key in finger_tip_coords.keys()])
        ])

        # Adding the z values
        z_values = np.zeros((joint_coords.shape[0], 1))
        joint_coords = np.append(joint_coords, z_values, axis = 1)

        # Subtract all the coords with the wrist position to ignore the translation
        translated_joint_coords = joint_coords - joint_coords[0]

        # Finding the 3D direction vector and getting the cross product for X axis
        if self.normalize is True:
            direction_vector = translated_joint_coords[3]
            normal_vector = np.array([0, 0, np.linalg.norm(translated_joint_coords[3])])
            cross_product = np.cross(direction_vector / np.linalg.norm(translated_joint_coords[3]), normal_vector / np.linalg.norm(translated_joint_coords[3])) * np.linalg.norm(translated_joint_coords[3])
        else:
            direction_vector = translated_joint_coords[3] / np.linalg.norm(translated_joint_coords[3])
            normal_vector = np.array([0, 0, 1])
            cross_product = np.cross(direction_vector, normal_vector)

        original_coord_frame = [cross_product, direction_vector, normal_vector]

        # Finding the translation matrix to rotate the values
        rotation_matrix = np.linalg.solve(original_coord_frame, np.eye(3)).T
        transformed_hand_coords = (rotation_matrix @ translated_joint_coords.T).T

        if mirror_points is True:
            transformed_hand_coords[:, 0] = -transformed_hand_coords[:, 0]

        # Returning only the 2D coordinates   
        return transformed_hand_coords[:, :2]

    def get_absolute_coords(self, wrist_position, thumb_knuckle_position, index_knuckle_position, middle_knuckle_position, ring_knuckle_position, pinky_knuckle_position, finger_tip_coords, mirror_points = False):
        joint_coords = np.vstack([
            wrist_position, 
            thumb_knuckle_position,
            index_knuckle_position, 
            middle_knuckle_position, 
            ring_knuckle_position,
            pinky_knuckle_position, 
            np.array([finger_tip_coords[key] for key in finger_tip_coords.keys()])
        ])

        if mirror_points is True:
            joint_coords[:, 0] = self.cfg.realsense.resolution[0] - joint_coords[:, 0]

        return joint_coords

    def publish_transformed_coords(self, coords):
        coords_to_publish = Float64MultiArray()

        data = []
        for coordinate in coords:
            for ax in coordinate:
                data.append(float(ax))

        coords_to_publish.data = data
        self.trans_coord_publisher.publish(coords_to_publish)

    def publish_absolute_coords(self, coords):
        coords_to_publish = Float64MultiArray()

        data = []
        for coordinate in coords:
            for ax in coordinate:
                data.append(float(ax))

        coords_to_publish.data = data
        self.absolute_coord_publisher.publish(coords_to_publish)

    def publish_rgb_image(self, rgb_image):
        try:
            rgb_image = self.bridge.cv2_to_imgmsg(rgb_image, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.rgb_image_publisher.publish(rgb_image)

    def detect(self):
        # Setting the mediapipe hand parameters
        with self.mediapipe_hands.Hands(
            max_num_hands = 1, # Limiting the number of hands detected in the image to 1
            min_detection_confidence = 0.95,
            min_tracking_confidence = 0.95) as hand:

            while True:
                start = time.time()

                # Getting the image to process
                rgb_image = camera.getting_image_data(self.pipeline)

                if rgb_image is None:
                    print('Did not receive an image. Please wait!')
                    continue

                # Rotate image if needed
                if self.rotation_angle != 0:
                    rgb_image = camera.rotate_image(rgb_image, self.rotation_angle)
    
                # Getting the hand pose results out of the image
                rgb_image.flags.writeable = False
                estimate = hand.process(rgb_image)

                # If there is a mediapipe hand estimate
                if estimate.multi_hand_landmarks is not None:  

                    # Getting the hand coordinate values for the only detected hand
                    hand_landmarks = estimate.multi_hand_landmarks[0]

                    # Obtaining the joint coordinate estimates from Mediapipe
                    wrist_position, thumb_knuckle_position, index_knuckle_position, middle_knuckle_position, ring_knuckle_position, pinky_knuckle_position, finger_tip_positions = joint_handlers.get_joint_positions(hand_landmarks, self.cfg.realsense.resolution, self.cfg.mediapipe)

                    # Transforming the coordinates 
                    transformed_coords = self.transform_coords(wrist_position, thumb_knuckle_position, index_knuckle_position, middle_knuckle_position, ring_knuckle_position, pinky_knuckle_position, finger_tip_positions)
                    
                    # Also getting the absolute coordinates
                    absolute_coordinates = self.get_absolute_coords(wrist_position, thumb_knuckle_position, index_knuckle_position, middle_knuckle_position, ring_knuckle_position, pinky_knuckle_position, finger_tip_positions)
                    self.publish_absolute_coords(absolute_coordinates)

                    if self.moving_average is True:
                        self.moving_average_queue.append(transformed_coords)

                        if len(self.moving_average_queue) > MOVING_AVERAGE_LIMIT:
                            self.moving_average_queue.pop(0)

                        mean_transformed_value = np.mean(self.moving_average_queue, axis = 0)
                        self.publish_transformed_coords(mean_transformed_value)


                    else:
                        # Publishing the transformed coordinates
                        self.publish_transformed_coords(transformed_coords)

                    # Publishing the rgb and depth image data
                    rgb_img_pub_start_time = time.time()
                    self.publish_rgb_image(rgb_image)
                    rgb_img_pub_end_time = time.time()

                if self.display_image:

                    cv2.imshow('MediaPipe Hands', cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB))
                    if cv2.waitKey(5) & 0xFF == 27:
                        break
