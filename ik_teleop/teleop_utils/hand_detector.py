import numpy as np

import mediapipe
try:
    import rospy
except ModuleNotFoundError:
    print('Module rospy not found!\n'\
    'Check packages if you are using MediapipeJointsPublisher')
try:
    from std_msgs.msg import Float64MultiArray
except ModuleNotFoundError:
    print('Module std_msgs.msg NOT FOUND!\n'\
    'Check packages if using MediapipeJointsPublisher.'\
    'Ignore if using MediapipeJoints.')

import ik_teleop.utils.camera as camera
import ik_teleop.utils.joint_handling as joint_handlers

from hydra import initialize, compose
import cv2
import signal
import sys
import os
from datetime import datetime


POSE_COORD_TOPIC = '/mediapipe_joint_coords'
MOVING_AVERAGE_LIMIT = 10

class MediapipeJoints(object):
    def __init__(self, cfg = None, rotation_angle = 0, moving_average = True, normalize = True, cam_serial_num = None, record_demo = False):
        # Getting the configurations
        if cfg is None:
            initialize(config_path = "../parameters/")
            self.cfg = compose(config_name = "teleop")
        else:
            self.cfg = cfg

        # Creating a realsense pipeline
        if(cam_serial_num):
            #TODO use cfg file instead of bypassing
            self.pipeline, config = camera.create_realsense_pipeline(cam_serial_num, self.cfg.realsense.resolution, self.cfg.realsense.fps)
        else:
            self.pipeline, config = camera.create_realsense_pipeline(self.cfg.realsense.serial_numbers[0], self.cfg.realsense.resolution, self.cfg.realsense.fps)

        self.pipeline.start(config)

        self.rotation_angle = rotation_angle
        self.normalize = normalize

        # Creating mediapipe objects
        self.mediapipe_drawing = mediapipe.solutions.drawing_utils
        self.mediapipe_hands = mediapipe.solutions.hands

        self.moving_average = moving_average
        if self.moving_average is True:
            self.moving_average_queue = []
        self.queue = None

        self.record_demo = record_demo
        self.vid_file = ''
        self.unmrkd_file = ''

        if(self.record_demo):
            t= datetime.now()
            date_str = t.strftime('%b_%d_%H_%M')
            self.demo_dir = os.path.join('demos',"demo_{}".format(date_str))
            if(self.record_demo and not os.path.isdir(self.demo_dir)):
                os.mkdir(self.demo_dir)
            self.vid_file = os.path.join(self.demo_dir, 'demo.mp4')
            self.unmrkd_file = os.path.join(self.demo_dir,'orig.mp4')

        self.unmrkd_images = []
        self.images = []

        # if sys.argv[1] == "handle_signal":
        # signal.signal(signal.SIGTERM, self.finish_recording)

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

    def publish_coords(self, coords):
        if(self.queue is not None):
            self.queue.put(coords)
        return coords

    def detect(self):
        # Setting the mediapipe hand parameters
        with self.mediapipe_hands.Hands(
            max_num_hands = 1, # Limiting the number of hands detected in the image to 1
            min_detection_confidence = 0.95,
            min_tracking_confidence = 0.95) as hand:

            while True:
                # Getting the image to process
                image = camera.getting_image_data(self.pipeline)

                if image is None:
                    print('Did not receive an image. Please wait!')
                    continue

                # Rotate image if needed
                if self.rotation_angle != 0:
                    image = camera.rotate_image(image, self.rotation_angle)
    
                # Getting the hand pose results out of the image
                image.flags.writeable = False
                estimate = hand.process(image)

                # If there is a mediapipe hand estimate
                if estimate.multi_hand_landmarks is not None:  
                    if(self.record_demo):
                        self.unmrkd_images.append(image)
                        # self.images.append(image)
                    # Getting the hand coordinate values for the only detected hand
                    hand_landmarks = estimate.multi_hand_landmarks[0]

                    # Obtaining the joint coordinate estimates from Mediapipe
                    wrist_position, thumb_knuckle_position, index_knuckle_position, middle_knuckle_position, ring_knuckle_position, pinky_knuckle_position, finger_tip_positions = joint_handlers.get_joint_positions(hand_landmarks, self.cfg.realsense.resolution, self.cfg.mediapipe)

                    # Transforming the coordinates 
                    transformed_coords = self.transform_coords(wrist_position, thumb_knuckle_position, index_knuckle_position, middle_knuckle_position, ring_knuckle_position, pinky_knuckle_position, finger_tip_positions)
                    
                    if self.moving_average is True:
                        self.moving_average_queue.append(transformed_coords)

                        if len(self.moving_average_queue) > MOVING_AVERAGE_LIMIT:
                            self.moving_average_queue.pop(0)

                        mean_transformed_value = np.mean(self.moving_average_queue, axis = 0)
                        self.publish_coords(mean_transformed_value)

                    else:
                        # Publishing the transformed coordinates
                        self.publish_coords(transformed_coords)

    def finish_recording(self):
        if(self.record_demo):
            vid_writer = cv2.VideoWriter(self.vid_file,cv2.VideoWriter_fourcc(*'mp4v'), self.cfg.realsense.fps, self.cfg.realsense.resolution)
            for im in self.images:
                vid_writer.write(im)
            vid_writer.release

            uvid_writer = cv2.VideoWriter(self.unmrkd_file,cv2.VideoWriter_fourcc(*'mp4v'), self.cfg.realsense.fps, self.cfg.realsense.resolution)
            for im in self.unmrkd_images:
                uvid_writer.write(im)
            uvid_writer.release()
            sys.exit(0)


class MediapipeJointPublisher(MediapipeJoints):
    def __init__(self, cfg = None, rotation_angle = 0, moving_average = True, normalize = True):
        super().__init(cfg,rotation_angle,moving_average,normalize)
        try:
            rospy.init_node('teleop_camera')
        except:
            pass
        self.publisher = rospy.Publisher(POSE_COORD_TOPIC, Float64MultiArray, queue_size = 1)

    def publish_coords(self, coords):
        coords_to_publish = Float64MultiArray()

        data = []
        for coordinate in coords:
            for ax in coordinate:
                data.append(float(ax))

        coords_to_publish.data = data
        self.publisher.publish(coords_to_publish)