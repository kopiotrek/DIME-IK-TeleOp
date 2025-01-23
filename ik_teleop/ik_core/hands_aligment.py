#!/usr/bin/env python3

import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseArray, Pose
from tf.transformations import quaternion_matrix, quaternion_from_matrix
from xml.etree import ElementTree as ET
import numpy as np
from ik_teleop.teleop_utils.constants import *
from copy import deepcopy as copy
from ik_teleop.teleop_utils.vectorops import *
import time 
ROBOT_JOINTS_RAW = {
    'index': [0, 10, 12, 11],
    'middle': [13, 15, 17, 16],
    'ring': [18, 2, 4, 3],
    'thumb': [6, 7, 9, 8],
    'knuckles': [0, 13, 8],
}
ROBOT_JOINTS_NK = { 
    'index': [0, 1, 2, 3],
    'middle': [4, 5, 6, 7],
    'ring': [8, 9, 10, 11],
    'thumb': [12, 13, 14, 15],
    # 'knuckles': [16, 17, 18, 19],
}

ROBOT_KEYPOINTS_COUNT = 20

class AllegroRetargetingOptimizer:
    def __init__(self):
        rospy.init_node('allegro_retargeting_optimizer')

        rospy.Subscriber('/allegroHand/keypoints', PoseArray, callback=self._get_robot_joints_poses, queue_size=1)

        rospy.Subscriber('/XR/JointPoseArray', PoseArray, callback=self._get_XR_joints_poses, queue_size=1)
        self.pub_mod = rospy.Publisher('/XR/keypoints_transformed', PoseArray, queue_size=10)
        self.pub = rospy.Publisher('/XR/keypoints', PoseArray, queue_size=10)

        self.finger_coords = []
        self.finger_orientations = []
        self.robot_coords = []
        self.robot_coords_array = []
        self.keypoint_translation_array = []
        self.knuckle_points = (OCULUS_JOINTS['knuckles'][3], OCULUS_JOINTS['knuckles'][0])

        rospy.loginfo("Started AllegroRetargetingOptimizer Node")






    def _get_XR_joints_poses(self, msg):
        finger_coords_array = []
        finger_orientation_array = []
        if len(msg.poses) < OCULUS_NUM_KEYPOINTS:
            print("ERROR: not enough joints received")
            return
        for i in range(OCULUS_NUM_KEYPOINTS):
            # finger_poses_array.append(msg.poses[i])
            finger_coords_array.append(self.position_to_array(msg.poses[i].position))     
        self.finger_coords_array_np = np.array(finger_coords_array)
        for i in range(OCULUS_NUM_KEYPOINTS):
            # finger_poses_array.append(msg.poses[i])
            finger_orientation_array.append(self.orientation_to_array(msg.poses[i].orientation)) 
        self.finger_orientation_array_np = np.array(finger_orientation_array)

        self.transform_keypoints()
        self.finger_coords = dict(
            index = self.finger_coords_array_np[OCULUS_JOINTS['index']],
            middle = self.finger_coords_array_np[OCULUS_JOINTS['middle']],
            ring = self.finger_coords_array_np[OCULUS_JOINTS['ring']],
            thumb = self.finger_coords_array_np[OCULUS_JOINTS['thumb']],
            knuckles = self.finger_coords_array_np[OCULUS_JOINTS['knuckles']],
        )
        
        pose_array_msg = PoseArray()
        pose_array_msg.header.stamp = rospy.Time.now()
        pose_array_msg.header.frame_id = "palm_link"
        for i in range(OCULUS_NUM_KEYPOINTS - len(OCULUS_JOINTS['little'])):
            pose = Pose()
            pose.position.x = self.finger_coords_array_np[i][0]
            pose.position.y = self.finger_coords_array_np[i][1]
            pose.position.z = self.finger_coords_array_np[i][2]
            pose.orientation.x = self.finger_orientation_array_np[i][0]
            pose.orientation.y = self.finger_orientation_array_np[i][1]
            pose.orientation.z = self.finger_orientation_array_np[i][2]
            pose.orientation.w = self.finger_orientation_array_np[i][3]
            pose_array_msg.poses.append(pose)
        self.pub.publish(pose_array_msg)



        self.get_keypoint_difference()

        self.align_hand_to_robot()
        # pose_array_msg = PoseArray()
        # pose_array_msg.header.stamp = rospy.Time.now()
        # pose_array_msg.header.frame_id = "palm_link"
        # for i in range(OCULUS_NUM_KEYPOINTS - len(OCULUS_JOINTS['little'])):
        #     pose = Pose()
        #     pose.position.x = self.finger_coords_array_np[i][0]
        #     pose.position.y = self.finger_coords_array_np[i][1]
        #     pose.position.z = self.finger_coords_array_np[i][2]
        #     pose.orientation.x = self.finger_orientation_array_np[i][0]
        #     pose.orientation.y = self.finger_orientation_array_np[i][1]
        #     pose.orientation.z = self.finger_orientation_array_np[i][2]
        #     pose.orientation.w = self.finger_orientation_array_np[i][3]
        #     pose_array_msg.poses.append(pose)
        self.pub_mod.publish(self.create_marker_msg())

    def create_marker_msg(self):
        pose_array_msg = PoseArray()
        pose_array_msg.header.stamp = rospy.Time.now()
        pose_array_msg.header.frame_id = "palm_link"
        for finger in self.finger_coords:
            for joint in range(0, len(self.finger_coords[finger])):
                pose = Pose()
                pose.position.x = self.finger_coords[finger][joint][0]
                pose.position.y = self.finger_coords[finger][joint][1]
                pose.position.z = self.finger_coords[finger][joint][2]
                pose.orientation.x = self.finger_orientations[finger][joint][0]
                pose.orientation.y = self.finger_orientations[finger][joint][1]
                pose.orientation.z = self.finger_orientations[finger][joint][2]
                pose.orientation.w = self.finger_orientations[finger][joint][3]
                pose_array_msg.poses.append(pose)
        return pose_array_msg

    def _translate_coords(self, coords):
        return copy(coords) - coords[12]
        
    def _get_coord_frame(self, wrist_coord, index_knuckle_coord, little_knuckle_coord):
        z_axis = normalize_vector(wrist_coord)
        x_axis = normalize_vector(index_knuckle_coord - little_knuckle_coord)
        y_axis = normalize_vector(np.cross(z_axis, x_axis))
        
        return [y_axis, x_axis, -z_axis] # Change from left-handed unity system to right-handed

    def transform_keypoints(self):
        self.finger_coords_array_np = self._translate_coords(self.finger_coords_array_np)
        original_coord_frame = self._get_coord_frame(
            self.finger_coords_array_np[OCULUS_JOINTS['wrist'][0]],
            self.finger_coords_array_np[OCULUS_JOINTS['knuckles'][0]],
            self.finger_coords_array_np[OCULUS_JOINTS['knuckles'][3]]
        )

        if np.linalg.det(original_coord_frame) == 0:
            rospy.logerr("Original coord frame is singular and cannot be inverted")
            return

        try:
            # Compute the rotation matrix
            rotation_matrix = np.linalg.solve(original_coord_frame, np.eye(3)).T

            # Transform positions
            finger_coords = (rotation_matrix @ self.finger_coords_array_np.T).T

            # Transform orientations
            transformed_orientations = []
            for quaternion in self.finger_orientation_array_np:
                rotation_matrix_quat = quaternion_matrix(quaternion)[:3, :3]
                transformed_matrix = rotation_matrix @ rotation_matrix_quat
                full_transform_matrix = np.eye(4)
                full_transform_matrix[:3, :3] = transformed_matrix
                transformed_quat = quaternion_from_matrix(full_transform_matrix)
                transformed_orientations.append(transformed_quat)

            self.finger_coords_array_np = finger_coords
            self.finger_orientation_array_np = np.array(transformed_orientations)
        except np.linalg.LinAlgError as e:
            rospy.logerr(f"Error computing rotation matrix: {e}")

        # Update finger coordinate dictionary
        self.finger_coords = dict(
            wrist=self.finger_coords_array_np[OCULUS_JOINTS['wrist']],
            # palm=self.finger_coords_array_np[OCULUS_JOINTS['palm']],
            thumb=self.finger_coords_array_np[OCULUS_JOINTS['thumb']],
            index=self.finger_coords_array_np[OCULUS_JOINTS['index']],
            middle=self.finger_coords_array_np[OCULUS_JOINTS['middle']],
            ring=self.finger_coords_array_np[OCULUS_JOINTS['ring']],
            # little=self.finger_coords_array_np[OCULUS_JOINTS['little']],
            # metacarpals=self.finger_coords_array_np[OCULUS_JOINTS['metacarpals']],
            knuckles=self.finger_coords_array_np[OCULUS_JOINTS['knuckles']],
        )
        # Update finger orientation dictionary
        self.finger_orientations = dict(
            wrist=self.finger_orientation_array_np[OCULUS_JOINTS['wrist']],
            # palm=self.finger_orientation_array_np[OCULUS_JOINTS['palm']],
            thumb=self.finger_orientation_array_np[OCULUS_JOINTS['thumb']],
            index=self.finger_orientation_array_np[OCULUS_JOINTS['index']],
            middle=self.finger_orientation_array_np[OCULUS_JOINTS['middle']],
            ring=self.finger_orientation_array_np[OCULUS_JOINTS['ring']],
            # little=self.finger_orientation_array_np[OCULUS_JOINTS['little']],
            # metacarpals=self.finger_orientation_array_np[OCULUS_JOINTS['metacarpals']],
            knuckles=self.finger_orientation_array_np[OCULUS_JOINTS['knuckles']],
        )

        

    def position_to_array(self, position):
        return np.array([position.x, position.y, position.z])

    def orientation_to_array(self, orientation):
        return np.array([orientation.x, orientation.y, orientation.z, orientation.w])

    def _get_robot_joints_poses(self, msg):
        robot_coords_array = []
        for i in range(ROBOT_KEYPOINTS_COUNT):
            robot_coords_array.append(self.position_to_array(msg.poses[i].position))    
        robot_coords_array_np = np.array(robot_coords_array)
        self.robot_coords = dict(
            index = robot_coords_array_np[ROBOT_JOINTS_RAW['index']],
            middle = robot_coords_array_np[ROBOT_JOINTS_RAW['middle']],
            ring = robot_coords_array_np[ROBOT_JOINTS_RAW['ring']],
            thumb = robot_coords_array_np[ROBOT_JOINTS_RAW['thumb']],
            knuckles = robot_coords_array_np[ROBOT_JOINTS_RAW['knuckles']],
        )


    def get_keypoint_difference(self):

        # Create an empty list to store the Euclidean distances between corresponding keypoints
        self.keypoint_difference_array = []
        # Iterate over each finger and calculate the Euclidean distance between corresponding keypoints
        # print(f"self.robot_coords {self.robot_coords}")
        # print(f"self.finger_coords {self.finger_coords}")
        for finger in ROBOT_JOINTS_NK:
            # Get the robot's keypoints for this finger
            robot_keypoints = self.robot_coords[finger]
            # Get the oculus' keypoints for this finger
            oculus_keypoints = self.finger_coords[finger]

            # print(f"\nKeypoint Differences for {finger.capitalize()} Finger:")

            # Ensure both robot_keypoints and oculus_keypoints are arrays (for multiple keypoints in a finger)
            for idx, (r_point, o_point) in enumerate(zip(robot_keypoints, oculus_keypoints)):
                formatted_r_point = np.array([f"{coord:.5f}" for coord in r_point])
                formatted_o_point = np.array([f"{coord:.5f}" for coord in o_point])
                # print(f"  r_point {formatted_r_point} o_point {formatted_o_point}")
            
                # Calculate Euclidean distance between corresponding robot and Oculus keypoints
                translation = r_point - o_point
                self.keypoint_translation_array.append(translation)

                # Print the index and the distance
                # print(f"  Keypoint {idx + 1}: Distance = {translation} meters")

    def align_hand_to_robot(self):
        # Robot finger lengths (in meters)
        robot_finger_lengths = {
            'index': 0.1527,
            'middle': 0.1527,
            'ring': 0.1527,
            'thumb': 0.1308, #0.0363 is real length of servo no.1
        }

        # Calculate scaling factors for each finger based on their lengths
        finger_scales = {}
        for finger in ['index', 'middle', 'ring', 'thumb']:
            # Calculate the total length of the current finger
            total_length = 0.0
            finger_joints = self.finger_coords[finger]
            for i in range(len(finger_joints) - 1):
                link_length = np.linalg.norm(finger_joints[i] - finger_joints[i + 1])
                total_length += link_length

            # Calculate the scaling factor (robot length / XR length)
            scaling_factor = robot_finger_lengths[finger] / total_length
            finger_scales[finger] = scaling_factor
            
            # print(f"Scaling factor for {finger.capitalize()} finger: {scaling_factor:.4f}")

        # Apply the scaling factors and translation to align fingers with the robot's measurements
        translation_array_offset = {'index': 0, 'middle': 4, 'ring': 8, 'thumb': 12}
        for finger in self.finger_coords:
            if finger in translation_array_offset:
                for joint_idx in range(0, len(self.finger_coords[finger])):
                    if joint_idx is not 0:
                        scale = finger_scales[finger]
                        tmp_coord = self.finger_coords[finger][joint_idx] - self.finger_coords[finger][0]
                        tmp_coord *= scale
                        self.finger_coords[finger][joint_idx] = tmp_coord + self.finger_coords[finger][0]
                        self.finger_coords[finger][joint_idx] = tmp_coord + self.finger_coords[finger][0]
                        self.finger_coords[finger][joint_idx] += self.keypoint_translation_array[translation_array_offset[finger]]
                    else:
                        self.finger_coords[finger][joint_idx] += self.keypoint_translation_array[translation_array_offset[finger]]



        for i in range(OCULUS_NUM_KEYPOINTS):
            # if i in OCULUS_JOINTS['index'] and i is not OCULUS_JOINTS['index'][0]:
            #     scale = finger_scales['index']
            #     tmp_coord = self.finger_coords_array_np[i] - self.finger_coords_array_np[OCULUS_JOINTS['index'][0]]
            #     tmp_coord *= scale
            #     self.finger_coords_array_np[i] = tmp_coord + self.finger_coords_array_np[OCULUS_JOINTS['index'][0]]
            #     self.finger_coords_array_np[i] += self.keypoint_translation_array[0]
            # if i is OCULUS_JOINTS['index'][0]:
            #     self.finger_coords_array_np[i] += self.keypoint_translation_array[0]

            if i in OCULUS_JOINTS['middle'] and i is not OCULUS_JOINTS['middle'][0]:
                scale = finger_scales['middle']
                tmp_coord = self.finger_coords_array_np[i] - self.finger_coords_array_np[OCULUS_JOINTS['middle'][0]]
                tmp_coord *= scale
                self.finger_coords_array_np[i] = tmp_coord + self.finger_coords_array_np[OCULUS_JOINTS['middle'][0]]
                self.finger_coords_array_np[i] += self.keypoint_translation_array[4]
            elif i is OCULUS_JOINTS['middle'][0]:
                self.finger_coords_array_np[i] += self.keypoint_translation_array[4]

            elif i in OCULUS_JOINTS['ring'] and i is not OCULUS_JOINTS['ring'][0]:
                scale = finger_scales['ring']
                tmp_coord = self.finger_coords_array_np[i] - self.finger_coords_array_np[OCULUS_JOINTS['ring'][0]]
                tmp_coord *= scale
                self.finger_coords_array_np[i] = tmp_coord + self.finger_coords_array_np[OCULUS_JOINTS['ring'][0]]
                self.finger_coords_array_np[i] += self.keypoint_translation_array[8]
            elif i is OCULUS_JOINTS['ring'][0]:
                self.finger_coords_array_np[i] += self.keypoint_translation_array[8]

            elif i in OCULUS_JOINTS['thumb'] and i is not OCULUS_JOINTS['thumb'][0]:
                scale = finger_scales['thumb']
                tmp_coord = self.finger_coords_array_np[i] - self.finger_coords_array_np[OCULUS_JOINTS['thumb'][0]]
                tmp_coord *= scale
                self.finger_coords_array_np[i] = tmp_coord + self.finger_coords_array_np[OCULUS_JOINTS['thumb'][0]]
                self.finger_coords_array_np[i] += self.keypoint_translation_array[12]
            elif i is OCULUS_JOINTS['thumb'][0]:
                self.finger_coords_array_np[i] += self.keypoint_translation_array[12]


        self.keypoint_translation_array = []

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    listener = AllegroRetargetingOptimizer()
    listener.run()