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
ROBOT_JOINTS = {
    'index': [11, 0, 1, 2],
    'middle': [4, 5, 6, 7],
    'ring': [9, 12, 13, 14],
    'thumb': [16, 17, 18, 19],
    'knuckles': [10, 3, 8, 15],
}
ROBOT_KEYPOINTS_COUNT = 20

class AllegroRetargetingOptimizer:
    def __init__(self):
        rospy.init_node('allegro_retargeting_optimizer')

        rospy.Subscriber('/allegroHand/keypoints', PoseArray, callback=self._get_robot_joints_poses, queue_size=1)
        # Data order in /allegroHand/keypoints
        # 0  link_2
        # 1  link_3
        # 2  link_3_tip
        # 3  link_4
        # 4  link_5
        # 5  link_6
        # 6  link_7
        # 7  link_7_tip
        # 8  link_8
        # 9  link_9
        # 10 link_0
        # 11 link_1
        # 12 link_10
        # 13 link_11
        # 14 link_11_tip
        # 15 link_12
        # 16 link_13
        # 17 link_14
        # 18 link_15
        # 19 link_15_tip


        rospy.Subscriber('/XR/JointPoseArray', PoseArray, callback=self._get_XR_joints_poses, queue_size=1)
        self.pub = rospy.Publisher('/XR/keypoints_transformed', PoseArray, queue_size=10)

        self.finger_coords = []
        self.finger_orientations = []
        self.robot_coords = []
        self.robot_coords_array = []

        self.knuckle_points = (OCULUS_JOINTS['knuckles'][3], OCULUS_JOINTS['knuckles'][0])

        rospy.loginfo("Started AllegroRetargetingOptimizer Node")

    def align_hand_to_robot(self):
        for i in range(OCULUS_NUM_KEYPOINTS):

            if i in OCULUS_JOINTS['index']:
                self.finger_coords_array_np[i]*=1.65
                self.finger_coords_array_np[i][0]-=0.015
                self.finger_coords_array_np[i][1]+=0.005
                self.finger_coords_array_np[i][2]+=0.009

            elif i in OCULUS_JOINTS['middle']:
                self.finger_coords_array_np[i]*=1.43
                self.finger_coords_array_np[i][0]-=0.002
                self.finger_coords_array_np[i][2]+=0.001

            elif i in OCULUS_JOINTS['ring']:
                self.finger_coords_array_np[i]*=1.47
                self.finger_coords_array_np[i][0]-=0.005
                self.finger_coords_array_np[i][1]-=0.012

            elif i in OCULUS_JOINTS['thumb']:
                self.finger_coords_array_np[i]*=1.3
                self.finger_coords_array_np[i][0]-=0.02




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
        self.align_hand_to_robot()
        pose_array_msg = PoseArray()
        pose_array_msg.header.stamp = rospy.Time.now()
        pose_array_msg.header.frame_id = "palm_link"
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
        
        # self.get_keypoint_difference()
    
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
            print(f"self.finger_orientation_array_np {self.finger_orientation_array_np}")
        except np.linalg.LinAlgError as e:
            rospy.logerr(f"Error computing rotation matrix: {e}")

        # Update finger coordinate dictionary
        self.finger_coords = dict(
            wrist=self.finger_coords_array_np[OCULUS_JOINTS['wrist']],
            palm=self.finger_coords_array_np[OCULUS_JOINTS['palm']],
            thumb=self.finger_coords_array_np[OCULUS_JOINTS['thumb']],
            index=self.finger_coords_array_np[OCULUS_JOINTS['index']],
            middle=self.finger_coords_array_np[OCULUS_JOINTS['middle']],
            ring=self.finger_coords_array_np[OCULUS_JOINTS['ring']],
            little=self.finger_coords_array_np[OCULUS_JOINTS['little']],
            metacarpals=self.finger_coords_array_np[OCULUS_JOINTS['metacarpals']],
            knuckles=self.finger_coords_array_np[OCULUS_JOINTS['knuckles']],
        )
        # Update finger orientation dictionary
        self.finger_orientations = dict(
            wrist=self.finger_orientation_array_np[OCULUS_JOINTS['wrist']],
            palm=self.finger_orientation_array_np[OCULUS_JOINTS['palm']],
            thumb=self.finger_orientation_array_np[OCULUS_JOINTS['thumb']],
            index=self.finger_orientation_array_np[OCULUS_JOINTS['index']],
            middle=self.finger_orientation_array_np[OCULUS_JOINTS['middle']],
            ring=self.finger_orientation_array_np[OCULUS_JOINTS['ring']],
            little=self.finger_orientation_array_np[OCULUS_JOINTS['little']],
            metacarpals=self.finger_orientation_array_np[OCULUS_JOINTS['metacarpals']],
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
            index = robot_coords_array_np[ROBOT_JOINTS['index']],
            middle = robot_coords_array_np[ROBOT_JOINTS['middle']],
            ring = robot_coords_array_np[ROBOT_JOINTS['ring']],
            thumb = robot_coords_array_np[ROBOT_JOINTS['thumb']],
            knuckles = robot_coords_array_np[ROBOT_JOINTS['knuckles']],
        )

    def get_keypoint_difference(self):
        # Create an empty list to store the Euclidean distances between corresponding keypoints
        self.keypoint_difference_array = []

        # Iterate over each finger and calculate the Euclidean distance between corresponding keypoints
        for finger in ROBOT_JOINTS:
            # Get the robot's keypoints for this finger
            robot_keypoints = self.robot_coords[finger]
            # Get the oculus' keypoints for this finger
            oculus_keypoints = self.finger_coords[finger]

            # Ensure both robot_keypoints and oculus_keypoints are arrays (for multiple keypoints in a finger)
            for r_point, o_point in zip(robot_keypoints, oculus_keypoints):
                # Calculate Euclidean distance between corresponding robot and Oculus keypoints
                distance = np.linalg.norm(r_point - o_point)
                self.keypoint_difference_array.append(distance)
        
        print(self.keypoint_difference_array)
        # time.sleep(.3)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    listener = AllegroRetargetingOptimizer()
    listener.run()
