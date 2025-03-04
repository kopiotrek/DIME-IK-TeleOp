import rospy
import os
from geometry_msgs.msg import PoseArray
import numpy as np
from datetime import datetime
from ik_teleop.ik_core.allegro_retargeters import AllegroKinematicControl, AllegroJointControl, AllegroKDL
from ik_teleop.ik_core.allegro_operator import AllegroHandOperator
from ik_teleop.teleop_utils.files import *
from ik_teleop.teleop_utils.constants import *
from copy import deepcopy as copy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import time


MAX_ANGLE = 2.1

# List of all ROS Topics
JOINT_POSE_TOPIC = '/quest/joint_poses' 
PAUSE_TELEOP_TOPIC = '/quest/pause' 
JOINT_STATE_TOPIC = '/allegroHand/joint_states' 
GRAV_COMP_TOPIC = '/allegroHand/grav_comp_torques' 
COMM_JOINT_STATE_TOPIC = '/allegroHand/commanded_joint_states' 
JOINT_COMM_TOPIC = '/allegroHand/joint_cmd'
# JOINT_COMM_DELTA_TOPIC = '/allegroHand/joint_cmd_delta'
JOINT_COMM_DELTA_TOPIC = '/kth_franka_plant/in/allegro_cmd'
DEFAULT_VAL = None

class TeleOp(object):
    def __init__(self):
    # def __init__(self, record_demo=False, hide_window=False, cfg=None, enable_moving_average=True):
        # Initialize ROS subscriber to get 3D hand knuckle coordinates
        if not rospy.core.is_initialized():
            # rospy.init_node('allegro_hand_operator', anonymous=True)
            try:
                rospy.init_node('hardware_teleop')
            except rospy.ROSException as e:
                rospy.loginfo(f'Node initialization failed: {str(e)}')
                pass
        self.desired_joint_angles = np.array([0.0, 0.28113237, 0.16851817, 0.0, 0.0, 0.17603329, 
            0.21581194, 0.0, 0.2928223, 0.16747166, 1.45242466, 1.45812127, 0.69531447, 1.1, 1.1, 1.1])
        self.desired_joint_angles_delta = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        # Initialize AllegroKDL for inverse kinematics
        self.allegroKDL = AllegroKDL()
        self.allegroJC = AllegroJointControl()
        self.allegroKC = AllegroKinematicControl()
        self.allegro_hand_config = get_yaml_data(get_path_in_package("configs/allegro_sim.yaml"))

        self.allegro_hand_operator = AllegroHandOperator(self.allegro_hand_config)
        self.grav_comp = DEFAULT_VAL
        self.current_joint_pose = DEFAULT_VAL
        self.cmd_joint_state = DEFAULT_VAL
        self.pause = False
        rospy.Subscriber(JOINT_STATE_TOPIC, JointState, self._sub_callback_joint_state)
        rospy.Subscriber(JOINT_POSE_TOPIC, PoseArray, self._callback_knuckle_coordinates, queue_size=1)
        rospy.Subscriber(PAUSE_TELEOP_TOPIC, Bool, self._sub_pause_teleop, queue_size=1)
        self.joint_comm_publisher = rospy.Publisher(JOINT_COMM_TOPIC, JointState, queue_size=1)
        self.joint_comm_publisher_delta = rospy.Publisher(JOINT_COMM_DELTA_TOPIC, JointState, queue_size=1)
        self.absolute = False
    
    def _sub_callback_joint_state(self, data):
        self.current_joint_pose = data

    def _sub_pause_teleop(self, data):
        if data.data:
            rospy.loginfo("||")
        else:
            rospy.loginfo("▷")
        self.pause = data.data


    def _sub_callback_grav_comp(self, data):
        self.grav_comp = data

    def _sub_callback_cmd__joint_state(self, data):
        self.cmd_joint_state = data
        
    def _clip(self, action, value):
        return np.clip(action, -value, value)

    def hand_pose(self, desired_action = np.zeros(16)):
        if self.current_joint_pose == DEFAULT_VAL:
            rospy.loginfo('No joint data received!')
            return
        action = self._clip(desired_action, MAX_ANGLE)

        if self.absolute is True:
            desired_angles = np.array(action)
            self.desired_joint_angles = copy(self.current_joint_pose)
            self.desired_joint_angles.position = list(desired_angles)
            self.desired_joint_angles.effort = list([])
            self.desired_joint_angles.velocity = list([])
            self.joint_comm_publisher.publish(self.desired_joint_angles)
        else:
            desired_angles = np.array(action)
            desired_angles_delta = desired_angles - self.current_joint_pose.position

            self.desired_joint_angles_delta = copy(self.current_joint_pose)
            self.desired_joint_angles_delta.position = list(desired_angles_delta)
            self.desired_joint_angles_delta.effort = list([])
            self.desired_joint_angles_delta.velocity = list([])
            self.joint_comm_publisher_delta.publish(self.desired_joint_angles_delta)

    def _callback_knuckle_coordinates(self, msg):
        # Extract the 21 3D coordinates from the received message (21 x 3 = 63 elements)
        # joints_coords = np.array(msg.data).reshape(21, 3)

        # # Map relevant knuckle coordinates to fingertips (you might have to adjust these indices)
        # index_tip_coord = joints_coords[8]  # Adjust index as per your knuckle mapping
        # middle_tip_coord = joints_coords[12]
        # ring_tip_coord = joints_coords[16]
        # thumb_tip_coord = joints_coords[4]

        # Compute the desired joint angles using inverse kinematics
        # self.desired_joint_angles = self.allegroKDL.get_joint_state_from_coord(
        #     index_tip_coord, middle_tip_coord, ring_tip_coord, thumb_tip_coord, seed_angles
        # )
        if not self.pause:
            self.desired_joint_angles = self.allegro_hand_operator._apply_retargeted_angles()
            self.hand_pose(self.desired_joint_angles)


    def teleop_loop(self):
        while not rospy.is_shutdown():
            continue
                    
if __name__ == '__main__':
    main = TeleOp()
    # main.allegro_hand_operator._calibrate_bounds()

    main.teleop_loop()

