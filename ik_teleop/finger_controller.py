import rospy
import os
from geometry_msgs.msg import PoseArray
import numpy as np
from datetime import datetime
from ik_teleop.ik_core.allegro_retargeters import AllegroKDL
from ik_teleop.ik_core.allegro_operator import AllegroHandOperator
from ik_teleop.teleop_utils.files import *
from ik_teleop.teleop_utils.constants import *
from copy import deepcopy as copy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import time


# List of all ROS Topics
XR_KEYPOINTS_TOPIC = '/XR/keypoints_transformed' 
PAUSE_TELEOP_TOPIC = '/XR/Pause' 
JOINT_STATE_TOPIC = '/allegroHand/joint_states' 
COMM_JOINT_STATE_TOPIC = '/allegroHand/commanded_joint_states' 
JOINT_COMM_TOPIC = '/allegroHand/joint_cmd'
JOINT_COMM_DELTA_TOPIC = '/allegroHand/joint_cmd_delta'
# JOINT_COMM_TOPIC = '/kth_franka_plant/in/allegro_cmd'

class TeleOp(object):
    def __init__(self):
    # def __init__(self, record_demo=False, hide_window=False, cfg=None, enable_moving_average=True):
        # Initialize ROS subscriber to get 3D hand knuckle coordinates
        if not rospy.core.is_initialized():
            try:
                rospy.init_node('hardware_teleop')
            except rospy.ROSException as e:
                print(f'Node initialization failed: {str(e)}')
                pass
        self.desired_joint_angles = np.array([0.0, 0.28113237, 0.16851817, 0.0, 0.0, 0.17603329, 
            0.21581194, 0.0, 0.2928223, 0.16747166, 1.45242466, 1.45812127, 0.69531447, 1.1, 1.1, 1.1])
        self.desired_joint_angles_delta = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.finger_type = None
        self.allegro_hand_config = get_yaml_data(get_path_in_package("configs/allegro_sim.yaml"))

        self.allegro_hand_operator = AllegroHandOperator(self.allegro_hand_config)
        self.current_joint_pose = None
        self.cmd_joint_state = None
        self.pause = True
        self.delta_control_mode = True

        rospy.Subscriber(JOINT_STATE_TOPIC, JointState, self._sub_callback_joint_state)
        rospy.Subscriber(XR_KEYPOINTS_TOPIC, PoseArray, self._callback_knuckle_coordinates, queue_size=1)
        rospy.Subscriber(PAUSE_TELEOP_TOPIC, Bool, self._sub_pause_teleop, queue_size=1)
        if self.delta_control_mode:
            rospy.Subscriber(JOINT_COMM_DELTA_TOPIC, JointState, self._sub_callback_joint_cmd)

        self.joint_comm_publisher = rospy.Publisher(JOINT_COMM_TOPIC, JointState, queue_size=1)
        self.joint_comm_publisher_delta = rospy.Publisher(JOINT_COMM_DELTA_TOPIC, JointState, queue_size=1)
    
    def _sub_callback_joint_state(self, data):
        self.current_joint_pose = data

    def _sub_pause_teleop(self, data):
        if data.data:
            print("||")
        else:
            print("▷")
        self.pause = data.data

    def _sub_callback_joint_cmd(self, data):
        cmd_joint_state = data.position
        current_angles = self.current_joint_pose.position

        desired_angles = np.array(cmd_joint_state) + np.array(current_angles)

        desired_js = copy(self.current_joint_pose)
        desired_js.position = list(desired_angles)
        desired_js.effort = list([])
        desired_js.velocity = list([])

        self.joint_comm_publisher.publish(desired_js)


    def hand_pose(self, action=np.zeros(16)):
        if self.current_joint_pose == None:
            print('No joint data received!')
            return
    
        current_angles = np.array(self.current_joint_pose.position)  # Convert JointState to numpy array
    
        if self.delta_control_mode is False:
            desired_angles = np.array(action)
            self.desired_joint_angles = copy(self.current_joint_pose)
            self.desired_joint_angles.position = list(desired_angles)
            self.desired_joint_angles.effort = list([])
            self.desired_joint_angles.velocity = list([])
            self.joint_comm_publisher.publish(self.desired_joint_angles)
        else:
            desired_angles = np.array(action)
            desired_angles_delta = desired_angles - current_angles  # Correct subtraction
    
            self.desired_joint_angles_delta = copy(self.current_joint_pose)
            self.desired_joint_angles_delta.position = list(desired_angles_delta)
            self.desired_joint_angles_delta.effort = list([])
            self.desired_joint_angles_delta.velocity = list([])
            self.joint_comm_publisher_delta.publish(self.desired_joint_angles_delta)
    
    def _callback_knuckle_coordinates(self, msg):
        if not self.pause:
            self.desired_joint_angles = self.allegro_hand_operator._apply_retargeted_angles(self.finger_type)
            self.hand_pose(self.desired_joint_angles)
        else:
            if self.current_joint_pose != None:
                current_angles_array = np.array(self.current_joint_pose.position)  # Ensure correct type
                self.hand_pose(current_angles_array)

    def control_finger(self, finger_type):
        self.finger_type = finger_type
        while not rospy.is_shutdown():
            continue


