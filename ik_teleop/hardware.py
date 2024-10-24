import rospy
import os
from std_msgs.msg import Float64MultiArray
import numpy as np
from datetime import datetime
from ik_teleop.ik_core.allegro_retargeters import AllegroKinematicControl, AllegroJointControl, AllegroKDL
from ik_teleop.ik_core.allegro_operator import AllegroHandOperator
from ik_teleop.ik_core.allegro_control import DexArmControl
from ik_teleop.teleop_utils.files import *
from ik_teleop.teleop_utils.constants import *
from copy import deepcopy as copy
from sensor_msgs.msg import JointState
import time


MAX_ANGLE = 2.1

# List of all ROS Topics
JOINT_STATE_TOPIC = '/allegroHand/joint_states' 
GRAV_COMP_TOPIC = '/allegroHand/grav_comp_torques' 
COMM_JOINT_STATE_TOPIC = '/allegroHand/commanded_joint_states' 
JOINT_COMM_TOPIC = '/allegroHand/joint_cmd'
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
                print(f'Node initialization failed: {str(e)}')
                pass
        self.desired_joint_angles = np.array([0.0, 0.28113237, 0.16851817, 0.0, 0.0, 0.17603329, 
            0.21581194, 0.0, 0.2928223, 0.16747166, 1.45242466, 1.45812127, 0.69531447, 1.1, 1.1, 1.1])
        
        # Initialize AllegroKDL for inverse kinematics
        self.allegroKDL = AllegroKDL()
        self.allegroJC = AllegroJointControl()
        self.allegroKC = AllegroKinematicControl()
        self.allegroDAC = DexArmControl()
        self.allegro_hand_config = get_yaml_data('/home/piotr/RPL/DIME-IK-TeleOp/ik_teleop/configs/allegro_sim.yaml')

        self.allegro_hand_operator = AllegroHandOperator(self.allegro_hand_config)
        self.grav_comp = DEFAULT_VAL
        self.current_joint_pose = DEFAULT_VAL
        self.cmd_joint_state = DEFAULT_VAL
        rospy.Subscriber(JOINT_STATE_TOPIC, JointState, self._sub_callback_joint_state)
        rospy.Subscriber('/transformed_hand_coords', Float64MultiArray, self._callback_knuckle_coordinates, queue_size=1)
        self.joint_comm_publisher = rospy.Publisher(JOINT_COMM_TOPIC, JointState, queue_size=1)
    
    def _sub_callback_joint_state(self, data):
        self.current_joint_pose = data

    def _sub_callback_grav_comp(self, data):
        self.grav_comp = data

    def _sub_callback_cmd__joint_state(self, data):
        self.cmd_joint_state = data
        
    def _clip(self, action, value):
        return np.clip(action, -value, value)
    def hand_pose(self, desired_action = np.zeros(16), absolute = True):
        if self.current_joint_pose == DEFAULT_VAL:
            print('No joint data received!')
            return
        action = self._clip(desired_action, MAX_ANGLE)
        # current_angles = self.current_joint_pose.position

        if absolute is True:
            desired_angles = np.array(action)
        else:
            desired_angles = np.array(action) + np.array(current_angles)

        self.desired_joint_angles = copy(self.current_joint_pose)
        self.desired_joint_angles.position = list(desired_angles)
        self.desired_joint_angles.effort = list([])


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
        self.desired_joint_angles = self.allegro_hand_operator._apply_retargeted_angles()
        self.hand_pose(self.desired_joint_angles)
        self.joint_comm_publisher.publish(self.desired_joint_angles)


    def teleop_loop(self):
        while not rospy.is_shutdown():
            continue
                    
if __name__ == '__main__':
    main = TeleOp()
    # main.allegro_hand_operator._calibrate_bounds()

    main.teleop_loop()

