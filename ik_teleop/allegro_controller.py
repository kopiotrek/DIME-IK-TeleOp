import concurrent.futures
import signal
import sys
import rospy
import os
import numpy as np
from copy import copy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray
import multiprocessing
import time

JOINT_STATE_TOPIC = "/allegroHand/joint_states"

class AllegroController:
    def __init__(self):
        self.finger_types = ['index', 'middle', 'ring', 'thumb']
        self.current_joint_state = JointState()
        self.processes = []  # Track subprocesses

        self.joint_comm_publisher = rospy.Publisher('/allegroHand/joint_cmd', JointState, queue_size=1)
        rospy.Subscriber(JOINT_STATE_TOPIC, JointState, self._sub_callback_joint_state)
        rospy.Subscriber('/XR/keypoints_transformed' , PoseArray, self._callback_knuckle_coordinates, queue_size=1)
        rospy.Subscriber('/allegroHand/index/joint_cmd_delta', JointState, self._sub_callback_index_delta_cmd)
        rospy.Subscriber('/allegroHand/middle/joint_cmd_delta', JointState, self._sub_callback_middle_delta_cmd)
        rospy.Subscriber('/allegroHand/ring/joint_cmd_delta', JointState, self._sub_callback_ring_delta_cmd)
        rospy.Subscriber('/allegroHand/thumb/joint_cmd_delta', JointState, self._sub_callback_thumb_delta_cmd)

    def _sub_callback_joint_state(self, data):
        self.current_joint_state = data

    def _callback_knuckle_coordinates(self, data):
        # cmd_joint_state = self.index_delta_cmd + np.zeros(12)
        cmd_joint_state = list(self.index_delta_cmd[0:4]) + list(self.middle_delta_cmd[4:8]) + list(self.ring_delta_cmd[8:12]) + list(self.thumb_delta_cmd[12:])
        # cmd_joint_state = list(self.index_delta_cmd[0:4]) + list(np.zeros(12))
        current_angles = self.current_joint_state.position
        print(f"cmd_joint_state {cmd_joint_state}")
        print(f"elf.thumb_delta_cmd[13:] {self.thumb_delta_cmd[12:]}")
        print(f"elf.self.ring_delta_cmd[9:12] {self.ring_delta_cmd[9:12]}")

        desired_angles = np.array(cmd_joint_state) + np.array(current_angles)

        desired_js = copy(self.current_joint_state)
        desired_js.position = list(desired_angles)
        desired_js.effort = []
        desired_js.velocity = []

        if self.index_mutex and self.middle_mutex and self.ring_mutex and self.thumb_mutex is True:
            self.joint_comm_publisher.publish(desired_js)
            self.index_mutex = False
            self.middle_mutex = False
            self.ring_mutex = False
            self.thumb_mutex = False


    def _sub_callback_index_delta_cmd(self, data):
        self.index_delta_cmd = data.position
        self.index_mutex = True

    def _sub_callback_middle_delta_cmd(self, data):
        self.middle_delta_cmd = data.position
        self.middle_mutex = True

    def _sub_callback_ring_delta_cmd(self, data):
        self.ring_delta_cmd = data.position
        self.ring_mutex = True

    def _sub_callback_thumb_delta_cmd(self, data):
        self.thumb_delta_cmd = data.position
        self.thumb_mutex = True



if __name__ == '__main__':
    rospy.init_node('allegro_controller')
    allegro_controller = AllegroController()

    # Set up signal handler for Ctrl+C

    print('Started Allegro Hand controller')
    try:
        # allegro_controller.start_finger_controllers()
        rospy.spin()
    except Exception as e:
        print(f"Exception occurred: {e}")
    finally:
        print("Allegro Controller terminated.")
