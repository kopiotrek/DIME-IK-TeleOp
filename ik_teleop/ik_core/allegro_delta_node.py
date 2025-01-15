# !/usr/bin/env python

# Basic imports
import os
import numpy as np
import yaml
import csv

# Other ROS imports
import rospy
from sensor_msgs.msg import JointState

# Other imports
from datetime import datetime
from copy import deepcopy as copy
from IPython import embed

# List of all ROS Topics
JOINT_STATE_TOPIC = '/allegroHand/joint_states' 
JOINT_COMM_TOPIC = '/allegroHand/joint_cmd'
JOINT_COMM_DELTA_TOPIC = '/allegroHand/joint_cmd_delta'

DEFAULT_VAL = None

class AllegroDelta(object):
    def __init__(self):
        try:
            rospy.init_node('allegro_delta_node')
        except:
            pass

        rospy.Subscriber(JOINT_STATE_TOPIC, JointState, self._sub_callback_joint_state)
        rospy.Subscriber(JOINT_COMM_DELTA_TOPIC, JointState, self._sub_callback_joint_cmd)
        self.joint_comm_delta_publisher = rospy.Publisher(JOINT_COMM_TOPIC, JointState, queue_size=-1)

        self.current_joint_pose = DEFAULT_VAL
        
    def _sub_callback_joint_state(self, data):
        self.current_joint_pose = data

    def _sub_callback_joint_cmd(self, data):
        cmd_joint_state = data.position
        current_angles = self.current_joint_pose.position

        desired_angles = np.array(cmd_joint_state) + np.array(current_angles)

        desired_js = copy(self.current_joint_pose)
        desired_js.position = list(desired_angles)
        desired_js.effort = list([])
        desired_js.velocity = list([])

        self.joint_comm_delta_publisher.publish(desired_js)

if __name__ == '__main__':
    main = AllegroDelta()
    while not rospy.is_shutdown():
        continue
 