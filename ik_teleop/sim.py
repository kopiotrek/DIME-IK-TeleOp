
"""
Used to record demonstrations for environments in mujoco.
To run and record demos:
    python ik_teleop/sim.py --record

To run with a different env:
    python ik_teleop/sim.py --record --env_name block-v5
"""
# Standard imports
import os
import sys
import numpy as np

# Parameter management imports
from hydra import initialize, compose

# Image based imports
import cv2
import mediapipe
import pyrealsense2 as rs

# Gym imports
from mjrl.utils.gym_env import GymEnv
import mj_allegro_envs

#from sensor_msgs.msg import JointState

from datetime import datetime

# Allegro Inverse Kinematics based controller
from ik_teleop.ik_core.allegro_controller import AllegroIKController

# Other utility imports
import utils.camera as camera
import utils.joint_handling as joint_handlers
from utils.transformations import perform_persperctive_transformation

# Other miscellaneous imports
from copy import deepcopy as copy

import argparse
import pickle

# Debugging imports
from IPython import embed

from multiprocessing import Process, Queue
from ik_teleop.teleop_utils.hand_detector import MediapipeJoints
from ik_teleop.teleop_utils.mediapipe_visualizer import PlotMediapipeHand
from ik_teleop.teleop_utils.hand_simulator import HandSimulator

CAM_SERIAL_NUM = '023322062089'
CALIBRATION_FILE_PATH = os.path.join(os.getcwd(), 'bound_data', 'calibrated_values.npy')
hand_coordinates = None

def detector(queue, record_demo):
    print('Entered detector!')
    try:
        print('record_demo val: ' + str(record_demo))
        detector = MediapipeJoints(cam_serial_num=CAM_SERIAL_NUM,record_demo=record_demo)
        detector.queue = queue
        
        if(record_demo):
            demo_dir_q.put(detector.demo_dir)
        detector.detect()
    except KeyboardInterrupt:
        print("ENTERED DETECTOR KEYBOARD INTERRUPT")
        detector.finish_recording()

def visualizer(queue,hand_coords):
    global HAND_COORD_TOPIC, hand_coordinates
    print('Entered Visualizer')

    if os.path.exists(CALIBRATION_FILE_PATH):
        calibration_values = np.load(CALIBRATION_FILE_PATH)
        hand_vis = PlotMediapipeHand(calibration_values)
    else:
        hand_vis = PlotMediapipeHand()

    while True:
        joints = None
        while(joints is None):
            joints = queue.get()

        hand_coordinates = joints
        hand_coords.put(joints)

        if hand_coordinates is not None:
            hand_vis.draw(hand_coordinates[:, 0], hand_coordinates[:, 1])
    

def robot_simulator(env_name, hand_coords,record_demo):
    # global hand_coordinates
    demo_dirname = None
    if(record_demo):
        demo_dirname = demo_dir_q.get()
        print('DEMO DIR' + str(demo_dirname))

    sim = HandSimulator(env_name=env_name,record_demo=record_demo,demo_dir = demo_dirname)
    while True:
        joints =  hand_coords.get()
        if joints is not None:
            sim.move(joints)
        else:
            print("joints is none")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Use teleop to operate Mujoco sim')
    parser.add_argument('--record', action='store_true', default=False)
    parser.add_argument('--env_name', type=str, default='block-v3')
    args = parser.parse_args()
    record_demo = args.record

    q = Queue()
    hand_q = Queue()
    demo_dir_q = Queue()

    try:
        print("***************************************************************\n     Starting detection process \n***************************************************************")
        det_process = Process(target = detector, args=(q,record_demo))
        det_process.start()
        print(q.get())
        print("\nHand detection process started!\n")

        print("***************************************************************\n     Starting visualizer process \n***************************************************************")
        vis_process = Process(target = visualizer, args=(q,hand_q))
        vis_process.start()
        print("\nVisualization process started!\n")

        print("\n***************************************************************\n     Starting controller process \n***************************************************************")
        controller_process = Process(target = robot_simulator, args=(args.env_name, hand_q,record_demo,))
        controller_process.start()
        print("\Simulator process started!")    

        # det_process.join()
        # vis_process.join()
        # controller_process.join()
    except KeyboardInterrupt:
        print("ENCOUNTERED KEYBOARD INTERRUPT")
        det_process.terminate()
        vis_process.terminate()
        controller_process.terminate()
        import sys
        sys.exit(0)
    