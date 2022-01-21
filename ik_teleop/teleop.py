import os
import time
import numpy as np
from multiprocessing import Process

import rospy
from std_msgs.msg import Float64MultiArray

from ik_teleop.teleop_utils.hand_detector import MediapipeJointPublisher
from ik_teleop.teleop_utils.dexarm_operation import DexArmOp
from ik_teleop.teleop_utils.mediapipe_visualizer import PlotMediapipeHand

HAND_COORD_TOPIC = '/mediapipe_joint_coords'
hand_coordinates = None

CALIBRATION_FILE_PATH = os.path.join(os.getcwd(), 'bound_data', 'calibrated_values.npy')

def detector():
    mp_detector = MediapipeJointPublisher()
    mp_detector.detect()

def _callback(hand_coord):
    global hand_coordinates
    hand_coordinates = np.array(list(hand_coord.data)).reshape(11, 2)

def visualizer():
    global HAND_COORD_TOPIC, hand_coordinates

    try:
        rospy.init_node('hand_vis')
    except:
        pass
    
    rospy.Subscriber(HAND_COORD_TOPIC, Float64MultiArray, _callback, queue_size = 1)

    if os.path.exists(CALIBRATION_FILE_PATH):
        calibration_values = np.load(CALIBRATION_FILE_PATH)
        hand_vis = PlotMediapipeHand(calibration_values)
    else:
        hand_vis = PlotMediapipeHand()

    while True:
        if hand_coordinates is not None:
            hand_vis.draw(hand_coordinates[:, 0], hand_coordinates[:, 1])

def robot_controller():
    control = DexArmOp()
    control.move()

if __name__ == '__main__':
    print("***************************************************************\n     Starting detection process \n***************************************************************")
    det_process = Process(target = detector)
    det_process.start()
    print("\nHand detection process started!\n")

    print("***************************************************************\n     Starting visualizer process \n***************************************************************")
    vis_process = Process(target = visualizer)
    vis_process.start()
    print("\nVisualization process started!\n")

    print("Keep your hand under the camera to display the graph.")
    time.sleep(3)

    print("\n***************************************************************\n     Starting controller process \n***************************************************************")
    controller_process = Process(target = robot_controller)
    controller_process.start()
    print("\nController process started!")    
    
    det_process.join()
    vis_process.join()
    controller_process.join()