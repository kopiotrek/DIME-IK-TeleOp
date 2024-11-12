from copy import deepcopy as copy
from .operator import Operator
import rospy
from std_msgs.msg import Float64MultiArray

from shapely.geometry import Point, Polygon 
from shapely.ops import nearest_points
from ik_teleop.ik_core.allegro_calibrator import OculusThumbBoundCalibrator
# from ik_teleop.ik_core.allegro import AllegroHand
from ik_teleop.ik_core.allegro_retargeters import AllegroKDLControl, AllegroJointControl
from ik_teleop.teleop_utils.files import *
from ik_teleop.teleop_utils.vectorops import coord_in_bound
from ik_teleop.teleop_utils.timer import FrequencyTimer
from ik_teleop.teleop_utils.constants import *

class AllegroHandOperator(Operator):
    def __init__(self, finger_configs):
        # self.notify_component_start('allegro hand operator')
        # self._host, self._port = host, transformed_keypoints_port
        # Subscriber for the transformed hand keypoints
        # self._transformed_hand_keypoint_subscriber = ZMQKeypointSubscriber(
        #     host = self._host,
        #     port = self._port,
        #     topic = 'transformed_hand_coords'
        # )
        # # Subscriber for the transformed arm frame
        # self._transformed_arm_keypoint_subscriber = ZMQKeypointSubscriber(
        #     host = self._host,
        #     port = self._port,
        #     topic = 'transformed_hand_frame'
        # )
        if not rospy.core.is_initialized():
            try:
                rospy.init_node('allegro_hand_operator')
            except rospy.ROSInterruptException:
                pass
        self._transformed_hand_keypoint_subscriber = rospy.Subscriber('transformed_hand_coords', Float64MultiArray, callback=self._get_finger_coords, queue_size=1)

        # Initializing the  finger configs
        self.finger_configs = finger_configs
        
        #Initializing the solvers for allegro hand
        self.fingertip_solver = AllegroKDLControl()
        self.finger_joint_solver = AllegroJointControl()

        # Initializing the robot controller
        # self._robot = AllegroHand()

        # Initialzing the moving average queues
        self.moving_average_queues = {
            'thumb': [],
            'index': [],
            'middle': [],
            'ring': []
        }

        self.last_desired_joint_angles = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.263, 0.0, 0.0, 0.0])

        # Calibrating to get the thumb bounds
        self._calibrate_bounds()

        # Getting the bounds for the allegro hand
        allegro_bounds_path = get_path_in_package('configs/allegro.yaml')
        self.allegro_bounds = get_yaml_data(allegro_bounds_path)

        self._timer = FrequencyTimer(VR_FREQ)

        # Using 3 dimensional thumb motion or two dimensional thumb motion
        if self.finger_configs.get('three_dim'):
            self.thumb_angle_calculator = self._get_3d_thumb_angles
        else:
            self.thumb_angle_calculator = self._get_2d_thumb_angles

    @property
    def timer(self):
        return self._timer

    @property
    def robot(self):
        return None
        # return self._robot

    # @property
    # def transformed_arm_keypoint_subscriber(self):
    #     return self._transformed_arm_keypoint_subscriber
    
    @property
    def transformed_hand_keypoint_subscriber(self):
        return self._transformed_hand_keypoint_subscriber
    
    # This function differentiates between the real robot and simulation
    def return_real(self):
        return True

    # Calibrate the thumb bounds
    def _calibrate_bounds(self):
        calibrator = OculusThumbBoundCalibrator()
        self.hand_thumb_bounds = calibrator.get_bounds() # Provides [thumb-index bounds, index-middle bounds, middle-ring-bounds]
        print(f'THUMB BOUNDS IN THE OPERATOR: {self.hand_thumb_bounds}')

    # Get the transformed finger coordinates
    def _get_finger_coords(self,msg):
        data = np.array(msg.data).reshape(21, 3)
        self.finger_coords = dict(
            # wrist = np.vstack([data[0], data[OCULUS_JOINTS['wrist']]]),
            wrist = np.vstack([data[0], np.array([[0.0, 0.0, 0.0]])]),
            index = np.vstack([data[0], data[OCULUS_JOINTS['index']]]),
            middle = np.vstack([data[0], data[OCULUS_JOINTS['middle']]]),
            ring = np.vstack([data[0], data[OCULUS_JOINTS['ring']]]),
            thumb =  np.vstack([data[0], data[OCULUS_JOINTS['thumb']]])
        )

    # Get robot thumb angles when moving only in 2D motion
    def _get_2d_thumb_angles(self, thumb_keypoints, curr_angles):
        # print("curr_angles:",curr_angles)
        # print("thumb_keypoints",thumb_keypoints)
        for idx, thumb_bounds in enumerate(self.hand_thumb_bounds):
            if coord_in_bound(thumb_bounds[:4], thumb_keypoints[:2]) > -1:
                return self.fingertip_solver.thumb_motion_2D(
                    hand_coordinates = thumb_keypoints, 
                    xy_hand_bounds = thumb_bounds[:4],
                    yz_robot_bounds = self.allegro_bounds['thumb_bounds'][idx]['projective_bounds'],
                    robot_x_val = self.allegro_bounds['x_coord'],
                    moving_avg_arr = self.moving_average_queues['thumb'], 
                    curr_angles = curr_angles
                )
        
        return curr_angles

    # Get robot thumb angles when moving in 3D motion
    # def _get_3d_thumb_angles(self, thumb_keypoints, curr_angles):
    #     # Precompute reused values
    #     planar_thumb_bounds_2d = Polygon(self.hand_thumb_bounds[:4])
    #     z_hand_bound = self.hand_thumb_bounds[4]

    #     # Using shapely's nearest_points to get the closest point within the bounds
    #     planar_point = Point(thumb_keypoints[:2])  # Only use the 2D points for planar calculations
    #     closest_point = nearest_points(planar_thumb_bounds_2d, planar_point)[0]

    #     # Form 3D coordinates by reusing z from thumb_keypoints
    #     closest_point_coords = [closest_point.x, closest_point.y, thumb_keypoints[2]]

    #     # Convert polygon to list of points for OpenCV perspective transform
    #     thumb_bounds_points = np.array(self.hand_thumb_bounds[:4], dtype=np.float32)

    #     return self.fingertip_solver.thumb_motion_3D(
    #         hand_coordinates=closest_point_coords,
    #         xy_hand_bounds=thumb_bounds_points,  # Pass as a list of points instead of Polygon
    #         yz_robot_bounds=self.allegro_bounds['thumb_bounds'][0]['projective_bounds'],
    #         z_hand_bound=z_hand_bound,
    #         x_robot_bound=self.allegro_bounds['thumb_bounds'][0]['x_bounds'],
    #         moving_avg_arr=self.moving_average_queues['thumb'], 
    #         curr_angles=curr_angles
    #     )
    

    # 27 mm is the distance between index and middle knuckle. In the hand tracking it is 1
    # Axis meaning: 
    # Origin is where palm meets middle finger
    # X is vector from origin outwards (perpendicular to palm)
    # Y is vector from origin to the thumb (left)
    # Z is vector from origin to middle finger (up)
    def _get_3d_thumb_angles(self, thumb_joint_coords, curr_angles):

            # 27 mm is the distance between my index and middle knuckle. In the hand tracking coord system it is 1
        # transformed_thumb_tip_keypoint = thumb_tip_keypoint*0.027 #conversion to m
        thumb_joint_coords = thumb_joint_coords*0.017 #conversion to m
        return self.fingertip_solver.thumb_motion_3D(
            thumb_joint_coords=thumb_joint_coords,
            moving_avg_arr=self.moving_average_queues['thumb'], 
            curr_angles=curr_angles
        )

        
    # Generate frozen angles for the fingers
    def _generate_frozen_angles(self, joint_angles, finger_type):
        for idx in range(ALLEGRO_JOINTS_PER_FINGER):
            if idx > 0:
                joint_angles[idx + ALLEGRO_JOINT_OFFSETS[finger_type]] = 0.05
            else:
                joint_angles[idx + ALLEGRO_JOINT_OFFSETS[finger_type]] = 0

        return joint_angles
    
    # Apply the retargeted angles to the robot
    def _apply_retargeted_angles(self):
        # while not rospy.is_shutdown():
            hand_keypoints = self.finger_coords
            desired_joint_angles = self.last_desired_joint_angles
            
            desired_joint_angles = self.finger_joint_solver.calculate_finger_angles(
                    finger_type = 'index',
                    finger_joint_coords = hand_keypoints['index'],
                    curr_angles = desired_joint_angles,
                    moving_avg_arr = self.moving_average_queues['index']
                )
            
            desired_joint_angles = self.finger_joint_solver.calculate_finger_angles(
                    finger_type = 'middle',
                    finger_joint_coords = hand_keypoints['middle'],
                    curr_angles = desired_joint_angles,
                    moving_avg_arr = self.moving_average_queues['middle']
                )
           
            
            desired_joint_angles = self.finger_joint_solver.calculate_finger_angles(
                    finger_type = 'ring',
                    finger_joint_coords = hand_keypoints['ring'],
                    curr_angles = desired_joint_angles,
                    moving_avg_arr = self.moving_average_queues['ring']
                )

            
            desired_joint_angles = self._get_3d_thumb_angles(
                    thumb_joint_coords = hand_keypoints['thumb'],
                    curr_angles = desired_joint_angles,
            )

            # self.last_desired_joint_angles = np.round(desired_joint_angles, 2)
            self.last_desired_joint_angles = desired_joint_angles
            # print(f"last_desired_joint_angles{self.last_desired_joint_angles}")

            
            return desired_joint_angles