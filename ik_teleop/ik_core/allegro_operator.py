from copy import deepcopy as copy
from .operator import Operator
import rospy
from geometry_msgs.msg import PoseArray, Pose

from shapely.geometry import Point, Polygon 
from shapely.ops import nearest_points
from ik_teleop.ik_core.allegro_calibrator import OculusThumbBoundCalibrator
# from ik_teleop.ik_core.allegro import AllegroHand
from ik_teleop.ik_core.allegro_retargeters import AllegroKDLControl, AllegroJointControl
from ik_teleop.teleop_utils.files import *
from ik_teleop.teleop_utils.vectorops import *
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
        self._transformed_hand_keypoint_subscriber = rospy.Subscriber('XR/JointPoseArray', PoseArray, callback=self._get_joints_poses, queue_size=1)
        JOINT_COUNT = 26
        # Initializing the  finger configs
        self.finger_configs = finger_configs
        # self.finger_poses_array = []
        self.finger_coords = []
        # for i in range(OCULUS_NUM_KEYPOINTS):
        #     pose = Pose()
        #     self.finger_poses.poses.append(pose)

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
        self.knuckle_points = (OCULUS_JOINTS['knuckles'][3], OCULUS_JOINTS['knuckles'][0])

        self.last_desired_joint_angles = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.263, 0.0, 0.0, 0.0])

        # Calibrating to get the thumb bounds
        # self._calibrate_bounds()

        # Getting the bounds for the allegro hand
        allegro_bounds_path = get_path_in_package('configs/allegro.yaml')
        self.allegro_bounds = get_yaml_data(allegro_bounds_path)

        # self._timer = FrequencyTimer(VR_FREQ)

        # # Using 3 dimensional thumb motion or two dimensional thumb motion
        # if self.finger_configs.get('three_dim'):
        #     self.thumb_angle_calculator = self._get_3d_thumb_angles
        # else:
        #     self.thumb_angle_calculator = self._get_2d_thumb_angles

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
        # print(f'THUMB BOUNDS IN THE OPERATOR: {self.hand_thumb_bounds}')

    def position_to_array(self, position):
        return np.array([position.x, position.y, position.z])

    # Get the joints poses
    def _get_joints_poses(self, msg):
        finger_coords_array = []
        if len(msg.poses) < OCULUS_NUM_KEYPOINTS:
            print("ERROR: not enough joints received")
            return
        for i in range(OCULUS_NUM_KEYPOINTS):
            # finger_poses_array.append(msg.poses[i])
            finger_coords_array.append(self.position_to_array(msg.poses[i].position))     
        finger_coords_array_np = np.array(finger_coords_array)

        self.transform_keypoints(finger_coords_array_np)


        # self.finger_coords = dict(
        #     wrist = finger_coords_array_np[OCULUS_JOINTS['wrist']],
        #     palm = finger_coords_array_np[OCULUS_JOINTS['palm']],
        #     thumb = finger_coords_array_np[OCULUS_JOINTS['thumb']],
        #     index = finger_coords_array_np[OCULUS_JOINTS['index']],
        #     middle = finger_coords_array_np[OCULUS_JOINTS['middle']],
        #     ring = finger_coords_array_np[OCULUS_JOINTS['ring']],
        #     little =  finger_coords_array_np[OCULUS_JOINTS['little']],
        #     metacarpals =  finger_coords_array_np[OCULUS_JOINTS['metacarpals']],
        #     knuckles =  finger_coords_array_np[OCULUS_JOINTS['knuckles']],
        # )

        # print(f"self.finger_coords {self.finger_coords}")
        return


    # Function to find hand coordinates with respect to the wrist
    def _translate_coords(self, coords):
        return copy(coords) - coords[12]

    # Create a coordinate frame for the hand
    # Axis meaning: 
    # Origin is where palm meets middle finger
    # X is vector from origin outwards (perpendicular to palm)
    # Y is vector from origin to the thumb (left)
    # Z is vector from origin to middle finger (up)
    def _get_coord_frame(self, wrist_coord, index_knuckle_coord, little_knuckle_coord):
        z_axis = normalize_vector(wrist_coord)
        x_axis = normalize_vector(index_knuckle_coord - little_knuckle_coord)
        y_axis = normalize_vector(np.cross(z_axis, x_axis))
        
        return [x_axis, y_axis, z_axis]

    def transform_keypoints(self, finger_coords_array_np):
        finger_coords_array_np = self._translate_coords(finger_coords_array_np)
        original_coord_frame = self._get_coord_frame(
            finger_coords_array_np[OCULUS_JOINTS['wrist'][0]],
            finger_coords_array_np[OCULUS_JOINTS['knuckles'][0]],
            finger_coords_array_np[OCULUS_JOINTS['knuckles'][3]]
        )
        if np.linalg.det(original_coord_frame) == 0:
            rospy.logerr("Original coord frame is singular and cannot be inverted")
            return

        try:
            rotation_matrix = np.linalg.solve(original_coord_frame, np.eye(3)).T
            finger_coords = (rotation_matrix @ finger_coords_array_np.T).T
        except np.linalg.LinAlgError as e:
            rospy.logerr(f"Error computing rotation matrix: {e}")
        self.finger_coords = dict(
            wrist = finger_coords[OCULUS_JOINTS['wrist']],
            palm = finger_coords[OCULUS_JOINTS['palm']],
            thumb = finger_coords[OCULUS_JOINTS['thumb']],
            index = finger_coords[OCULUS_JOINTS['index']],
            middle = finger_coords[OCULUS_JOINTS['middle']],
            ring = finger_coords[OCULUS_JOINTS['ring']],
            little =  finger_coords[OCULUS_JOINTS['little']],
            metacarpals =  finger_coords[OCULUS_JOINTS['metacarpals']],
            knuckles =  finger_coords[OCULUS_JOINTS['knuckles']],
        )

    # def _get_joints_coords(self, msg):
    #     for i in OCULUS_JOINTS:
    #         self.finger_poses.append(msg[i])
    #     return

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
            
            # print(f"hand_keypoints['index'] {hand_keypoints['index']}")
            # print(f"hand_keypoints['index'], {hand_keypoints['index'],}")

            # desired_joint_angles = self.finger_joint_solver.calculate_finger_angles(
            #         finger_type = 'index',
            #         finger_joint_coords = hand_keypoints['index'],
            #         metacarpals_coords = hand_keypoints['metacarpals'],
            #         curr_angles = desired_joint_angles,
            #         moving_avg_arr = self.moving_average_queues['index']
            #     )
            
            # desired_joint_angles = self.finger_joint_solver.calculate_finger_angles(
            #         finger_type = 'middle',
            #         finger_joint_coords = hand_keypoints['middle'],
            #         metacarpals_coords = hand_keypoints['metacarpals'],
            #         curr_angles = desired_joint_angles,
            #         moving_avg_arr = self.moving_average_queues['middle']
            #     )
           
            
            # desired_joint_angles = self.finger_joint_solver.calculate_finger_angles(
            #         finger_type = 'ring',
            #         finger_joint_coords = hand_keypoints['ring'],
            #         metacarpals_coords = hand_keypoints['metacarpals'],
            #         curr_angles = desired_joint_angles,
            #         moving_avg_arr = self.moving_average_queues['ring']
            #     )

            
            # desired_joint_angles = self._get_3d_thumb_angles(
            #         thumb_joint_coords = hand_keypoints['thumb'],
            #         curr_angles = desired_joint_angles,
            # )
            desired_joint_angles = self.finger_joint_solver.calculate_thumb_angles(
                    index_knuckle = hand_keypoints['knuckles'][1],
                    thumb_joint_coords = hand_keypoints['thumb'],
                    curr_angles = desired_joint_angles,
                    moving_avg_arr = self.moving_average_queues['thumb']
                )

            # self.last_desired_joint_angles = np.round(desired_joint_angles, 2)
            self.last_desired_joint_angles = desired_joint_angles
            # print(f"last_desired_joint_angles{self.last_desired_joint_angles}")

            
            return desired_joint_angles

            