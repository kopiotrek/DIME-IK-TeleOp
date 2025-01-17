from copy import deepcopy as copy
import rospy
from geometry_msgs.msg import PoseArray, Pose

from shapely.geometry import Point, Polygon 
from shapely.ops import nearest_points
from ik_teleop.ik_core.allegro_retargeters import AllegroKDLControl, AllegroJointControl
from ik_teleop.teleop_utils.files import *
from ik_teleop.teleop_utils.vectorops import *
from ik_teleop.teleop_utils.constants import *

class AllegroHandOperator:
    def __init__(self, finger_configs):
        if not rospy.core.is_initialized():
            try:
                rospy.init_node('allegro_hand_operator')
            except rospy.ROSInterruptException:
                pass
        self._transformed_hand_keypoint_subscriber = rospy.Subscriber('XR/keypoints_transformed', PoseArray, callback=self._get_joints_poses, queue_size=1)
        JOINT_COUNT = 26
        # Initializing the  finger configs
        self.finger_configs = finger_configs
        self.finger_coords = []
        self.finger_orientations = []

        #Initializing the solvers for allegro hand
        self.fingertip_solver = AllegroKDLControl()
        self.finger_joint_solver = AllegroJointControl()

        # Initialzing the moving average queues
        self.moving_average_queues = {
            'thumb': [],
            'index': [],
            'middle': [],
            'ring': []
        }

        self.last_desired_joint_angles = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.263, 0.0, 0.0, 0.0])

        # Getting the bounds for the allegro hand
        allegro_bounds_path = get_path_in_package('configs/allegro.yaml')
        self.allegro_bounds = get_yaml_data(allegro_bounds_path)

    def position_to_array(self, position):
        return np.array([position.x, position.y, position.z])

    def orientation_to_array(self, orientation):
        return np.array([orientation.x, orientation.y, orientation.z, orientation.w])

    # Get the joints poses
    def _get_joints_poses(self, msg):
        finger_coords_array = []
        finger_orientation_array = []
        if len(msg.poses) < OCULUS_NUM_KEYPOINTS - len(OCULUS_JOINTS['little']):
            print("ERROR: not enough joints received")
            return
        for i in range(OCULUS_NUM_KEYPOINTS - len(OCULUS_JOINTS['little'])):
            # finger_poses_array.append(msg.poses[i])
            finger_coords_array.append(self.position_to_array(msg.poses[i].position))     
        finger_coords_array_np = np.array(finger_coords_array)
        for i in range(OCULUS_NUM_KEYPOINTS - len(OCULUS_JOINTS['little'])):
            # finger_poses_array.append(msg.poses[i])
            finger_orientation_array.append(self.orientation_to_array(msg.poses[i].orientation)) 
        finger_orientation_array_np = np.array(finger_orientation_array)
        self.finger_coords = dict(
            wrist = finger_coords_array_np[OCULUS_JOINTS['wrist']],
            palm = finger_coords_array_np[OCULUS_JOINTS['palm']],
            thumb = finger_coords_array_np[OCULUS_JOINTS['thumb']],
            index = finger_coords_array_np[OCULUS_JOINTS['index']],
            middle = finger_coords_array_np[OCULUS_JOINTS['middle']],
            ring = finger_coords_array_np[OCULUS_JOINTS['ring']],
            # little =  finger_coords_array_np[OCULUS_JOINTS['little']],
            metacarpals =  finger_coords_array_np[OCULUS_JOINTS['metacarpals']],
            # knuckles =  finger_coords_array_np[OCULUS_JOINTS['knuckles']],
        )

        self.finger_orientations = dict(
            wrist = finger_orientation_array_np[OCULUS_JOINTS['wrist']],
            palm = finger_orientation_array_np[OCULUS_JOINTS['palm']],
            thumb = finger_orientation_array_np[OCULUS_JOINTS['thumb']],
            index = finger_orientation_array_np[OCULUS_JOINTS['index']],
            middle = finger_orientation_array_np[OCULUS_JOINTS['middle']],
            ring = finger_orientation_array_np[OCULUS_JOINTS['ring']],
            # little =  finger_orientation_array_np[OCULUS_JOINTS['little']],
            metacarpals =  finger_orientation_array_np[OCULUS_JOINTS['metacarpals']],
            # knuckles =  finger_orientation_array_np[OCULUS_JOINTS['knuckles']],
        )
        return

    # Apply the retargeted angles to the robot
    def _apply_retargeted_angles(self):
        # while not rospy.is_shutdown():
            hand_keypoints = self.finger_coords
            desired_joint_angles = self.last_desired_joint_angles
            
            # Angles
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

            # IK
            desired_joint_angles = self.fingertip_solver.finger_3D_motion(
                    finger_type = 'index',
                    finger_joint_coords = hand_keypoints['index'],
                    moving_avg_arr = self.moving_average_queues['index'],
                    curr_angles = desired_joint_angles
                )
            
            desired_joint_angles = self.fingertip_solver.finger_3D_motion(
                    finger_type = 'middle',
                    finger_joint_coords = hand_keypoints['middle'],
                    moving_avg_arr = self.moving_average_queues['middle'],
                    curr_angles = desired_joint_angles
                )

            desired_joint_angles = self.fingertip_solver.finger_3D_motion(
                    finger_type = 'ring',
                    finger_joint_coords = hand_keypoints['ring'],
                    moving_avg_arr = self.moving_average_queues['ring'],
                    curr_angles = desired_joint_angles
                )
            
            desired_joint_angles = self.fingertip_solver.thumb_motion_3D(
                    thumb_joint_coords = hand_keypoints['thumb'],
                    moving_avg_arr = self.moving_average_queues['thumb'],
                    curr_angles = desired_joint_angles
                )



            self.last_desired_joint_angles = desired_joint_angles

            
            return desired_joint_angles

            