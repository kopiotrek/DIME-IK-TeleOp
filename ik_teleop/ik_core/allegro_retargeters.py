import numpy as np
from abc import ABC
from copy import deepcopy as copy
from .allegro_kdl import AllegroKDL
from  ik_teleop.teleop_utils.files import *
from  ik_teleop.teleop_utils.vectorops import *
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import tf.transformations as tf

class AllegroKinematicControl(ABC):
    def __init__(self, bounded_angles = True):
        np.set_printoptions(suppress = True)

        # Loading the Allegro Hand configs
        self.hand_configs = get_yaml_data(get_path_in_package("robot/allegro/configs/allegro_info.yaml"))
        self.finger_configs = get_yaml_data(get_path_in_package("robot/allegro/configs/allegro_link_info.yaml"))
        self.bound_info = get_yaml_data(get_path_in_package("robot/allegro/configs/allegro_bounds.yaml"))

        self.time_steps = self.bound_info['time_steps']

        self.bounded_angles = bounded_angles
        self.bounds = {}
        for finger in self.hand_configs['fingers'].keys():
            self.bounds[finger] = np.array(self.bound_info['jointwise_angle_bounds'][
                self.finger_configs['links_info'][finger]['offset'] : self.finger_configs['links_info'][finger]['offset'] + 4
            ])

    def _get_curr_finger_angles(self, curr_angles, finger_type):
        return np.array(curr_angles[
            self.finger_configs['links_info'][finger_type]['offset'] : self.finger_configs['links_info'][finger_type]['offset'] + 4
        ])


class AllegroJointControl(AllegroKinematicControl):
    def __init__(self, bounded_angles = True):
        super().__init__(bounded_angles)
        np.set_printoptions(suppress = True)

        self.linear_scaling_factors = self.bound_info['linear_scaling_factors']
        self.rotatory_thumb_scaling_factors = self.bound_info['rotatory_thumb_scaling_factors']

    def _get_filtered_angles(self, finger_type, calc_finger_angles, curr_angles, moving_avg_arr):
        curr_finger_angles = self._get_curr_finger_angles(curr_angles, finger_type)
        avg_finger_angles = moving_average(calc_finger_angles, moving_avg_arr, self.time_steps)       
        desired_angles = np.array(copy(curr_angles))

        
        for idx in range(self.hand_configs['joints_per_finger']):
                desired_angles[self.finger_configs['links_info'][finger_type]['offset'] + idx] = avg_finger_angles[idx]
                

        for idx in range(1, 16):
            if desired_angles[idx] > 2.5:
                desired_angles[idx] = 0

        return desired_angles 

    def _get_filtered_thumb_angles(self, finger_type, calc_finger_angles, curr_angles, moving_avg_arr):
        
        curr_finger_angles = self._get_curr_finger_angles(curr_angles, finger_type)
        avg_finger_angles = moving_average(calc_finger_angles, moving_avg_arr, self.time_steps)       
        desired_angles = np.array(copy(curr_angles))

        
        for idx in range(self.hand_configs['joints_per_finger']):
                desired_angles[self.finger_configs['links_info'][finger_type]['offset'] + idx] = avg_finger_angles[idx-1]
                

        return desired_angles 

    # def calculate_rotatory_joint_angle(self, finger_type, finger_joint_coords, knuckles_coords):
    #     if finger_type is 'index':
    #         idx = 4
    #         offset = 1.8
    #     elif finger_type is 'middle':
    #         idx = 4
    #         offset = 1.8
    #     elif finger_type is 'ring':
    #         idx = 4
    #         offset = 1.7
    #     origin = finger_joint_coords[1]
    #     vector_origin_to_joint = finger_joint_coords[2] - origin
    #     vector_origin_to_next_knuckle = knuckles_coords[idx] - origin

    #     z_axis = np.cross(vector_origin_to_next_knuckle, vector_origin_to_joint)
    #     z_axis /= np.linalg.norm(z_axis)

    #     x_axis = vector_origin_to_next_knuckle / np.linalg.norm(vector_origin_to_next_knuckle)
    #     y_axis = np.cross(z_axis, x_axis)

    #     rotation_matrix = np.column_stack((x_axis, y_axis, z_axis))

    #     vector_in_plane = np.dot(rotation_matrix.T, vector_origin_to_joint)
    #     angle = np.arctan2(vector_in_plane[1], vector_in_plane[0]) - offset
    #     return angle

    def calculate_rotatory_joint_angle(self, finger_type, finger_joint_coords, metacarpals_coords):
        if finger_type == 'index':
            idx = 1
        elif finger_type == 'middle':
            idx = 2
        elif finger_type == 'ring':
            idx = 3
        origin = finger_joint_coords[0]
        vector_origin_to_joint = finger_joint_coords[1] - origin
        vector_origin_to_metacarpal = metacarpals_coords[idx] - origin

        z_axis = np.cross(vector_origin_to_metacarpal, vector_origin_to_joint)
        z_axis /= np.linalg.norm(z_axis)

        x_axis = vector_origin_to_metacarpal / np.linalg.norm(vector_origin_to_metacarpal)
        y_axis = np.cross(z_axis, x_axis)

        rotation_matrix = np.column_stack((x_axis, y_axis, z_axis))

        vector_in_plane = np.dot(rotation_matrix.T, vector_origin_to_joint)
        angle = np.arctan2(vector_in_plane[1], vector_in_plane[0])
        
        print(f"rotatory_joint_angle {angle} for finger {finger_type}")

        return angle


    def calculate_finger_angles(self, finger_type, finger_joint_coords, metacarpals_coords, curr_angles, moving_avg_arr):
        calc_finger_angles = []
        # rotatory_joint_angle = self.calculate_rotatory_joint_angle(finger_type, finger_joint_coords, metacarpals_coords)
        rotatory_joint_angle = 0.0
        calc_finger_angles.append(rotatory_joint_angle * self.linear_scaling_factors[0])
        if finger_type == 'index':
            idx = 1
        elif finger_type == 'middle':
            idx = 2
        elif finger_type == 'ring':
            idx = 3
        angle = calculate_angle(
            metacarpals_coords[idx],
            finger_joint_coords[0],
            finger_joint_coords[1]
        )

        angle *= self.linear_scaling_factors[idx]
        if angle > 1.71:
            angle = 1.71
        elif angle < -0.296:
            angle = -0.296
        calc_finger_angles.append(angle)
        for idx in range(1, self.hand_configs['joints_per_finger']-1):
            # print(f"finger_joint_coords{idx} {finger_joint_coords[idx]}")

            angle = calculate_angle(
                finger_joint_coords[idx - 1],
                finger_joint_coords[idx],
                finger_joint_coords[idx + 1]
            )
            angle *= self.linear_scaling_factors[idx]
            if angle > 1.71:
                angle = 1.71
            elif angle < -0.274:
                angle = -0.274
            calc_finger_angles.append(angle)

        filtered_angles = self._get_filtered_angles(finger_type, calc_finger_angles, curr_angles, moving_avg_arr)
        return filtered_angles

    def calculate_joint_1_angle(self, thumb_joint_coords):

        origin = thumb_joint_coords[1]
        reference_point = thumb_joint_coords[1].copy()
        reference_point[2] += 1

        vector_origin_to_index = reference_point - origin
        vector_origin_to_thumb = thumb_joint_coords[2] - origin

        if np.linalg.norm(vector_origin_to_index) == 0 or np.linalg.norm(vector_origin_to_thumb) == 0:
            print("One of the vectors is zero, unable to compute angle.")
            return np.nan

        z_axis = np.cross(vector_origin_to_index, vector_origin_to_thumb)
        if np.linalg.norm(z_axis) == 0:
            print("Cross product resulted in zero vector; vectors might be parallel.")
            return np.nan

        z_axis /= np.linalg.norm(z_axis)
        x_axis = vector_origin_to_index / np.linalg.norm(vector_origin_to_index)
        y_axis = np.cross(z_axis, x_axis)

        rotation_matrix = np.column_stack((x_axis, y_axis, z_axis))

        vector_in_plane = np.dot(rotation_matrix.T, vector_origin_to_thumb)

        angle = np.arctan2(vector_in_plane[1], vector_in_plane[0])

        return angle

    def calculate_x_axis_angle(self, orientation):
        """
        Calculate the angle around the X-axis from a geometry_msgs/Pose object.
        """
        # print(f"orientation {orientation}")
        # Extract quaternion from the pose
        qx = orientation[0]
        qy = orientation[1]
        qz = orientation[2]
        qw = orientation[3]

        # Convert quaternion to Euler angles
        euler_angles = tf.euler_from_quaternion([qx, qy, qz, qw])

        # Euler angles: Roll (x-axis), Pitch (y-axis), Yaw (z-axis)
        roll_angle = euler_angles[0]  # Angle around the X-axis

        return roll_angle

    # def calculate_joint_3_angle(self, thumb_joint_coords, thumb_joint_orientations):
    #     # angle = self.calculate_x_axis_angle(thumb_joint_orientations[3])
    #     # origin = thumb_joint_coords[2]

    #     # vector_origin_to_tip = thumb_joint_coords[3] - origin
    #     # vector_origin_to_joint = thumb_joint_coords[1] - origin
    #     # angle = calculate_angle(thumb_joint_coords[1], origin, thumb_joint_coords[3])

    #     # z_axis = np.cross(vector_origin_to_tip, vector_origin_to_joint)
    #     # z_axis /= np.linalg.norm(z_axis)

    #     # x_axis = vector_origin_to_tip / np.linalg.norm(vector_origin_to_tip)
    #     # y_axis = np.cross(z_axis, x_axis)

    #     # rotation_matrix = np.column_stack((x_axis, y_axis, z_axis))

    #     # vector_in_plane = np.dot(rotation_matrix.T, vector_origin_to_joint)
    #     # angle = np.arctan2(vector_in_plane[1], vector_in_plane[0]) - np.pi/4

    #     if angle < 0:
    #         angle = 0
    #     angle += -1.788
    #     print(f"angle {angle}")
    #     return angle

    def calculate_joint_3_angle(self, v1, v2, v3):
        # Normalize the vectors
        v2_normalized = v2 / np.linalg.norm(v2)

        # Project v1 and v3 onto the plane perpendicular to v2
        v1_proj = v1 - np.dot(v1, v2_normalized) * v2_normalized
        v3_proj = v3 - np.dot(v3, v2_normalized) * v2_normalized

        # Normalize the projected vectors
        v1_proj_norm = v1_proj / np.linalg.norm(v1_proj)
        v3_proj_norm = v3_proj / np.linalg.norm(v3_proj)

        # Compute the angle between the projected vectors
        cos_angle = np.dot(v1_proj_norm, v3_proj_norm)
        cos_angle += 1
        print(f"angle {cos_angle}")
        return cos_angle

    def calculate_thumb_tip_angle(self, thumb_joint_coords, thumb_joint_orientations, curr_angles, moving_avg_arr):
        angle = self.calculate_joint_3_angle(thumb_joint_coords[0],thumb_joint_coords[1],thumb_joint_coords[2])
        curr_angles[15] = angle * self.rotatory_thumb_scaling_factors[3]

        return curr_angles

    def calculate_joint_2_angle(self, thumb_joint_coords):

        origin = thumb_joint_coords[2]
        vector_origin_to_joint_2 = thumb_joint_coords[3] - origin
        vector_origin_to_joint_0 = thumb_joint_coords[1] - origin

        z_axis = np.cross(vector_origin_to_joint_2, vector_origin_to_joint_0)
        z_axis /= np.linalg.norm(z_axis)

        x_axis = vector_origin_to_joint_2 / np.linalg.norm(vector_origin_to_joint_2)
        y_axis = np.cross(z_axis, x_axis)

        rotation_matrix = np.column_stack((x_axis, y_axis, z_axis))

        vector_in_plane = np.dot(rotation_matrix.T, vector_origin_to_joint_0)
        angle = np.arctan2(vector_in_plane[1], vector_in_plane[0])

        return 3.14-angle

    def calculate_thumb_angles(self, index_knuckle, thumb_joint_coords, curr_angles, moving_avg_arr):

        calc_finger_angles = []
        # joint 1
        angle = self.calculate_joint_1_angle(thumb_joint_coords)
        # print(f"angle1 {angle}")
        # angle = -0.105
        # angle = 2.0
        angle -= 2.3
        # print(f"angle1 {angle}")
        # time.sleep(0.1)
        calc_finger_angles.append(angle * self.rotatory_thumb_scaling_factors[1])
        
        # joint 2
        angle = self.calculate_joint_2_angle(thumb_joint_coords)
        # angle = -0.189
        # angle = 1.644
        angle += 0.2
        # print(f"angle2 {angle}")
        calc_finger_angles.append(angle * self.rotatory_thumb_scaling_factors[2])


        # joint 3
        angle = self.calculate_joint_3_angle(thumb_joint_coords)
        # angle = -0.162
        # angle = 1.719
        angle -= 0.2
        # print(f"angle3 {angle}")
        calc_finger_angles.append(angle * self.rotatory_thumb_scaling_factors[3])
        
        # joint 0
        # 1.7 open - 2.2 closed
        # robot: 0.263 - 1.396

        angle = -calculate_angle_z(
            [1.0,0.0,0.0],
            [0.0,0.0,0.0],
            thumb_joint_coords[1]
        )
        # angle = 0.263
        # angle = 1.396
        angle += 2.8
        # print(f"angle0 {angle}")
        calc_finger_angles.append(angle * self.rotatory_thumb_scaling_factors[0])

        filtered_angles = self._get_filtered_thumb_angles("thumb", calc_finger_angles, curr_angles, moving_avg_arr)
        # print(f"filtered_angles {filtered_angles}")
        return filtered_angles



class AllegroKDLControl(AllegroKinematicControl):
    def __init__(self,  bounded_angles = True):
        super().__init__(bounded_angles)
        self.solver = AllegroKDL()
        self.ajc = AllegroJointControl()

    def calculate_desired_angles(
        self, 
        finger_type, 
        finger_joint_coords, 
        moving_avg_arr, 
        curr_angles
    ):
        tip_coord = finger_joint_coords[3]

        curr_finger_angles = self._get_curr_finger_angles(curr_angles, finger_type)  
        calc_finger_angles = self.solver.finger_inverse_kinematics(finger_type, tip_coord, curr_finger_angles)

        desired_angles = np.array(copy(curr_angles))

        # Applying angular bounds
        if self.bounded_angles is True:
            del_finger_angles = calc_finger_angles - curr_finger_angles
            clipped_del_finger_angles = np.clip(del_finger_angles, - self.bounds[finger_type], self.bounds[finger_type])
            for idx in range(self.hand_configs['joints_per_finger']):
                desired_angles[self.finger_configs['links_info'][finger_type]['offset'] + idx] += clipped_del_finger_angles[idx]
        else:
            for idx in range(self.hand_configs['joints_per_finger']):
                desired_angles[self.finger_configs['links_info'][finger_type]['offset'] + idx] = calc_finger_angles[idx]

        return desired_angles 

    def finger_1D_motion(
        self, 
        finger_type, 
        hand_y_val, 
        robot_x_val, 
        robot_y_val, 
        y_hand_bound, 
        z_robot_bound, 
        moving_avg_arr, 
        curr_angles
    ):
        '''
        For 1D control along the Z direction - used in index and middle fingers at a fixed depth and fixed y
        '''
        x_robot_coord = robot_x_val
        y_robot_coord = robot_y_val
        z_robot_coord = linear_transform(hand_y_val, y_hand_bound, z_robot_bound)
        transformed_coords = [x_robot_coord, y_robot_coord, z_robot_coord]

        desired_angles = self.calculate_desired_angles(finger_type, transformed_coords, moving_avg_arr, curr_angles)
        return desired_angles

    def finger_2D_motion(
        self, 
        finger_type, 
        hand_x_val,
        hand_y_val, 
        robot_x_val, 
        x_hand_bound, 
        y_hand_bound, 
        y_robot_bound, 
        z_robot_bound, 
        moving_avg_arr, 
        curr_angles
    ):
        '''
        For 2D control in Y and Z directions - used in ring finger at a fixed depth
        '''
        x_robot_coord = robot_x_val
        y_robot_coord = linear_transform(hand_x_val, x_hand_bound, y_robot_bound)
        z_robot_coord = linear_transform(hand_y_val, y_hand_bound, z_robot_bound)
        transformed_coords = [x_robot_coord, y_robot_coord, z_robot_coord]

        desired_angles = self.calculate_desired_angles(finger_type, transformed_coords, moving_avg_arr, curr_angles)
        return desired_angles

    def finger_2D_depth_motion(
        self, 
        finger_type, 
        hand_y_val, 
        robot_y_val, 
        hand_z_val, 
        y_hand_bound, 
        z_hand_bound, 
        x_robot_bound, 
        z_robot_bound, 
        moving_avg_arr, 
        curr_angles
    ):
        '''
        For 2D control in X and Z directions - used in index and middle fingers at a varied depth
        '''
        x_robot_coord = linear_transform(hand_z_val, z_hand_bound, x_robot_bound)
        y_robot_coord = robot_y_val
        z_robot_coord = linear_transform(hand_y_val, y_hand_bound, z_robot_bound)
        transformed_coords = [x_robot_coord, y_robot_coord, z_robot_coord]

        desired_angles = self.calculate_desired_angles(finger_type, transformed_coords, moving_avg_arr, curr_angles)
        return desired_angles

    def finger_3D_motion(
        self, 
        finger_type,
        finger_joint_coords, 
        moving_avg_arr, 
        curr_angles
    ):
        # Compute the desired joint angles based on the transformed coordinates
        return self.calculate_desired_angles(
            finger_type,
            finger_joint_coords, 
            moving_avg_arr, 
            curr_angles
        )

    # def finger_3D_motion(
    #     self, 
    #     finger_type, 
    #     hand_x_val, 
    #     hand_y_val, 
    #     hand_z_val, 
    #     x_hand_bound, 
    #     y_hand_bound, 
    #     z_hand_bound, 
    #     x_robot_bound, 
    #     y_robot_bound, 
    #     z_robot_bound, 
    #     moving_avg_arr, 
    #     curr_angles
    # ):
    #     '''
    #     For 3D control in all directions - used in ring finger at a varied depth
    #     '''
    #     x_robot_coord = linear_transform(hand_z_val, z_hand_bound, x_robot_bound)
    #     y_robot_coord = linear_transform(hand_x_val, x_hand_bound, y_robot_bound)
    #     z_robot_coord = linear_transform(hand_y_val, y_hand_bound, z_robot_bound)
    #     transformed_coords = [x_robot_coord, y_robot_coord, z_robot_coord]

    #     desired_angles = self.calculate_desired_angles(finger_type, transformed_coords, moving_avg_arr, curr_angles)
    #     return desired_angles

    def thumb_motion_2D(
        self, 
        hand_coordinates, 
        xy_hand_bounds, 
        yz_robot_bounds, 
        robot_x_val, 
        moving_avg_arr, 
        curr_angles
    ):
        '''
        For 2D control in Y and Z directions - human bounds are mapped to robot bounds
        '''
        y_robot_coord, z_robot_coord = perspective_transform(
            (hand_coordinates[0], hand_coordinates[1]), 
            xy_hand_bounds, 
            yz_robot_bounds
        )

        x_robot_coord = robot_x_val        
        transformed_coords = [x_robot_coord, y_robot_coord, z_robot_coord]
        
        desired_angles = self.calculate_desired_angles('thumb', transformed_coords, moving_avg_arr, curr_angles)
        return desired_angles


    def thumb_motion_3D(
        self, 
        thumb_joint_coords, 
        moving_avg_arr, 
        curr_angles
    ):
        # Compute the desired joint angles based on the transformed coordinates
        return self.calculate_desired_angles(
            'thumb', 
            thumb_joint_coords, 
            moving_avg_arr, 
            curr_angles
        )

        