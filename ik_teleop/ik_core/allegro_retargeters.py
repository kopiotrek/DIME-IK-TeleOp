import numpy as np
from abc import ABC
from copy import deepcopy as copy
from .allegro_kdl import AllegroKDL
from  ik_teleop.teleop_utils.files import *
from  ik_teleop.teleop_utils.vectorops import *
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


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
        # print(f"[finger_type]: {finger_type}")
        # print(f"curr_angles: {curr_angles}")
        # print(f"moving_avg_arr: {moving_avg_arr}")
        
        curr_finger_angles = self._get_curr_finger_angles(curr_angles, finger_type)
        avg_finger_angles = moving_average(calc_finger_angles, moving_avg_arr, self.time_steps)       
        desired_angles = np.array(copy(curr_angles))

        
        for idx in range(1, self.hand_configs['joints_per_finger']):
                # print('config: ', self.finger_configs['links_info'][finger_type]['offset'])
                desired_angles[self.finger_configs['links_info'][finger_type]['offset'] + idx] = avg_finger_angles[idx-1]
                

        for idx in range(1, 16):
            if desired_angles[idx] > 2.5:
                desired_angles[idx] = 0
        # print(f"desired_angles: {desired_angles}")

        return desired_angles 

    def _get_filtered_thumb_angles(self, finger_type, calc_finger_angles, curr_angles, moving_avg_arr):
        # print(f"[finger_type]: {finger_type}")
        # print(f"curr_angles: {curr_angles}")
        # print(f"moving_avg_arr: {moving_avg_arr}")
        
        curr_finger_angles = self._get_curr_finger_angles(curr_angles, finger_type)
        avg_finger_angles = moving_average(calc_finger_angles, moving_avg_arr, self.time_steps)       
        desired_angles = np.array(copy(curr_angles))

        
        for idx in range(self.hand_configs['joints_per_finger']):
                # print('config: ', self.finger_configs['links_info'][finger_type]['offset'])
                desired_angles[self.finger_configs['links_info'][finger_type]['offset'] + idx] = avg_finger_angles[idx-1]
                

        # for idx in range(1, 16):
        #     if desired_angles[idx] > 2.5:
        #         desired_angles[idx] = 0
        # print(f"desired_angles: {desired_angles}")

        return desired_angles 

    def calculate_finger_angles(self, finger_type, finger_joint_coords, curr_angles, moving_avg_arr):
        # print(f"finger_joint_coords: {finger_joint_coords}")
        translatory_angles = []
        # print(f"[finger_type]: {finger_type}")

        for idx in range(self.hand_configs['joints_per_finger']-1): # Ignoring the rotatory joint
            
            
            angle = calculate_angle(
                finger_joint_coords[idx],
                finger_joint_coords[idx + 1],
                finger_joint_coords[idx + 2]
            )
            # print(f"Calculated angle for joint {idx}: {angle}")
            translatory_angles.append(angle * self.linear_scaling_factors[idx])

        # calc_finger_angles = rotatory_angle + translatory_angles
        calc_finger_angles = translatory_angles
        # calc_finger_angles = [3.0] + translatory_angles
        filtered_angles = self._get_filtered_angles(finger_type, calc_finger_angles, curr_angles, moving_avg_arr)
        return filtered_angles

    # def calculate_joint_1_angle(self, index_knuckle, thumb_joint_coords):
    #     # 1. Create plane A crossing index_knuckle,thumb_joint_coords[0],thumb_joint_coords[2]
    #     #    origin point in thumb_joint_coords[0], X axis pointing index_knuckle, Z axis being vector multiplication
    #     #    of vector origin->index_knuckle and vector origin->thumb_joint_coords[2]
    #     # 3. Calculate angle between index_knuckle,thumb_joint_coords[0],thumb_joint_coords[2] around Z axis
    #     # Step 1: Define origin and vectors for the plane
    #     origin = thumb_joint_coords[1]
    #     vector_origin_to_index = index_knuckle - origin
    #     vector_origin_to_thumb = thumb_joint_coords[3] - origin

    #     # Step 2: Create a Z-axis vector as the cross product of the two vectors
    #     z_axis = np.cross(vector_origin_to_index, vector_origin_to_thumb)
    #     z_axis /= np.linalg.norm(z_axis)  # Normalize Z-axis vector

    #     # Step 3: Project vector_origin_to_index and vector_origin_to_thumb onto the XY plane
    #     # The X-axis is aligned with vector_origin_to_index
    #     x_axis = vector_origin_to_index / np.linalg.norm(vector_origin_to_index)
    #     y_axis = np.cross(z_axis, x_axis)  # Y-axis to complete the orthogonal basis

    #     # Step 4: Construct rotation matrix from these axes
    #     rotation_matrix = np.column_stack((x_axis, y_axis, z_axis))

    #     # Step 5: Calculate the angle between the projected vectors around the Z-axis
    #     vector_in_plane = np.dot(rotation_matrix.T, vector_origin_to_thumb)
    #     angle = np.arctan2(vector_in_plane[1], vector_in_plane[0])

    #     return angle

    def calculate_joint_1_angle(self, index_knuckle, thumb_joint_coords):

        origin = thumb_joint_coords[1]
        reference_point = thumb_joint_coords[1].copy()
        reference_point[2] += 10.0
        # print(f"origin {origin}")
        # print(f"reference_point {reference_point}")
        # print(f"thumb_joint_coords[2] {thumb_joint_coords[2]}")

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


    def calculate_joint_3_angle(self, thumb_joint_coords):
        # 1. Create plane A crossing index_knuckle,thumb_joint_coords[0],thumb_joint_coords[2]
        #    origin point in thumb_joint_coords[0], X axis pointing index_knuckle, Z axis being vector multiplication
        #    of vector origin->index_knuckle and vector origin->thumb_joint_coords[2]
        # 3. Calculate angle between index_knuckle,thumb_joint_coords[0],thumb_joint_coords[2] around Z axis
        # Step 1: Define origin and vectors for the plane
        origin = thumb_joint_coords[3]

        vector_origin_to_tip = thumb_joint_coords[4] - origin
        vector_origin_to_joint = thumb_joint_coords[2] - origin

        # Step 2: Create a Z-axis vector as the cross product of the two vectors
        z_axis = np.cross(vector_origin_to_tip, vector_origin_to_joint)
        z_axis /= np.linalg.norm(z_axis)  # Normalize Z-axis vector

        # Step 3: Project vector_origin_to_index and vector_origin_to_thumb onto the XY plane
        # The X-axis is aligned with vector_origin_to_index
        x_axis = vector_origin_to_tip / np.linalg.norm(vector_origin_to_tip)
        y_axis = np.cross(z_axis, x_axis)  # Y-axis to complete the orthogonal basis

        # Step 4: Construct rotation matrix from these axes
        rotation_matrix = np.column_stack((x_axis, y_axis, z_axis))

        # Step 5: Calculate the angle between the projected vectors around the Z-axis
        vector_in_plane = np.dot(rotation_matrix.T, vector_origin_to_joint)
        angle = 3.14 - np.arctan2(vector_in_plane[1], vector_in_plane[0])
        if angle < 0:
            angle = 0
        return angle


    def calculate_joint_2_angle(self, thumb_joint_coords):

        # angle = calculate_angle(
        #     thumb_joint_coords[2],
        #     thumb_joint_coords[0],
        #     index_knuckle
        # )

        origin = thumb_joint_coords[2]
        vector_origin_to_joint_2 = thumb_joint_coords[3] - origin
        vector_origin_to_joint_0 = thumb_joint_coords[1] - origin

        # Step 2: Create a Z-axis vector as the cross product of the two vectors
        z_axis = np.cross(vector_origin_to_joint_2, vector_origin_to_joint_0)
        z_axis /= np.linalg.norm(z_axis)  # Normalize Z-axis vector

        # Step 3: Project vector_origin_to_index and vector_origin_to_thumb onto the XY plane
        # The X-axis is aligned with vector_origin_to_index
        x_axis = vector_origin_to_joint_2 / np.linalg.norm(vector_origin_to_joint_2)
        y_axis = np.cross(z_axis, x_axis)  # Y-axis to complete the orthogonal basis

        # Step 4: Construct rotation matrix from these axes
        rotation_matrix = np.column_stack((x_axis, y_axis, z_axis))

        # Step 5: Calculate the angle between the projected vectors around the Z-axis
        vector_in_plane = np.dot(rotation_matrix.T, vector_origin_to_joint_0)
        angle = np.arctan2(vector_in_plane[1], vector_in_plane[0])

        return 3.14-angle

    def calculate_thumb_angles(self, index_knuckle, thumb_joint_coords, curr_angles, moving_avg_arr):
#   thumb:
#     name: 'Thumb'
#     link: 'joint_12.0'
#     offset: 12
#     joint_min: 
#       - 0.263 
#       - -0.105
#       - -0.189
#       - -0.162
#     joint_max: 
#       - 1.396
#       - 2
#       - 1.644
#       - 1.719

# angle1 2.0748385330232537
# angle2 2.770279925046859
# angle3 0.27820246207554605
# angle4 0.14607366505253935

        calc_finger_angles = []
        # joint 1
        angle = self.calculate_joint_1_angle(index_knuckle,thumb_joint_coords)
        # print(f"angle1 {angle}")

        # angle = calculate_angle_z(
        #     thumb_joint_coords[2],
        #     thumb_joint_coords[0],
        #     index_knuckle
        # )
        # angle = -0.105
        # angle = 2.0
        angle -= 0.6
        # print(f"angle1 {angle}")
        # time.sleep(0.1)
        calc_finger_angles.append(angle * self.rotatory_thumb_scaling_factors[1])
        
        # joint 2
        angle = self.calculate_joint_2_angle(thumb_joint_coords)
        # angle = -0.189
        # angle = 1.644
        angle += 0.2
        print(f"angle2 {angle}")
        calc_finger_angles.append(angle * self.rotatory_thumb_scaling_factors[2])


        # joint 3
        angle = self.calculate_joint_3_angle(thumb_joint_coords)
        # angle = -0.162
        # angle = 1.719
        angle -= 0.2
        # print(f"angle3 {angle}")
        calc_finger_angles.append(angle * self.rotatory_thumb_scaling_factors[3])
        
        # joint 0
        angle = calculate_angle_z(
            [1.0,0.0,0.0],
            [0.0,0.0,0.0],
            thumb_joint_coords[2]
        )
        # angle = 0.263
        # angle = 1.396
        angle -= 1.2
        # print(f"angle0 {angle}")
        calc_finger_angles.append(angle * self.rotatory_thumb_scaling_factors[0])

        filtered_angles = self._get_filtered_thumb_angles("thumb", calc_finger_angles, curr_angles, moving_avg_arr)
        # print(f"filtered_angles {filtered_angles}")
        return filtered_angles


    # def calculate_finger_rotation(self, finger_joint_coords):
    #     # print(finger_joint_coords)
    #     angle = calculate_angle(finger_joint_coords[0], finger_joint_coords[1], finger_joint_coords[2])
        
    #     # Checking if the finger tip is on the left side or the right side of the knuckle
    #     knuckle_vector = finger_joint_coords[1] - finger_joint_coords[0]
    #     tip_vector = finger_joint_coords[-1] - finger_joint_coords[0]
    #     knuckle_vector_slope = knuckle_vector[1] / knuckle_vector[0]
    #     tip_vector_slope = tip_vector[1] / tip_vector[0]

    #     if knuckle_vector_slope > tip_vector_slope:
    #         return angle
    #     else:
    #         return -1 * angle


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
        # print(f"finger_joint_coords {finger_joint_coords}")

        curr_finger_angles = self._get_curr_finger_angles(curr_angles, finger_type)  

        # avg_finger_coords = moving_average(tip_coord, moving_avg_arr, self.time_steps)    
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

        # print(f"desired_angles{desired_angles}")
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
        hand_x_val, 
        hand_y_val, 
        hand_z_val, 
        x_hand_bound, 
        y_hand_bound, 
        z_hand_bound, 
        x_robot_bound, 
        y_robot_bound, 
        z_robot_bound, 
        moving_avg_arr, 
        curr_angles
    ):
        '''
        For 3D control in all directions - used in ring finger at a varied depth
        '''
        x_robot_coord = linear_transform(hand_z_val, z_hand_bound, x_robot_bound)
        y_robot_coord = linear_transform(hand_x_val, x_hand_bound, y_robot_bound)
        z_robot_coord = linear_transform(hand_y_val, y_hand_bound, z_robot_bound)
        transformed_coords = [x_robot_coord, y_robot_coord, z_robot_coord]

        desired_angles = self.calculate_desired_angles(finger_type, transformed_coords, moving_avg_arr, curr_angles)
        return desired_angles

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


    # def thumb_motion_3D(
    #     self, 
    #     hand_coordinates, 
    #     xy_hand_bounds,  # Now a list of points, not a Polygon
    #     yz_robot_bounds, 
    #     z_hand_bound, 
    #     x_robot_bound, 
    #     moving_avg_arr, 
    #     curr_angles
    # ):
    #     # Apply perspective transformation to obtain robot coordinates
    #     y_robot_coord, z_robot_coord = perspective_transform(
    #         hand_coordinates[:2],  # 2D hand coordinates (x, y)
    #         xy_hand_bounds,  # Ensure this is in the correct format (np.float32)
    #         yz_robot_bounds
    #     )
        
    #     # Perform a linear transform for the z-axis
    #     x_robot_coord = linear_transform(hand_coordinates[2], z_hand_bound, x_robot_bound)
        
    #     # Combine the transformed coordinates
    #     transformed_coords = [x_robot_coord, y_robot_coord, z_robot_coord]
        
    #     # Compute the desired joint angles based on the transformed coordinates
    #     return self.calculate_desired_angles(
    #         'thumb', 
    #         transformed_coords, 
    #         moving_avg_arr, 
    #         curr_angles
    #     )

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

        