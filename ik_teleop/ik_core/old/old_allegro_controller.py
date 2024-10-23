# Standard imports
import os
import numpy as np

# Copying imports
from copy import deepcopy as copy

# Parameter management import
from hydra import compose, initialize

# Allegro Inverse Kinematics library
from ik_teleop.ik_core.allegro_ik import AllegroInvKDL

# Debugging tools
from IPython import embed 

class AllegroIKController (object):
    def __init__(self, cfg = None, urdf_path=None):
        # To ignore scientific notations
        np.set_printoptions(suppress = True)

        # Importing configs from yaml file
        if cfg is None:
            initialize(config_path = "../parameters/allegro")
            self.cfg = compose(config_name = "allegro_bounds")
        else:
            self.cfg = cfg
        
        # Initializing the allegro inverse kinematics object
        self.allegro_ik = AllegroInvKDL(self.cfg, urdf_path)

        # Getting the allegro finger info for the offsets
        self.fingers = self.cfg.fingers

        # Getting the joint bounds to limit the range of motion
        self.joint_bounds = self.cfg.jointwise_angle_bounds

        # Setting the moving average limit parameter
        self.time_steps = self.cfg.time_steps
        print("Using {} time steps for moving average.".format(self.time_steps))

    def bounded_finger_motion(self, finger_type, coordinate, moving_average_array, current_angles):
        # Initializing a seed from the current angles
        current_finger_angles = np.array(current_angles[self.fingers[finger_type].offset : self.fingers[finger_type].offset + self.cfg.joints_per_finger])

        # Calculate the moving average on the coordinates
        moving_average_array.append(copy(coordinate))

        if len(moving_average_array) > self.cfg.time_steps:
            moving_average_array.pop(0)

        averaged_finger_tip_coords = np.mean(np.array(moving_average_array), 0)

        # Calculating the angles using the inverse kinematics function
        calculated_average_angles = self.allegro_ik.finger_inverse_kinematics(finger_type, averaged_finger_tip_coords, current_finger_angles)

        # Calculating the angle change
        calculated_angle_change = np.array(calculated_average_angles) - np.array(current_finger_angles)

        # Bounding the corresponding angle changes
        finger_bounds = np.array(self.joint_bounds[self.fingers[finger_type].offset : self.fingers[finger_type].offset + self.cfg.joints_per_finger])
        clipped_angle_changes = np.clip(calculated_angle_change, -finger_bounds, finger_bounds)

        # Updating the angle changes in the final angle array
        desired_angles = copy(current_angles)

        for idx in range(self.cfg.joints_per_finger):
            desired_angles[self.fingers[finger_type].offset + idx] += clipped_angle_changes[idx]
        return desired_angles

    def bounded_linear_finger_motion(self, finger_type, x_val, y_val, z_val, z_bound_array, target_z_bound_array, moving_average_array, current_angles):
        current_finger_angles = np.array(current_angles[self.fingers[finger_type].offset : self.fingers[finger_type].offset + self.cfg.joints_per_finger])

        # Calculating the target coordinate using linear transformation
        target_z_val = (z_val - z_bound_array[0]) * ((target_z_bound_array[-1] - target_z_bound_array[0])/(z_bound_array[-1] - z_bound_array[0])) + target_z_bound_array[0]

        target_coordinate = [x_val, y_val, target_z_val]
        
        # calculating the moving average
        moving_average_array.append(copy(target_coordinate))

        if len(moving_average_array) > self.time_steps:
            moving_average_array.pop(0)

        averaged_finger_coordinate = np.mean(np.array(moving_average_array), 0)
        
        # Calculating the desired angle based on the target coordinate using the inverse kinematics function
        calculated_average_angles = self.allegro_ik.finger_inverse_kinematics(finger_type, averaged_finger_coordinate, current_finger_angles)

        # Calculating the angle change
        calculated_angle_change = np.array(calculated_average_angles) - np.array(current_finger_angles)

        # Bounding the corresponding angle changes
        finger_bounds = np.array(self.joint_bounds[self.fingers[finger_type].offset : self.fingers[finger_type].offset + self.cfg.joints_per_finger])
        clipped_angle_changes = np.clip(calculated_angle_change, -finger_bounds, finger_bounds)

        # Updating the angle changes in the final angle array
        desired_angles = copy(current_angles)

        for idx in range(self.cfg.joints_per_finger):
            desired_angles[self.fingers[finger_type].offset + idx] += clipped_angle_changes[idx]

        return desired_angles

    def bounded_linear_fingers_motion(self, finger_type, x_val, y_val, z_val, y_bound_array, z_bound_array, target_y_bound_array, target_z_bound_array, moving_average_array, current_angles):
        # Initializing a seed from the current angles
        current_finger_angles = np.array(current_angles[self.fingers[finger_type].offset : self.fingers[finger_type].offset + self.cfg.joints_per_finger])

        # Calculating the target coordinate using linear transformation
        target_y_val = (y_val - y_bound_array[0]) * ((target_y_bound_array[-1] - target_y_bound_array[0])/(y_bound_array[-1] - y_bound_array[0])) + target_y_bound_array[0]
        target_z_val = (z_val - z_bound_array[0]) * ((target_z_bound_array[-1] - target_z_bound_array[0])/(z_bound_array[-1] - z_bound_array[0])) + target_z_bound_array[0]

        target_coordinate = [x_val, target_y_val, target_z_val]

        # Calculating the moving average
        moving_average_array.append(copy(target_coordinate))

        if len(moving_average_array) > self.time_steps:
            moving_average_array.pop(0)

        averaged_finger_coordinate = np.mean(np.array(moving_average_array), 0)


        # Calculating the desired angle based on the target coordinate using the inverse kinematics function
        calculated_average_angles = self.allegro_ik.finger_inverse_kinematics(finger_type, averaged_finger_coordinate, current_finger_angles)

        # Calculating the angle change
        calculated_angle_change = np.array(calculated_average_angles) - np.array(current_finger_angles)

        # Bounding the corresponding angle changes
        finger_bounds = np.array(self.joint_bounds[self.fingers[finger_type].offset : self.fingers[finger_type].offset + self.cfg.joints_per_finger])
        clipped_angle_changes = np.clip(calculated_angle_change, -finger_bounds, finger_bounds)

        # Updating the angle changes in the final angle array
        desired_angles = copy(current_angles)

        for idx in range(self.cfg.joints_per_finger):
            desired_angles[self.fingers[finger_type].offset + idx] += clipped_angle_changes[idx]

        return desired_angles


    def bounded_all_fingers_motion(self, coordinate_array, moving_average_arrays, current_angles):
        # Finding the desired angles for each finger using the given coordinates
        desired_angles = copy(current_angles)
        for idx, finger in enumerate(self.fingers.keys()):
            desired_angles = self.bounded_finger_motion(finger, coordinate_array[idx], moving_average_arrays[idx], desired_angles)

        return desired_angles

    def direct_finger_motion(self, finger_type, coordinate, moving_average_array, current_angles):
        # Initializing a seed from the current angles
        seed = np.array(current_angles[self.fingers[finger_type].offset : self.fingers[finger_type].offset + self.cfg.joints_per_finger])

        # Calculating the moving average
        moving_average_array.append(copy(coordinate))

        if len(moving_average_array) > self.cfg.time_steps:
            moving_average_array.pop(0)

        averaged_finger_coordinate = np.mean(np.array(moving_average_array), 0)

        # Calculating the angles using the inverse kinematics function
        calculated_average_angles = self.allegro_ik.finger_inverse_kinematics(finger_type, averaged_finger_coordinate, seed)

        # Updating the angle changes in the final angle array
        desired_angles = copy(current_angles)

        for idx in range(self.cfg.joints_per_finger):
            desired_angles[self.fingers[finger_type].offset + idx] = calculated_average_angles[idx]

        return desired_angles

    def direct_linear_fingers_motion(self, finger_type, x_val, y_val, z_val, y_bound_array, z_bound_array, target_y_bound_array, target_z_bound_array, moving_average_array, current_angles):
        # Initializing a seed from the current angles
        current_finger_angles = np.array(current_angles[self.fingers[finger_type].offset : self.fingers[finger_type].offset + self.cfg.allegro.joints_per_finger])

        # Calculating the target coordinate using linear transformation
        target_y_val = (y_val - y_bound_array[0]) * ((target_y_bound_array[-1] - target_y_bound_array[0])/(y_bound_array[-1] - y_bound_array[0])) + target_y_bound_array[0]
        target_z_val = (z_val - z_bound_array[0]) * ((target_z_bound_array[-1] - target_z_bound_array[0])/(z_bound_array[-1] - z_bound_array[0])) + target_z_bound_array[0]

        target_coordinate = [x_val, target_y_val, target_z_val]

        # Calculating the moving average
        moving_average_array.append(copy(target_coordinate))

        if len(moving_average_array) > self.cfg.time_steps:
            moving_average_array.pop(0)

        averaged_finger_coordinate = np.mean(np.array(moving_average_array), 0)

        # Calculating the desired angle based on the target coordinate using the inverse kinematics function
        calculated_average_angles = self.allegro_ik.finger_inverse_kinematics(finger_type, averaged_finger_coordinate, current_finger_angles)

        # Updating the angle changes in the final angle array
        desired_angles = copy(current_angles)

        for idx in range(self.cfg.joints_per_finger):
            desired_angles[self.fingers[finger_type].offset + idx] = calculated_average_angles[idx]

        return desired_angles

    def direct_all_fingers_motion(self, coordinate_array, moving_average_arrays, current_angles):
        # Getting the seeds for all the fingers
        seed_array = []
        for finger in self.fingers.keys():
            seed_array.append(current_angles[self.fingers[finger].offset : self.fingers[finger].offset + self.cfg.joints_per_finger])

        # Finding the desired angles for each finger using the given coordinates
        desired_angles = copy(current_angles)
        for idx, finger in enumerate(self.fingers.keys()):
            desired_angles = self.direct_finger_motion(finger, coordinate_array[idx], moving_average_arrays[idx], desired_angles)

        return desired_angles

if __name__ == '__main__':
    allegro = AllegroIKController()
    embed()
