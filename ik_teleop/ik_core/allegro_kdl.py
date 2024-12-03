from ikpy import chain
import numpy as np
from copy import deepcopy as copy
from ik_teleop.teleop_utils.files import *
from ik_teleop.teleop_utils.constants import *
from ik_teleop.teleop_utils.vectorops import *
from ik_teleop.ik_core.allegro_thumb_ik import ThumbIK 

class AllegroKDL(object):
    def __init__(self):
        # Getting the URDF path
        urdf_path = get_path_in_package("robot/assets/allegro_hand_right.urdf")
        # urdf_path = get_path_in_package("robot/assets/allegro_hand_right.urdf")

        # Loading Allegro Hand configs
        self.hand_configs = get_yaml_data(get_path_in_package("robot/allegro/configs/allegro_info.yaml"))
        # self.hand_configs = get_yaml_data(get_path_in_package("robot/allegro/configs/allegro_info.yaml"))
        self.finger_configs = get_yaml_data(get_path_in_package("robot/allegro/configs/allegro_link_info.yaml"))
        # self.finger_configs = get_yaml_data(get_path_in_package("robot/allegro/configs/allegro_link_info.yaml"))

        # Parsing chains from the urdf file
        self.chains = {}
        for finger in self.hand_configs['fingers'].keys():
            self.chains[finger] = chain.Chain.from_urdf_file(
                urdf_path, 
                base_elements = [
                    self.finger_configs['links_info']['base']['link'], 
                    self.finger_configs['links_info'][finger]['link']
                ], 
                name = finger
            )
        self.ik = ThumbIK()
    
    def finger_forward_kinematics(self, finger_type, input_angles):
        # Checking if the number of angles is equal to 4
        if len(input_angles) != self.hand_configs['joints_per_finger']:
            print('Incorrect number of angles')
            return 

        # Checking if the input finger type is a valid one
        if finger_type not in self.hand_configs['fingers'].keys():
            print('Finger type does not exist')
            return
        
        # Clipping the input angles based on the finger type
        finger_info = self.finger_configs['links_info'][finger_type]
        for iterator in range(len(input_angles)):
            if input_angles[iterator] > finger_info['joint_max'][iterator]:
                input_angles[iterator] = finger_info['joint_max'][iterator]
            elif input_angles[iterator] < finger_info['joint_min'][iterator]:
                input_angles[iterator] = finger_info['joint_min'][iterator]

        # Padding values at the beginning and the end to get for a (1x6) array
        input_angles = list(input_angles)
        input_angles.insert(0, 0)
        input_angles.append(0)

        # Performing Forward Kinematics 
        output_frame = self.chains[finger_type].forward_kinematics(input_angles)
        return output_frame[:3, 3], output_frame[:3, :3]

    def finger_inverse_kinematics(self, finger_type, input_position, seed = None):
        # Checking if the input figner type is a valid one
        if finger_type not in self.hand_configs['fingers'].keys():
            print('Finger type does not exist')
            return

        print(f"input_position {input_position}")
        rotation_angles = (np.pi + np.deg2rad(5), 0, 0)
        input_position += [0.0182, -0.016958, 0.073288]
        input_position += [-0.02, 0, 0]
        input_position[0] *= 1.4
        input_position[1] *= 1.0
        input_position[2] *= 1.0
        # rotation_angles = (-np.pi, -np.pi/2, np.deg2rad(5)+np.pi)
        input_position = rotate_point(input_position, rotation_angles)
        # input_position = [-0.01344712, -0.00566355, -0.03781227]

        print(f"input_position trans{input_position}")
        # TEE for fully extended thumb to the side    
        # [0.0431817626,−0.160388732,0.0000003505]
        if seed is not None:
            # Checking if the number of angles is equal to 4
            if len(seed) != self.hand_configs['joints_per_finger']:
                print('Incorrect seed array length')
                return 

            # Clipping the input angles based on the finger type
            finger_info = self.finger_configs['links_info'][finger_type]
            for iterator in range(len(seed)):
                if seed[iterator] > finger_info['joint_max'][iterator]:
                    seed[iterator] = finger_info['joint_max'][iterator]
                elif seed[iterator] < finger_info['joint_min'][iterator]:
                    seed[iterator] = finger_info['joint_min'][iterator]

            # Padding values at the beginning and the end to get for a (1x6) array
            seed = list(seed)
            seed.insert(0, 0)
            seed.append(0)
        # output_angles = self.chains[finger_type].inverse_kinematics(input_position, initial_position = seed)
        output_angles = self.ik.compute_ik(input_position)
        print("Computed Joint Angles (IK):", output_angles)
        return output_angles[0:4]

        

    def get_fingertip_coords(self, joint_positions):
        # index_coords = self.finger_forward_kinematics('index', joint_positions[:4])[0]
        # middle_coords = self.finger_forward_kinematics('middle', joint_positions[4:8])[0]
        # ring_coords = self.finger_forward_kinematics('ring', joint_positions[8:12])[0]
        # thumb_coords = self.finger_forward_kinematics('thumb', joint_positions[12:16])[0]
        
        index_coords = self.finger_forward_kinematics('index', joint_positions[OCULUS_JOINTS['index']])[0]
        middle_coords = self.finger_forward_kinematics('middle', joint_positions[OCULUS_JOINTS['middle']])[0]
        ring_coords = self.finger_forward_kinematics('ring', joint_positions[OCULUS_JOINTS['ring']])[0]
        thumb_coords = self.finger_forward_kinematics('thumb', joint_positions[OCULUS_JOINTS['thumb']])[0]
        # thumb_coords = self.finger_inverse_kinematics('thumb', joint_positions[OCULUS_JOINTS['thumb']])[0]


        finger_tip_coords = np.hstack([index_coords, middle_coords, ring_coords, thumb_coords])
        return np.array(finger_tip_coords)

    def get_joint_state_from_coord(self, index_tip_coord, middle_tip_coord, ring_tip_coord, thumb_tip_coord, seed):
        # print(seed)
        # index_joint_angles = self.finger_inverse_kinematics('index', index_tip_coord, seed[0:4])
        # middle_joint_angles = self.finger_inverse_kinematics('middle', middle_tip_coord, seed[4:8])
        # ring_joint_angles = self.finger_inverse_kinematics('ring', ring_tip_coord, seed[8:12])
        index_joint_angles = [0,0,0,0]
        middle_joint_angles = [0,0,0,0]
        ring_joint_angles = [0,0,0,0]
        thumb_joint_angles = self.finger_inverse_kinematics('thumb', thumb_tip_coord, seed[12:16])

        desired_joint_angles = copy(seed)
        
        for idx in range(4):
            desired_joint_angles[idx] = index_joint_angles[idx]
            desired_joint_angles[4 + idx] = middle_joint_angles[idx]
            desired_joint_angles[8 + idx] = ring_joint_angles[idx]
            desired_joint_angles[12 + idx] = thumb_joint_angles[idx]

        return desired_joint_angles
    

# if __name__ == '__main__':
#     ik_control = AllegroKDL()

#     # Set desired position and orientation
#     # output_frame = ik_control.finger_forward_kinematics('thumb',[ 0.49158065,  0.62981548, -2.99420419,  3.29378238])  # Example position
#     thumb_joint_angles = ik_control.finger_inverse_kinematics('thumb', [ 0.02698166,  0.16099207, -0.07196472])
#     print(f"thumb_joint_angles {thumb_joint_angles}")
#     # ik_control.rotation = np.array([1, 0, 0, 0])     # Identity quaternion

#     # Perform the IK update
#     # ik_control.Update()