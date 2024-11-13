from ikpy import chain
import numpy as np
from copy import deepcopy as copy
from ik_teleop.teleop_utils.files import *
from ik_teleop.teleop_utils.constants import *
from threading import Thread
import time
import pickle
from io import StringIO
import sys

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
        # Load or initialize the IK cache
        self.cache_file_path = "ik_cache.pkl"
        self.ik_cache = self.load_cache()
        self.chains["thumb"].links.pop(5)
        self.chains["thumb"].links.pop(4)
        self.chains["thumb"].active_links_mask = np.delete(self.chains["thumb"].active_links_mask, 5)
        self.chains["thumb"].active_links_mask = np.delete(self.chains["thumb"].active_links_mask, 4)
        self.last_knuckle_angles = [0.0, 0.43077692, 0.08167671, 0.81602719, 0.00001407, 0.0]

       
    def load_cache(self):
        # Load the cache from a file if it exists; otherwise, return an empty dictionary
        if os.path.exists(self.cache_file_path):
            with open(self.cache_file_path, "rb") as file:
                print("Loading IK cache from file.")
                return pickle.load(file)
        print("Initializing new IK cache.")
        return {}

    def save_cache(self):
        # Save the cache to a file
        with open(self.cache_file_path, "wb") as file:
            pickle.dump(self.ik_cache, file)

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

    def finger_inverse_kinematics(self, finger_type, raw_tip_coord, seed):
        # print(f"finger_inverse_kinematics seed{seed}")

        tip_coord = tuple(round(p,3) for p in raw_tip_coord)
        # Validate the finger type
        if finger_type not in self.hand_configs['fingers']:
            print('Finger type does not exist')
            return

        # Clipping and adjusting seed
        if seed is not None:
            if len(seed) != self.hand_configs['joints_per_finger']:
                print('Incorrect seed array length')
                return 
            # seed = np.concatenate(([0.0], seed, [0.0]))
            seed = np.concatenate(([0.0], seed))
            # seed[1] = 0.263

            # finger_info = self.finger_configs['links_info'][finger_type]
            # seed = [min(max(s, min_val), max_val) 
            #         for s, min_val, max_val in zip(seed, finger_info['joint_min'], finger_info['joint_max'])]
            # seed = [0] + seed + [0]

        # Check cache first
        cache_key = (tuple(tip_coord))
        start_time = time.time()
        if cache_key in self.ik_cache:
            print("Cache used")
            elapsed_time = time.time() - start_time
            print(f"Time taken for cache_key: {elapsed_time:.6f} seconds")

            return self.ik_cache[cache_key][1:5]
        else:
            output_angles = self.ik_with_timeout(self.chains[finger_type], tip_coord, seed)
            # output_angles = self.chains[finger_type].inverse_kinematics(
            #     tip_coord,
            #     initial_position=seed,
            #     orientation_mode=None,
            #     max_iter=1
            # )
            self.ik_cache[cache_key] = output_angles
            self.save_cache()  # Update cache file after every IK computation
            return output_angles[1:4]


    def ik_with_timeout(self, chain, tip_coord, seed, timeout=2.4):
        # Wrapper to hold the result and control completion status
        result = {"angles": None, "completed": False}
        seed = np.delete(seed, 4)
        # seed = np.delete(seed, 3)

        print(f"seed{seed}")
        print(f"tip_coord{tip_coord}")
        tip_coord = list(tip_coord)  # Convert tuple to list
        tip_coord[0] = tip_coord[0] * 0.5  # Modify the value
        tip_coord = tuple(tip_coord)  # Convert back to tuple if necessary
        print(f"tip_coord{tip_coord}")
        original_stdout = sys.stdout
        sys.stdout = StringIO()  # Redirect stdout to a dummy StringIO object
        def run_ik():
            # print(f"links{chain.links}")
            # print(f"active_links_mask{chain.active_links_mask}")
            # print(f"name{chain.name}")
            # print(f"urdf_metadata{chain.urdf_metadata}")
            result["angles"] = chain.inverse_kinematics(
                tip_coord,
                initial_position=seed,
                orientation_mode=None,
                # max_iter=1,
                # regularization_parameter=0.005
            )
            result["completed"] = True
        sys.stdout = original_stdout

        # Start the IK calculation in a separate thread
        ik_thread = Thread(target=run_ik)
        start_time = time.time()
        ik_thread.start()
        ik_thread.join(timeout)  # Wait for IK to complete or timeout
        elapsed_time = time.time() - start_time
        print(f"Time taken for IK operation: {elapsed_time:.6f} seconds")

        # print(f"tip_coord{tip_coord}")
        # print(f"seed{seed}")
        # print(f"result[angles]{result['angles']}")

        # Check if the thread completed within the timeout
        if not result["completed"]:
            print("IK computation exceeded time limit.")
            return self.last_knuckle_angles
        self.last_knuckle_angles = result["angles"]
        return result["angles"]

    # def finger_inverse_kinematics(self, finger_type, input_position, seed = None):
    #     # Checking if the input figner type is a valid one
    #     if finger_type not in self.hand_configs['fingers'].keys():
    #         print('Finger type does not exist')
    #         return
        
    #     if seed is not None:
    #         # Checking if the number of angles is equal to 4
    #         if len(seed) != self.hand_configs['joints_per_finger']:
    #             print('Incorrect seed array length')
    #             return 

    #         # Clipping the input angles based on the finger type
    #         finger_info = self.finger_configs['links_info'][finger_type]
    #         for iterator in range(len(seed)):
    #             if seed[iterator] > finger_info['joint_max'][iterator]:
    #                 # print(f"joint_max for {iterator} reached!")
    #                 seed[iterator] = finger_info['joint_max'][iterator]
    #             elif seed[iterator] < finger_info['joint_min'][iterator]:
    #                 # print(f"seed={seed[iterator]}joint_min for {iterator} reached!")
    #                 seed[iterator] = finger_info['joint_min'][iterator]

    #         # Padding values at the beginning and the end to get for a (1x6) array
    #         seed = list(seed)
    #         seed.insert(0, 0)
    #         seed.append(0)
    #     # print(self.chains[finger_type])
    #     start_time = time.time()
    #     output_angles = self.chains[finger_type].inverse_kinematics(input_position, 
    #     initial_position = seed,
    #     max_iter = 3,
    #     orientation_mode = None
    #     )
    #     elapsed_time = time.time() - start_time
    #     print(f"Time taken for IK operation: {elapsed_time:.6f} seconds")
    #     # print(f"output_angles{output_angles}")

    #     return output_angles[1:5]

    def get_fingertip_coords(self, joint_positions):
        # index_coords = self.finger_forward_kinematics('index', joint_positions[:4])[0]
        # middle_coords = self.finger_forward_kinematics('middle', joint_positions[4:8])[0]
        # ring_coords = self.finger_forward_kinematics('ring', joint_positions[8:12])[0]
        # thumb_coords = self.finger_forward_kinematics('thumb', joint_positions[12:16])[0]
        
        index_coords = self.finger_forward_kinematics('index', joint_positions[OCULUS_JOINTS['index']])[0]
        middle_coords = self.finger_forward_kinematics('middle', joint_positions[OCULUS_JOINTS['middle']])[0]
        ring_coords = self.finger_forward_kinematics('ring', joint_positions[OCULUS_JOINTS['ring']])[0]
        thumb_coords = self.finger_forward_kinematics('thumb', joint_positions[OCULUS_JOINTS['thumb']])[0]


        finger_tip_coords = np.hstack([index_coords, middle_coords, ring_coords, thumb_coords])
        return np.array(finger_tip_coords)

    def get_joint_state_from_coord(self, index_tip_coord, middle_tip_coord, ring_tip_coord, thumb_tip_coord, seed):
        # print(seed)
        index_joint_angles = self.finger_inverse_kinematics('index', index_tip_coord, seed[0:4])
        middle_joint_angles = self.finger_inverse_kinematics('middle', middle_tip_coord, seed[4:8])
        ring_joint_angles = self.finger_inverse_kinematics('ring', ring_tip_coord, seed[8:12])
        thumb_joint_angles = self.finger_inverse_kinematics('thumb', thumb_tip_coord, seed[12:16])

        desired_joint_angles = copy(seed)
        
        for idx in range(4):
            desired_joint_angles[idx] = index_joint_angles[idx]
            desired_joint_angles[4 + idx] = middle_joint_angles[idx]
            desired_joint_angles[8 + idx] = ring_joint_angles[idx]
            desired_joint_angles[12 + idx] = thumb_joint_angles[idx]

        return desired_joint_angles
    