# Basic imports
import os
import numpy as np

# Parameter management import
from hydra import compose, initialize

# Inverse Kinematics Library imports
import ikpy.chain as chain
import ikpy.link as link

from IPython import embed

class AllegroInvKDL(object):
    def __init__(self, cfg = None, urdf_path = None):
        # Importing configs from yaml file
        if cfg is None:
            # GlobalHydra.instance().clear()
            initialize(config_path = "../parameters/allegro/")
            self.cfg = compose(config_name = "allegro_link_info")
        else:
            self.cfg = cfg

        self.fingers = self.cfg.fingers

        if urdf_path is None:
            urdf_path = os.path.join(os.path.abspath(os.pardir), 'ik_stuff', 'urdf_template', 'allegro_right.urdf')
        
        self.chains = {}
        for finger in self.fingers.keys():
            self.chains[finger] = chain.Chain.from_urdf_file(urdf_path, base_elements = [self.cfg.links_info['base']['link'], self.cfg.links_info[finger]['link']], name = finger)

    def finger_forward_kinematics(self, finger_type, input_angles):
        # Checking if the number of angles is equal to 4
        if len(input_angles) != self.cfg.joints_per_finger:
            print('Incorrect number of angles')
            return 

        # Checking if the input finger type is a valid one
        if finger_type not in self.fingers.keys():
            print('Finger type does not exist')
            return
        
        # Clipping the input angles based on the finger type
        finger_info = self.cfg.links_info[finger_type]
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
        if finger_type not in self.fingers.keys():
            print('Finger type does not exist')
            return
        
        if seed is not None:
            # Checking if the number of angles is equal to 4
            if len(seed) != self.cfg.joints_per_finger:
                print('Incorrect seed array length')
                return 

            # Clipping the input angles based on the finger type
            finger_info = self.cfg.links_info[finger_type]
            for iterator in range(len(seed)):
                if seed[iterator] > finger_info['joint_max'][iterator]:
                    seed[iterator] = finger_info['joint_max'][iterator]
                elif seed[iterator] < finger_info['joint_min'][iterator]:
                    seed[iterator] = finger_info['joint_min'][iterator]

            # Padding values at the beginning and the end to get for a (1x6) array
            seed = list(seed)
            seed.insert(0, 0)
            seed.append(0)

        

        output_angles = self.chains[finger_type].inverse_kinematics(input_position, initial_position = seed)
        #if you need to visualize
        #     import ikpy.utils.plot as plot_utils
        #     import matplotlib.pyplot as plt
        #     my_chain = self.chains[finger_type]
        #     fig, ax = plot_utils.init_3d_figure()
        #     my_chain.plot(my_chain.inverse_kinematics(input_position), ax, target=input_position)
        #     plt.xlim(-0.1, 0.1)
        #     plt.ylim(-0.1, 0.1)
        #     plt.show()


                    

        return output_angles[1:5]

if __name__ == '__main__':
    allegro = AllegroInvKDL()
    embed()
