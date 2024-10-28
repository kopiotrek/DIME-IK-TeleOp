import rospy
import os
from std_msgs.msg import Float64MultiArray
import numpy as np
from datetime import datetime
from ik_teleop.ik_core.allegro_retargeters import AllegroKinematicControl, AllegroJointControl, AllegroKDL
from ik_teleop.ik_core.allegro_operator import AllegroHandOperator
from mjrl.utils.gym_env import GymEnv
from ik_teleop.teleop_utils.files import *
import mj_allegro_envs


class TeleOpSim(object):
    def __init__(self):
    # def __init__(self, record_demo=False, hide_window=False, cfg=None, enable_moving_average=True):
        # Initialize ROS subscriber to get 3D hand knuckle coordinates
        if not rospy.core.is_initialized():
            # rospy.init_node('allegro_hand_operator', anonymous=True)
            try:
                rospy.init_node('simulation')
            except rospy.ROSException as e:
                print(f'Node initialization failed: {str(e)}')
                pass
        self.record_demo = False
        self.display_window = True
        self.desired_joint_angles = np.array([0.2, 0.28113237, 0.16851817, 0.2, 0.2, 0.17603329, 
            0.21581194, 0.2, 0.2928223, 0.16747166, 1.45242466, 1.45812127, 0.69531447, 1.1, 1.1, 1.1])
        
        self.env = GymEnv('block-v3')
        initial_env = self.env.reset()

        # Initialize AllegroKDL for inverse kinematics
        self.allegroKDL = AllegroKDL()
        self.allegroJC = AllegroJointControl()
        self.allegroKC = AllegroKinematicControl()
        self.allegro_hand_config = get_yaml_data('configs/allegro_sim.yaml')

        self.allegro_hand_operator = AllegroHandOperator(self.allegro_hand_config)
        
        rospy.Subscriber('/transformed_hand_coords', Float64MultiArray, self._callback_knuckle_coordinates, queue_size=1)
        
        # Demo setup
        if not os.path.isdir('demos'):
            os.mkdir('demos')
        t = datetime.now()
        date_str = t.strftime('%b_%d_%H_%M')

        self.obs_freq = 1
        self.obs_ctr = 0
        self.demo_dir = os.path.join('demos', f"demo_{date_str}")
        if self.record_demo and not os.path.isdir(self.demo_dir):
            os.mkdir(self.demo_dir)
        self.pkl_file = os.path.join(self.demo_dir, f'd_{date_str}.pickle')

        self.demo_dict = {}
        self.current_state = None
        self.set_init_state = False
        self.current_states = []
        if self.record_demo:
            self.initialize_demo_dict(initial_env)

    def _callback_knuckle_coordinates(self, msg):
        # Extract the 21 3D coordinates from the received message (21 x 3 = 63 elements)
        # joints_coords = np.array(msg.data).reshape(21, 3)

        # # Map relevant knuckle coordinates to fingertips (you might have to adjust these indices)
        # index_tip_coord = joints_coords[8]  # Adjust index as per your knuckle mapping
        # middle_tip_coord = joints_coords[12]
        # ring_tip_coord = joints_coords[16]
        # thumb_tip_coord = joints_coords[4]

        # Compute the desired joint angles using inverse kinematics
        seed_angles = self.desired_joint_angles  # Use the current joint angles as the seed
        # self.desired_joint_angles = self.allegroKDL.get_joint_state_from_coord(
        #     index_tip_coord, middle_tip_coord, ring_tip_coord, thumb_tip_coord, seed_angles
        # )
        self.desired_joint_angles = self.allegro_hand_operator._apply_retargeted_angles()

    def teleop_loop(self):
        while not rospy.is_shutdown():
            if self.desired_joint_angles is not None:
                # Send the desired joint angles to the simulation environment
                self.obs_ctr += 1
                obs, reward, done, info = self.env.step(self.desired_joint_angles)
    
                # Optionally record the demo data
                if self.record_demo and self.obs_ctr % self.obs_freq == 0:
                    self.add_demo_entry(self.env, self.desired_joint_angles, obs, reward, done, info)
    
                # #check if block needs to be reset
                # num_resets = 0
                # while(not self.env.env.is_on_palm()):
                #     num_resets += 1
                #     if(num_resets <= 5):
                #         joints = np.ones(16) *0.2
                #         for i in range(20):
                #             obs,reward, done, info = self.env.env.step(joints)
                #             if(self.display_window): self.env.env.render()
                        
                #         for i in range(5):
                #             self.env.env.reset_model()
                #             if(self.display_window): self.env.render()
                #         num_resets = 0
                #     print('RESETTING BLOCK')
                #     import pdb
                #     # pdb.set_trace()
                #     self.env.env.reset_model()
                # # print("joints")
                # print(self.current_joint_state)
                # for i in range(10):
                obs,reward, done, info = self.env.step(self.desired_joint_angles)   

                # Printing the image
                if(self.display_window):
                    self.env.render()
                    
if __name__ == '__main__':
    main = TeleOpSim()
    # main.allegro_hand_operator._calibrate_bounds()

    main.teleop_loop()

