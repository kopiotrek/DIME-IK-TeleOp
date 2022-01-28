import yaml
from hydra import initialize, compose
from mjrl.utils.gym_env import GymEnv
import mj_allegro_envs
from datetime import datetime
import os
from ik_teleop.ik_core.allegro_controller import AllegroIKController
# from ik_teleop.teleop_utils.calibrate import BoundCalibrator
from copy import deepcopy as copy
import numpy as np
import cv2
from ik_teleop.utils.transformations import perform_persperctive_transformation
import sys
import pickle
import signal

URDF_PATH = "./ik_teleop/urdf_template/allegro_right.urdf"

TRANS_HAND_TIPS = {
    'thumb': 6,
    'index': 7,
    'middle': 8,
    'ring': 9,
    'pinky': 10
}

class HandSimulator:
    def __init__(self,cfg=None, env_name=None, record_demo=False, demo_dir=None):
        # Getting the configurations
        if cfg is None:
            initialize(config_path = "../parameters/")
            self.cfg = compose(config_name = "teleop")
        else:
            self.cfg = cfg
        
        if(env_name is None):
            env_name = 'block-v3'
        
        self.env = GymEnv(env_name)
        initial_env = self.env.reset()


        # Initializing calibrator and performing calibration sequence
        # self.calibrator = BoundCalibrator(storage_dir = os.getcwd())
        # self.calibrated_bounds = self.calibrator.check_and_load_calibration_file()
        allegro_calibrated_path = './ik_teleop/bound_data/calibrated_values.npy'
        allegro_bound_yaml_path = './ik_teleop/bound_data/allegro_bounds.yaml'
        self.calibrated_bounds = np.load(allegro_calibrated_path) 
        self.ik_solver = AllegroIKController(cfg = self.cfg.allegro, urdf_path = URDF_PATH)

        self.moving_average_queues = {
                'thumb': [],
                'index': [],
                'middle': [],
                'ring': []
            }

        self.hand_coords = None
        self.desired_joint_angles = np.array([0, 0.28113237, 0.16851817, 0.2, 0.2, 0.17603329, 
            0.21581194, 0.2, 0.2928223, 0.16747166, 1.45242466, 1.45812127, 0.05, 0.05,0.05,0.05])

        with open(allegro_bound_yaml_path, 'r') as file:
            self.allegro_bounds = yaml.safe_load(file)['allegro_bounds']

        #Used for recording images and states
        self.record_demo = record_demo
        t= datetime.now()
        date_str = t.strftime('%b_%d_%H_%M_%S')

        self.demo_dir = './demos/throwaway'
        if(demo_dir):
            self.demo_dir = demo_dir
        else:
            #create a new directory
            self.demo_dir = os.path.join('demos',"demo_{}".format(date_str))
            
        if(self.record_demo and not os.path.isdir(self.demo_dir)):
            if(not os.path.isdir('demos')):
                os.mkdir('demos')
            os.mkdir(self.demo_dir)
        
        self.vid_file = os.path.join(self.demo_dir, 'demo.mp4')
        self.unmrkd_file = os.path.join(self.demo_dir,'orig.mp4')
        self.pkl_file = os.path.join(self.demo_dir, 'd_{}.pickle'.format(date_str))

        self.obs_freq = 1
        self.obs_ctr = 0
        
        self.demo_dict = {}
        self.current_state = None
        self.set_init_state = False
        self.unmrkd_images = []
        self.images = []
        self.current_states = []
        if(self.record_demo):
            self.initialize_demo_dict(initial_env)
        self.write_to_file = False #whether contents have been dumped to file
        signal.signal(signal.SIGTERM, self.finish_recording)
        signal.signal(signal.SIGINT, self.finish_recording)


    def initialize_demo_dict(self, start_state):
        #TODO double check what this means...
        self.demo_dict['terminated'] = True
         #TODO initialize this before first step
        self.demo_dict['init_state_dict']= {}       
        self.demo_dict['init_state_dict'] = {'desired_orien':self.env.env.desired_orien.copy(),
                                            'qvel':self.env.env.sim.data.qvel.copy(),
                                            'qpos':self.env.env.sim.data.qpos.copy()}

        self.demo_dict['env_infos'] = {'qvel':None, 'desired_orien':None, 'qpos':None}
        self.demo_dict['observations'] = None
        self.demo_dict['rewards'] = None    
        self.demo_dict['actions'] = None    

    def add_im(self, image):
        #TODO maybe add an option to save each individual frame? TBD
        self.images.append(image)

    def add_demo_entry(self, env, action, obs, reward, done, info):
        rentry = np.expand_dims(np.array(reward),0)
        if(self.demo_dict['rewards'] is None):
            self.demo_dict['rewards'] = rentry
        else:
            self.demo_dict['rewards'] = np.concatenate((self.demo_dict['rewards'],rentry),0)

        qvel = self.env.env.sim.data.qvel
        qvel = np.expand_dims(qvel, 0)

        qpos = copy(self.env.env.sim.data.qpos)
        qpos = np.expand_dims(qpos, 0)

        desired_goal = np.expand_dims(env.env.desired_orien,0)
        if (self.demo_dict['env_infos']['desired_orien'] is None):
            self.demo_dict['env_infos']['desired_orien'] = desired_goal
        else:
            self.demo_dict['env_infos']['env_infos'] = np.concatenate((self.demo_dict['env_infos']['desired_orien'], desired_goal),0)

        if (self.demo_dict['env_infos']['qvel'] is None):
            self.demo_dict['env_infos']['qvel'] = qvel
        else:
            self.demo_dict['env_infos']['qvel'] = np.concatenate((self.demo_dict['env_infos']['qvel'], qvel),0)

        if (self.demo_dict['env_infos']['qpos'] is None):
            self.demo_dict['env_infos']['qpos'] = qpos
        else:
            self.demo_dict['env_infos']['qpos'] = np.concatenate((self.demo_dict['env_infos']['qpos'], qpos),0)

        if (self.demo_dict['observations'] is None):
            self.demo_dict['observations'] = obs
        else:
            self.demo_dict['observations'] = np.concatenate((self.demo_dict['observations'], obs),0)

        action = np.expand_dims(action, 0)
        if (self.demo_dict['actions'] is None):
            self.demo_dict['actions'] = action
        else:
            self.demo_dict['actions'] = np.concatenate((self.demo_dict['actions'], action),0)


    def get_finger_tip_data(self):
        finger_tip_coords = {}

        for key in TRANS_HAND_TIPS.keys():
            finger_tip_coords[key] = self.hand_coords[TRANS_HAND_TIPS[key]]

        return finger_tip_coords

    def get_joint_angles(self, finger_tip_coords):
        print('finger_tip_coords (index)' + str(finger_tip_coords['index'][1]))
        desired_joint_angles = copy(self.desired_joint_angles)
        desired_joint_angles  = self.ik_solver.bounded_linear_finger_motion(
            "index", 
            self.allegro_bounds['height'], # X value 
            self.allegro_bounds['index']['y'], # Y value
            finger_tip_coords['index'][1], # Z value
            self.calibrated_bounds[0], 
            self.allegro_bounds['index']['z_bounds'], 
            self.moving_average_queues['index'], 
            desired_joint_angles
        )
        desired_joint_angles = self.ik_solver.bounded_linear_finger_motion(
                    "middle", 
                    self.allegro_bounds['height'], # X value 
                    self.allegro_bounds['middle']['y'], # Y value
                    finger_tip_coords['middle'][1], # Z value
                    self.calibrated_bounds[1], 
                    self.allegro_bounds['middle']['z_bounds'], 
                    self.moving_average_queues['middle'], 
                    desired_joint_angles
                )
        # Movement for the Ring finger using the ring and pinky fingers
        desired_joint_angles = self.ik_solver.bounded_linear_fingers_motion(
            "ring", 
            self.allegro_bounds['height'], 
            finger_tip_coords['pinky'][1], 
            finger_tip_coords['ring'][1], 
            self.calibrated_bounds[3], # Pinky bound for Allegro Y axis movement
            self.calibrated_bounds[2], # Ring bound for Allegro Z axis movement
            self.allegro_bounds['ring']['y_bounds'], 
            self.allegro_bounds['ring']['z_bounds'], 
            self.moving_average_queues['ring'], 
            desired_joint_angles
        )

        # Movement for the Thumb
        if cv2.pointPolygonTest(np.float32(self.calibrated_bounds[4:]), np.float32(finger_tip_coords['thumb'][:2]), False) > -1:
            transformed_thumb_coordinate = perform_persperctive_transformation(
                finger_tip_coords['thumb'],
                self.calibrated_bounds[4 : ],
                self.allegro_bounds['thumb'],
                self.allegro_bounds['height']
            )

            desired_joint_angles = self.ik_solver.bounded_finger_motion(
                "thumb",
                transformed_thumb_coordinate,
                self.moving_average_queues['thumb'],
                desired_joint_angles
            )
        return desired_joint_angles

    def move(self,coords):
        print(coords)
        self.hand_coords = coords
        finger_tip_coords = self.get_finger_tip_data()
        self.desired_joint_angles = self.get_joint_angles(finger_tip_coords)
        #check if block needs to be reset
        # num_resets = 0
        # while(not self.env.env.is_on_palm()):
        #     num_resets += 1
        #     if(num_resets <= 5):
        #         for i in range(20):
        #             obs,reward, done, info = self.env.env.step(self.desired_joint_angles)
        #             if(self.display_window): self.env.env.render()
                
        #         for i in range(5):
        #             self.env.env.reset_model()
        #             if(self.display_window): self.env.render()
        #         num_resets = 0

        for i in range(10):
            obs,reward, done, info = self.env.step(self.desired_joint_angles)
            if(self.record_demo):
                self.add_demo_entry(self.env, self.desired_joint_angles, obs,reward, done, info)
                self.obs_ctr += 1
        self.env.render()
        

       
        # if(self.record_demo and self.obs_ctr % (self.obs_freq * 20) == 0):
        #     print('SAVING DEMO...')
        #     file = open(self.pkl_file,'wb')
        #     pickle.dump(self.demo_dict, file)

    def finish_recording(self, signum, frame):
        print("**********ENTERED FINISH RECORDING**********")
        self.env.env.close()   
        if(self.record_demo):
            print('writing to: ' + str(self.pkl_file))
            print(self.obs_ctr)
            file = open(self.pkl_file,'wb')
            pickle.dump(self.demo_dict, file)
            self.write_to_file = True
        import glfw
        glfw.terminate()
        print("********** DONE WRITING TO PICKLE FILE**********")
        sys.exit(0)
