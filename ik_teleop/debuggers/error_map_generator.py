# Standard imports
import numpy as np

# Image based imports
import cv2

# Parameter management import
from hydra import compose, initialize

# Imports to plot the heatmaps
import matplotlib.pyplot as plt

# Imports to get the current joint state
import rospy 
from sensor_msgs.msg import JointState

# Allegro Inverse Kinematics Library
from ik_teleop.ik_core.allegro_ik import AllegroInvKDL

# Import to publish joint state
from allegro_robot.allegro_hand_control import AllegroEnv

# Utility imports
from utils.transformations import perform_perspective_transformation

# IPython import 
from IPython import embed

class ThumbErrorDensity(object):
    def __init__(self, map_size, cfg = None, frequency = 0.1):
        # To ignore scientific notations
        np.set_printoptions(suppress = True)

        # Importing configs from yaml file
        if cfg is None:
            initialize(config_path = "../parameters/")
            self.cfg = compose(config_name = "finger_debugging")
        else:
            self.cfg = cfg

        # Initializing a ROS node
        try:
            rospy.init_node('thumb_density_map')
        except:
            pass

        # Creating an IK object for the allegro hand
        self.allegro_ik = AllegroInvKDL()

                # Creating an object which publishes data in the allegro hand
        self.allegro_pub = AllegroEnv()

        # ROS stuff to get the current joint state of the Allegro hand
        self.current_joint_state = None
        rospy.Subscriber(self.cfg.joint_angle_topic, JointState, self._callback_current_joint_state, queue_size = 1)

        # Setting the sleep frequency
        self.rate = rospy.Rate(frequency)

        # Error density map initializations
        self.map_size = map_size

        self.error_density_map_vertices = [
            [0, 0], # Left bottom
            [0, self.map_size], # Left top
            [self.map_size, self.map_size], #Right top
            [self.map_size, 0] #Right bottom
        ]

        self.jointwise_error_map = np.zeros((JOINTS_PER_FINGER, self.map_size + 1, self.map_size + 1))   
        self.avg_joint_error_map = np.zeros((self.map_size + 1, self.map_size + 1))

    def _callback_current_joint_state(self, data):
        self.current_joint_state = data

    def generate_error(self, x_point, y_point):
        # Getting the desired XY coordinate
        cartesian_coordinate = [x_point, y_point, 1]
            
        # Obtaining the desired thumb coordinate by using perspective transformation on the cartesian coordinate and current thumb joint state
        thumb_coordinate = perform_perspective_transformation(cartesian_coordinate, self.error_density_map_vertices, self.cfg.allegro_bounds.thumb)
        current_thumb_state = np.array(list(self.current_joint_state.position)[self.cfg.allegro.fingers['thumb'].offset : self.cfg.allegro.fingers['thumb'].offset + self.cfg.allegro.joints_per_finger])
    
        # Performing inverse kinematics on the desired joint angles
        calculated_thumb_angles = self.allegro_ik.finger_inverse_kinematics('thumb', thumb_coordinate, current_thumb_state)

        # Making a copy of current angle array to update the current angles
        desired_angles = list(self.current_joint_state.position)

        for idx in range(self.cfg.allegro.joints_per_finger):
            desired_angles[self.cfg.allegro.fingers['thumb'].offset + idx] = calculated_thumb_angles[idx]

        # Publishing the angles in the joint_state rostopic using the Allegro hand wrapper function
        self.allegro_pub.pose_step(desired_angles)

        # Sleeping to wait for the thumb to reach the position
        rospy.sleep(2)

        # Calculating the error after moving the thumb
        reached_angles = np.array(list(self.current_joint_state.position)[self.cfg.allegro.fingers['thumb'].offset : self.cfg.allegro.fingers['thumb'].offset + self.cfg.allegro.joints_per_finger])
        error_vector = np.array(calculated_thumb_angles) - reached_angles

        print('Angle Error for point:', error_vector)

        # Logging jointwise errors
        for joint_num in range(self.cfg.allegro.joints_per_finger):
            self.jointwise_error_map[joint_num][y_point][x_point] = np.abs(error_vector[joint_num])

        # Logging average errors
        self.avg_joint_error_map[y_point][x_point] = np.abs(np.linalg.norm(error_vector))

    def generate_error_map(self):
        print('Waiting to receive current joint state from topic!')
        while self.current_joint_state is None:
            rospy.sleep(2)

        print('Started receiving current joint state data from topic. \nGenerating Error map now!')
        # Iterating over the y_axis
        for y_idx in range(self.map_size + 1):

            # Iterating over the x_axis
            if y_idx % 2 == 0:
                for x_idx in range(self.map_size + 1):
                    self.generate_error(x_idx, y_idx)
                    rospy.sleep(2)

            else:
                for x_idx in range(self.map_size, -1, -1):
                    self.generate_error(x_idx, y_idx)
                    rospy.sleep(2)
        
        print('Generated error map!')

        with open('jointwise_error_map.npy', 'wb') as file:
            np.save(file, self.jointwise_error_map)

        with open('avg_error_map.npy', 'wb') as file:
            np.save(file, self.avg_joint_error_map)


    def generate_jointwise_heat_map(self):
        # Plotting all the heatmaps jointwise
        plt.subplot(2,2,1)
        plt.imshow(self.jointwise_error_map[0] , cmap = 'rainbow' , interpolation = 'nearest' )
        plt.title( "Thumb Joint 0 Heatmap" )

        plt.subplot(2,2,2)
        plt.imshow(self.jointwise_error_map[1] , cmap = 'rainbow' , interpolation = 'nearest' )
        plt.title( "Thumb Joint 1 Heatmap" )

        plt.subplot(2,2,3)
        plt.imshow(self.jointwise_error_map[2] , cmap = 'rainbow' , interpolation = 'nearest' )
        plt.title( "Thumb Joint 2 Heatmap" )

        plt.subplot(2,2,4)
        plt.imshow(self.jointwise_error_map[3] , cmap = 'rainbow' , interpolation = 'nearest' )
        plt.title( "Thumb Joint 3 Heatmap" )

        # Displaying the plot
        plt.show()

        # Showing the metrics
        # Max and min values for the Thumb base
        print('Max value for thumb joint 0:', np.amax(self.jointwise_error_map[0]))
        print('Min value for thumb joint 0:', np.amin(self.jointwise_error_map[0]))

        # Max and min values for the Thumb rotatory joint
        print('Max value for thumb joint 1:', np.amax(self.jointwise_error_map[1]))
        print('Min value for thumb joint 1:', np.amin(self.jointwise_error_map[1]))

        # Max and min values for the Thumb metacarpal
        print('Max value for thumb joint 2:', np.amax(self.jointwise_error_map[2]))
        print('Min value for thumb joint 2:', np.amin(self.jointwise_error_map[2]))

        # Max and min values for the Thumb last joint
        print('Max value for thumb joint 3:', np.amax(self.jointwise_error_map[3]))
        print('Min value for thumb joint 3:', np.amin(self.jointwise_error_map[3]))

    def generate_avg_error_heat_map(self):
        # Plotting a single average heatmap
        plt.imshow(self.avg_joint_error_map , cmap = 'rainbow' , interpolation = 'nearest' )
        plt.title( "Thumb average error heat map" )
        
        # Displaying the plot
        plt.show()

        # Showing the metrics
        # Max and min values for the average error
        print('Max value for avg error:', np.amax(self.avg_joint_error_map))
        print('Min value for avg error:', np.amin(self.avg_joint_error_map))

if __name__ == '__main__':
    allegro = ThumbDensity(15)
    embed()