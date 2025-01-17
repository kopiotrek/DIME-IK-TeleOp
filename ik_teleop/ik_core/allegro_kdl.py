from ikpy import chain
import rospy
import numpy as np
from copy import deepcopy as copy
from ik_teleop.teleop_utils.files import *
from ik_teleop.teleop_utils.constants import *
from ik_teleop.teleop_utils.vectorops import *
from ik_teleop.ik_core.allegro_ik import ThumbIK, FingerIK 
import time
import warnings
warnings.filterwarnings('ignore', category=UserWarning, module='ikpy')


from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

THUMB_IK_MARKER_TOPIC = '/ik_marker/thumb'
INDEX_IK_MARKER_TOPIC = '/ik_marker/index'
MIDDLE_IK_MARKER_TOPIC = '/ik_marker/middle'
RING_IK_MARKER_TOPIC = '/ik_marker/ring'

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
        self.thumb_ik = ThumbIK()
        self.finger_ik = FingerIK()
        self.thumb_ik_marker_publisher = rospy.Publisher(THUMB_IK_MARKER_TOPIC, Marker, queue_size=1)
        self.index_ik_marker_publisher = rospy.Publisher(INDEX_IK_MARKER_TOPIC, Marker, queue_size=1)
        self.ring_ik_marker_publisher = rospy.Publisher(RING_IK_MARKER_TOPIC, Marker, queue_size=1)
        self.middle_ik_marker_publisher = rospy.Publisher(MIDDLE_IK_MARKER_TOPIC, Marker, queue_size=1)

    
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

    

    def finger_inverse_kinematics(self, finger_type, input_position, curr_finger_angles):
        # Checking if the input figner type is a valid one
        if finger_type not in self.hand_configs['fingers'].keys():
            print('Finger type does not exist')
            return

        if finger_type == 'thumb':
            # print(f"input_position {input_position}")
            # time.sleep(.3)

            marker = Marker()
            marker.header.frame_id = "palm_link"  # Change to your frame of reference if needed
            marker.header.stamp = rospy.Time.now()
            marker.ns = "basic_shapes"
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.scale.x = 0.02
            marker.scale.y = 0.02
            marker.scale.z = 0.02
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.5  # Alpha (transparency)

            marker.pose.position.x = input_position[0]
            marker.pose.position.y = input_position[1]
            marker.pose.position.z = input_position[2]

            self.thumb_ik_marker_publisher.publish(marker)

            rotation_angles = (np.pi + np.deg2rad(5), 0, 0)
            input_position += [0.0182, -0.016958, 0.073288]
            input_position = rotate_point(input_position, rotation_angles)

            input_position += [0,0,0.03]
            output_angles = self.thumb_ik.compute_ik(input_position)
            # print(f"Computed Joint Angles (IK) {finger_type}:", output_angles)
            output_angles = np.append(output_angles, 0)
            return output_angles[0:4]

        elif finger_type == 'index':
            # wyprostowany
            # ip_x = [-0.1415960225855222, 0.006090478759842102, 0.009434112446888552]
            # scisniety
            # ip_x = [0.06613992374305373, 0.0017504104318999403, 0.04478159966084637]
            # przeskakuje
            # ip_x = [0.032225730640252054, 0.02968919827946307, 0.14660298104548292]
            # ip_x = [-0.014293, -0.045098, 0.08]
            # pos_error: [-0.00085594  0.00848489  0.00083459]
            # q: [0.57       1.71       0.29181021 0.00000033]
            

            # input_position = [ip_x[2],-ip_x[1],-ip_x[0]]
            
            # input_position = [0.0, 0.0, 0.1527]

            marker = Marker()
            marker.header.frame_id = "palm_link"  # Change to your frame of reference if needed
            marker.header.stamp = rospy.Time.now()
            marker.ns = "basic_shapes"
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.scale.x = 0.02
            marker.scale.y = 0.02
            marker.scale.z = 0.02
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.5  # Alpha (transparency)
        
            marker.pose.position.x = input_position[0]
            marker.pose.position.y = input_position[1]
            marker.pose.position.z = input_position[2]

            self.index_ik_marker_publisher.publish(marker)
            
            rotation_angles = (-np.deg2rad(5), 0, 0)
            input_position = rotate_point(input_position, rotation_angles)
            # print(f"input_position trans{input_position}")
            # time.sleep(.3)
            input_position += [0, -0.045098, -0.014293]

            # ik_position = [-input_position[2],-input_position[1],input_position[0]]
            # ik_position = [input_position[0],input_position[1],input_position[2]]
            output_angles = self.finger_ik.compute_ik(finger_type, input_position, curr_finger_angles)
            # print(f"Computed Joint Angles (IK) {finger_type}:", output_angles)
            return output_angles[0:4]

        elif finger_type == 'middle':
            # print(f"input_position {input_position}")
            # input_position[2] -= 0.005
            # input_position[2] *= 3
            # input_position[0] -= 0.03
            # input_position[0] *= 1.5
            # input_position[1] *= 1.8
            # print(f"input_position trans{input_position}")
            # time.sleep(.3)

            marker = Marker()
            marker.header.frame_id = "palm_link"  # Change to your frame of reference if needed
            marker.header.stamp = rospy.Time.now()
            marker.ns = "basic_shapes"
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.scale.x = 0.02
            marker.scale.y = 0.02
            marker.scale.z = 0.02
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.5  # Alpha (transparency)

            marker.pose.position.x = input_position[0]
            marker.pose.position.y = input_position[1]
            marker.pose.position.z = input_position[2]

            self.middle_ik_marker_publisher.publish(marker)

            input_position += [0, 0, -0.0166]



            output_angles = self.finger_ik.compute_ik(finger_type, input_position, curr_finger_angles)
            # print(f"Computed Joint Angles (IK) {finger_type}:", output_angles)
            return output_angles[0:4]

        elif finger_type == 'ring':
            # print(f"input_position {input_position}")
            # input_position[2] -= 0.005
            # input_position[2] *= 3
            # input_position[0] -= 0.03
            # input_position[0] *= 1.5
            # input_position[1] *= 1.8
            # print(f"input_position trans{input_position}")
            # time.sleep(.3)

            marker = Marker()
            marker.header.frame_id = "palm_link"  # Change to your frame of reference if needed
            marker.header.stamp = rospy.Time.now()
            marker.ns = "basic_shapes"
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.scale.x = 0.02
            marker.scale.y = 0.02
            marker.scale.z = 0.02
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.5  # Alpha (transparency)
            marker.pose.position.x = input_position[0]
            marker.pose.position.y = input_position[1]
            marker.pose.position.z = input_position[2]

            self.ring_ik_marker_publisher.publish(marker)

            input_position += [0, 0.045098, -0.014293]
            rotation_angles = (-np.deg2rad(5), 0, 0)
            input_position = rotate_point(input_position, rotation_angles)

            output_angles = self.finger_ik.compute_ik(finger_type, input_position, curr_finger_angles)
            # print(f"Computed Joint Angles (IK) {finger_type}:", output_angles)
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

   

# if __name__ == '__main__':
#     ik_control = AllegroKDL()

#     # Set desired position and orientation
#     # output_frame = ik_control.finger_forward_kinematics('thumb',[ 0.49158065,  0.62981548, -2.99420419,  3.29378238])  # Example position
#     thumb_joint_angles = ik_control.finger_inverse_kinematics('thumb', [ 0.02698166,  0.16099207, -0.07196472])
#     print(f"thumb_joint_angles {thumb_joint_angles}")
#     # ik_control.rotation = np.array([1, 0, 0, 0])     # Identity quaternion

#     # Perform the IK update
#     # ik_control.Update()