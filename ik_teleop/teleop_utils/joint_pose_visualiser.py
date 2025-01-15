import numpy as np
import rospy
from copy import deepcopy as copy
from geometry_msgs.msg import PoseArray
from constants import *
from vectorops import *
from timer import FrequencyTimer
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # Needed for 3D plotting


class TransformHandPositionCoords():
    def __init__(self, moving_average_limit = 5):
        # self.notify_component_start('keypoint position transform')
        try:
            rospy.init_node('transformed_hand_coords_publisher')
        except rospy.ROSInterruptException:
            print('transformed_hand_coords_publisher initializing failed')
            pass
        
        # Initializing the subscriber for right hand keypoints
        self.original_keypoint_subscriber = rospy.Subscriber('/XR/JointPoseArray' , PoseArray, callback=self._get_hand_coords, queue_size=1)
        # self.original_keypoint_subscriber = ZMQKeypointSubscriber(host, keypoint_port, 'right')
        # Initializing the publisher for transformed right hand keypoints
        # Timer
        self.timer = FrequencyTimer(VR_FREQ)
        # Keypoint indices for knuckles
        self.knuckle_points = (OCULUS_JOINTS['knuckles'][0], OCULUS_JOINTS['knuckles'][1])
        # Moving average queue
        self.moving_average_limit = moving_average_limit
        # Create a queue for moving average
        self.coord_moving_average_queue, self.frame_moving_average_queue = [], []
        
        
        
        self.fingers = [
            [[0, 22], [22, 23], [23, 24], [24, 25]],  # Pinkie
            [[0, 17], [17, 18], [18, 19], [19, 20]],  # Ring
            [[0, 12], [12, 13], [13, 14], [14, 15]],    # Middle
            [[0, 7], [7, 8], [8, 9], [9, 10]],         # Index
            [[0, 2], [2, 3], [3, 4], [4, 5]]          # Thumb
        ]
        self.fingers_colors = ['red', 'blue', 'green', 'black', 'orange']
        self.Rz = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
        self.Rx = np.array([[1., 0., 0.], [0., -1., 0.], [0., 0., -1.]])
        
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        plt.ion()  # Enable interactive mode
        self.setup_plot()
        self.initialized = False
        
    def position_to_array(self, position):
        return np.array([position.x, position.y, position.z])

    def setup_plot(self):
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        self.ax.set_zticks([])
        # self.ax.set_xlim3d(0, 10)
        # self.ax.set_ylim3d(-5, 5)
        # self.ax.set_zlim3d(-5, 5)
        self.ax.set_xlim3d(-.0, .2)
        self.ax.set_ylim3d(-.1, .1)
        self.ax.set_zlim3d(-.1, .1)
        self.ax.set_xlabel('x')
        self.ax.set_ylabel('y')
        self.ax.set_zlabel('z')
        # Static plot elements
        self.ax.plot([0, .1], [0, 0], [0, 0], linewidth=2, color='red')
        self.ax.plot([0, 0], [0, .1], [0, 0], linewidth=2, color='green')
        self.ax.plot([0, 0], [0, 0], [0, .1], linewidth=2, color='blue')

    # Function to get the hand coordinates from the VR
    def _get_hand_coords(self, msg):
        # Convert the incoming data to a numpy array
        finger_coords_array = []
        if len(msg.poses) < OCULUS_NUM_KEYPOINTS:
            print("ERROR: not enough joints received")
            return
        for i in range(OCULUS_NUM_KEYPOINTS):
            finger_coords_array.append(self.position_to_array(msg.poses[i].position))     
        finger_coords_array_np = np.array(finger_coords_array)

        # Reshape into (OCULUS_NUM_KEYPOINTS, 3) for the hand keypoints
        try:
            self.hand_coords = np.array(finger_coords_array).reshape(OCULUS_NUM_KEYPOINTS, 3)
        except ValueError as e:
            rospy.logerr(f"Error reshaping data: {e}")

            
            

    
    # Function to find hand coordinates with respect to the wrist
    def _translate_coords(self, hand_coords):
        return copy(hand_coords) - hand_coords[12]

    # def _get_coord_frame(self, index_knuckle_coord, pinky_knuckle_coord, little_knuckle_coord):
    #     return [[1,0,0],[0,1,0],[0,0,1]]

    # Axis meaning: 
    # Origin is where palm meets middle finger
    # X is vector from origin outwards (perpendicular to palm)
    # Y is vector from origin to the thumb (left)
    # Z is vector from origin to middle finger (up)
    # Create a coordinate frame for the hand
    def _get_coord_frame(self, wrist_coord, index_knuckle_coord, little_knuckle_coord):
        z_axis = normalize_vector(wrist_coord)
        
        x_axis = normalize_vector(index_knuckle_coord - little_knuckle_coord)
        
        y_axis = normalize_vector(np.cross(z_axis, x_axis))
        
        return [y_axis, x_axis, -z_axis]

    # Create a coordinate frame for the arm 
    def _get_hand_dir_frame(self, origin_coord, index_knuckle_coord, pinky_knuckle_coord):

        palm_normal = normalize_vector(np.cross(index_knuckle_coord, pinky_knuckle_coord))   # Unity space - Y
        palm_direction = normalize_vector(index_knuckle_coord + pinky_knuckle_coord)         # Unity space - Z
        cross_product = normalize_vector(index_knuckle_coord - pinky_knuckle_coord)          # Unity space - X
        
        return [origin_coord, cross_product, palm_normal, palm_direction]

    def transform_keypoints(self, hand_coords):
        translated_coords = self._translate_coords(hand_coords)
        original_coord_frame = self._get_coord_frame(
            translated_coords[OCULUS_JOINTS['wrist'][0]],
            translated_coords[OCULUS_JOINTS['knuckles'][0]], 
            translated_coords[OCULUS_JOINTS['knuckles'][3]]
        )

        if np.linalg.det(original_coord_frame) == 0:
            rospy.logerr("Original coord frame is singular and cannot be inverted")
            return
        
        try:
            rotation_matrix = np.linalg.solve(original_coord_frame, np.eye(3)).T
            self.transformed_hand_coords = (rotation_matrix @ translated_coords.T).T
        except np.linalg.LinAlgError as e:
            rospy.logerr(f"Error computing rotation matrix: {e}")

        print(f"thumb tip {self.transformed_hand_coords[OCULUS_JOINTS['thumb'][3]]}")
        self.visualize_3d(self.transformed_hand_coords)
    
    def angle_between_vectors(self, v1, v2):
        """Calculate the angle between two vectors in radians."""
        dot_prod = np.dot(v1, v2)
        norm_v1 = np.linalg.norm(v1)
        norm_v2 = np.linalg.norm(v2)
        cos_theta = dot_prod / (norm_v1 * norm_v2)
        angle = np.arccos(np.clip(cos_theta, -1.0, 1.0))  # Clip to avoid numerical errors
        return angle

    def visualize_3d(self, kpts3d):
        """Visualize the keypoints for a single frame."""
        kpts3d_rotated = np.array([self.Rz @ self.Rx @ kpt for kpt in kpts3d])
        kpts3d_rotated[:, 2] *= -1  # Invert Z-axis
        # Clear plot axes for each frame and replot
        self.ax.cla()
        self.setup_plot()
        # Add axis arrows
        axis_length = 0.2  # Length of the arrows
        self.ax.quiver(0, 0, 0, axis_length, 0, 0, color='r', linewidth=2, arrow_length_ratio=0.1, label='X (positive)')
        self.ax.quiver(0, 0, 0, -axis_length, 0, 0, color='r', linewidth=2, arrow_length_ratio=0.1, linestyle='dotted', label='X (negative)')
        self.ax.quiver(0, 0, 0, 0, axis_length, 0, color='g', linewidth=2, arrow_length_ratio=0.1, label='Y (positive)')
        self.ax.quiver(0, 0, 0, 0, -axis_length, 0, color='g', linewidth=2, arrow_length_ratio=0.1, linestyle='dotted', label='Y (negative)')
        self.ax.quiver(0, 0, 0, 0, 0, axis_length, color='b', linewidth=2, arrow_length_ratio=0.1, label='Z (positive)')
        self.ax.quiver(0, 0, 0, 0, 0, -axis_length, color='b', linewidth=2, arrow_length_ratio=0.1, linestyle='dotted', label='Z (negative)')

        # Plot each finger
        for finger, finger_color in zip(self.fingers, self.fingers_colors):
            for _c in finger:
                self.ax.plot(
                    [kpts3d_rotated[_c[0], 0], kpts3d_rotated[_c[1], 0]],
                    [kpts3d_rotated[_c[0], 1], kpts3d_rotated[_c[1], 1]],
                    [kpts3d_rotated[_c[0], 2], kpts3d_rotated[_c[1], 2]],
                    linewidth=4, color=finger_color
                )

        # Select keypoints for which you want to calculate the angle
        point_1 = kpts3d_rotated[0]  # Example keypoints
        point_2 = kpts3d_rotated[5]
        point_3 = kpts3d_rotated[6]

        # Calculate vectors between points
        vec1 = point_1 - point_2
        vec2 = point_3 - point_2

        # Compute the angle between these vectors in radians
        angle = self.angle_between_vectors(vec1, vec2)

        # Display the calculated angle on the plot
        mid_point = (point_1 + point_3) / 2  # Midpoint to display the angle
        self.ax.text(mid_point[0], mid_point[1], mid_point[2], f'{angle:.2f} rad', color='black', fontsize=10)

        # Plot individual keypoints (optional visualization as before)
        self.ax.scatter([point_1[0]], [point_1[1]], [point_1[2]], color=self.fingers_colors[0], s=50)
        self.ax.scatter([point_2[0]], [point_2[1]], [point_2[2]], color=self.fingers_colors[1], s=50)
        self.ax.scatter([point_3[0]], [point_3[1]], [point_3[2]], color=self.fingers_colors[2], s=50)

        plt.draw()
        plt.pause(0.01)

    def stream(self):
        while not rospy.is_shutdown():
            try:
                self.timer.start_loop()
               
                # Shift the points to required axes
                try:
                    self.transform_keypoints(self.hand_coords)
                    self.initialized = True
                    
                except:
                    rospy.logerr("Problem transforming keypoints")
                    time.sleep(1)
                
                # self.timer.end_loop()
            except:
                break
        
        # self.original_keypoint_subscriber.stop()
        # self.transformed_keypoint_publisher.stop()

        print('Stopping the transformed hand coords node.')

if __name__ == '__main__':
    obj = TransformHandPositionCoords()
    obj.stream()    