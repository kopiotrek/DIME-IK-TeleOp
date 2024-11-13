import numpy as np
import rospy
from copy import deepcopy as copy
from std_msgs.msg import Float64MultiArray
from constants import *
from vectorops import *
from timer import FrequencyTimer
import time
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d
import math


class TransformHandPositionCoords():
    def __init__(self, moving_average_limit = 5):
        # self.notify_component_start('keypoint position transform')
        try:
            rospy.init_node('transformed_hand_coords_publisher')
        except rospy.ROSInterruptException:
            print('transformed_hand_coords_publisher initializing failed')
            pass
        
        # Initializing the subscriber for right hand keypoints
        self.original_keypoint_subscriber = rospy.Subscriber('hand_coords', Float64MultiArray, callback=self._get_hand_coords, queue_size=1)
        # self.original_keypoint_subscriber = ZMQKeypointSubscriber(host, keypoint_port, 'right')
        # Initializing the publisher for transformed right hand keypoints
        self.transformed_keypoint_publisher = rospy.Publisher('transformed_hand_coords', Float64MultiArray, queue_size=1)
        # self.transformed_keypoint_publisher = ZMQKeypointPublisher(host, transformation_port)
        # Timer
        self.timer = FrequencyTimer(VR_FREQ)
        # Keypoint indices for knuckles
        self.reframe_keypoints = (OCULUS_JOINTS['wrist'][0],OCULUS_JOINTS['knuckles'][0], OCULUS_JOINTS['knuckles'][1],OCULUS_JOINTS['knuckles'][2],OCULUS_JOINTS['knuckles'][3])
        # Moving average queue
        self.moving_average_limit = moving_average_limit
        # Create a queue for moving average
        self.coord_moving_average_queue, self.frame_moving_average_queue = [], []
        self.coords_to_publish = Float64MultiArray()
        
        self.fingers = [
            [[0, 17], [17, 18], [18, 19], [19, 20]],  # Pinkie
            [[0, 13], [13, 14], [14, 15], [15, 16]],  # Ring
            [[0, 9], [9, 10], [10, 11], [11, 12]],    # Middle
            [[0, 5], [5, 6], [6, 7], [7, 8]],         # Index
            [[0, 1], [1, 2], [2, 3], [3, 4]]          # Thumb
        ]
        self.fingers_colors = ['red', 'blue', 'green', 'black', 'orange']
        self.Rz = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
        self.Rx = np.array([[1., 0., 0.], [0., -1., 0.], [0., 0., -1.]])
        
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        plt.ion()  # Enable interactive mode
        self.setup_plot()
        self.initialized = False
        
    def setup_plot(self):
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        self.ax.set_zticks([])
        self.ax.set_xlim3d(0, 10)
        self.ax.set_ylim3d(-5, 5)
        self.ax.set_zlim3d(-5, 5)
        self.ax.set_xlabel('x')
        self.ax.set_ylabel('y')
        self.ax.set_zlabel('z')
        # Static plot elements
        self.ax.plot([0, 5], [0, 0], [0, 0], linewidth=2, color='red')
        self.ax.plot([0, 0], [0, 5], [0, 0], linewidth=2, color='blue')
        self.ax.plot([0, 0], [0, 0], [0, 5], linewidth=2, color='black')

    # Function to get the hand coordinates from the VR
    def _get_hand_coords(self, msg):
        # Convert the incoming data to a numpy array
        
        hand_coords_array = np.asanyarray(msg.data)

        # Ensure the data length is correct (should be OCULUS_NUM_KEYPOINTS * 3)
        if len(hand_coords_array) != OCULUS_NUM_KEYPOINTS * 3:
            rospy.logerr(f"Received data length {len(hand_coords_array)} does not match expected length {OCULUS_NUM_KEYPOINTS * 3}")
            # return None   

        # Reshape into (OCULUS_NUM_KEYPOINTS, 3) for the hand keypoints
        try:
            self.hand_coords = hand_coords_array.reshape(OCULUS_NUM_KEYPOINTS, 3)
            if self.initialized is True:
                self.coords_to_publish.data = self.transformed_hand_coords.flatten()
                self.transformed_keypoint_publisher.publish(self.coords_to_publish)

        except ValueError as e:
            rospy.logerr(f"Error reshaping data: {e}")

        
    def normalize_finger_length(self, coords, target_length=10):
        chain_length=math.dist(coords[0],coords[5])

        scale = target_length/chain_length
        print(scale)

        # Scale each segment of the finger
        for i in range(len(coords)):
            coords[i]*=scale
        return coords
    
    # Function to find hand coordinates with respect to the wrist
    def _translate_coords(self, hand_coords):
        return copy(hand_coords) - hand_coords[9]

    # Axis meaning: 
    # Origin is where palm meets middle finger
    # X is vector from origin outwards (perpendicular to palm)
    # Y is vector from origin to the thumb (left)
    # Z is vector from origin to middle finger (up)
    # Create a coordinate frame for the hand
    def _get_coord_frame(self, wrist_coord, index_knuckle_coord, middle_knuckle_coord, ring_knuckle_coord, pinky_knuckle_coord):
        # Calculate Z-axis: vector from wrist to middle knuckle (upwards towards the middle finger)
        z_axis = normalize_vector(middle_knuckle_coord - wrist_coord)
        
        # Calculate Y-axis: vector from wrist to index knuckle (towards the thumb)
        y_axis = normalize_vector(index_knuckle_coord - wrist_coord)
        
        # Calculate X-axis as the cross product of Y and Z axes to ensure orthogonality (outward from palm)
        x_axis = normalize_vector(np.cross(y_axis, z_axis))
        
        # Recalculate Y-axis to ensure orthogonality with the new X and Z axes
        y_axis = normalize_vector(np.cross(z_axis, x_axis))
        
        return [x_axis, y_axis, z_axis]



    def transform_keypoints(self, hand_coords):
        translated_coords = self._translate_coords(hand_coords)
        print("D1")
        translated_coords = self.normalize_finger_length(translated_coords)
        original_coord_frame = self._get_coord_frame(
            translated_coords[self.reframe_keypoints[0]], 
            translated_coords[self.reframe_keypoints[1]], 
            translated_coords[self.reframe_keypoints[2]], 
            translated_coords[self.reframe_keypoints[3]], 
            translated_coords[self.reframe_keypoints[4]]
        )

        if np.linalg.det(original_coord_frame) == 0:
            rospy.logerr("Original coord frame is singular and cannot be inverted")
            return

        try:
            rotation_matrix = np.linalg.solve(original_coord_frame, np.eye(3)).T
            self.transformed_hand_coords = (rotation_matrix @ translated_coords.T).T
        except np.linalg.LinAlgError as e:
            rospy.logerr(f"Error computing rotation matrix: {e}")
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
        """Visualize the keypoints for a single frame with angles between consecutive points."""
        kpts3d_rotated = np.array([kpt for kpt in kpts3d])
    
        # Clear plot axes for each frame and replot
        self.ax.cla()
        self.setup_plot()
    
        # Plot each finger (as before)
        for finger, finger_color in zip(self.fingers, self.fingers_colors):
            for _c in finger:
                self.ax.plot(
                    [kpts3d_rotated[_c[0], 0], kpts3d_rotated[_c[1], 0]],
                    [kpts3d_rotated[_c[0], 1], kpts3d_rotated[_c[1], 1]],
                    [kpts3d_rotated[_c[0], 2], kpts3d_rotated[_c[1], 2]],
                    linewidth=4, color=finger_color
                )
    
        # Define points and calculate angles between them
        points = [kpts3d_rotated[i] for i in range(5)]
    
        # Loop through consecutive triplets of points to calculate angles
        for i in range(3):
            p1, p2, p3 = points[i], points[i + 1], points[i + 2]
    
            # Calculate vectors and the angle between them
            vec1 = p1 - p2
            vec2 = p3 - p2
            angle = self.angle_between_vectors(vec1, vec2)
    
            # Midpoint between p1 and p3 to display angle
            mid_point = (p1 + p3) / 2
            self.ax.text(mid_point[0], mid_point[1], mid_point[2], f'{angle:.2f} rad', color='black', fontsize=10)
    
        # Plot individual keypoints (optional visualization)
        for idx, point in enumerate(points):
            self.ax.scatter([point[0]], [point[1]], [point[2]], color=self.fingers_colors[idx % len(self.fingers_colors)], s=50)
    
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