import cv2 as cv
import mediapipe as mp
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # Needed for 3D plotting

import numpy as np
import rospy
import time
from utils import DLT, get_projection_matrix, write_keypoints_to_disk
from std_msgs.msg import Float64MultiArray

mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands

frame_shape = [480, 640]

class HandJointStatePublisher:
    def __init__(self):
        try:
            rospy.init_node('hand_joint_state_publisher', anonymous=True)
        except rospy.ROSInterruptException:
            pass
        
        self.pub = rospy.Publisher('hand_coords', Float64MultiArray, queue_size=1)
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
        
    def setup_plot(self):
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        self.ax.set_zticks([])
        self.ax.set_xlim3d(0, 20)
        self.ax.set_ylim3d(-30, -10)
        self.ax.set_zlim3d(-30, -10)
        self.ax.set_xlabel('x')
        self.ax.set_ylabel('y')
        self.ax.set_zlabel('z')
        # Static plot elements
        self.ax.plot([0, 5], [0, 0], [0, 0], linewidth=2, color='red')
        self.ax.plot([0, 0], [0, 5], [0, 0], linewidth=2, color='blue')
        self.ax.plot([0, 0], [0, 0], [0, 5], linewidth=2, color='black')

    def visualize_3d(self, kpts3d):
        """Visualize the keypoints for a single frame."""
        kpts3d_rotated = np.array([self.Rz @ self.Rx @ kpt for kpt in kpts3d])

        # Clear plot axes for each frame and replot
        self.ax.cla()
        self.setup_plot()

        # Plot each finger
        for finger, finger_color in zip(self.fingers, self.fingers_colors):
            for _c in finger:
                self.ax.plot(
                    [kpts3d_rotated[_c[0], 0], kpts3d_rotated[_c[1], 0]],
                    [kpts3d_rotated[_c[0], 1], kpts3d_rotated[_c[1], 1]],
                    [kpts3d_rotated[_c[0], 2], kpts3d_rotated[_c[1], 2]],
                    linewidth=4, color=finger_color
                )
        plt.draw()
        plt.pause(0.01)
        
    def calculate_hand_joint_states(self, kpts_3d_array):
        """Visualize the latest frame's 3D keypoints and publish 63 coordinates."""
        if kpts_3d_array.ndim != 2 or kpts_3d_array.shape[1] != 3:
            raise ValueError("Expected kpts_3d_array to be of shape (21, 3)")      
        
        # Flatten the 21 keypoints (each having 3 coordinates) into a 1D array
        data = kpts_3d_array.flatten().tolist()
    
        # Prepare the ROS message with the flattened coordinates
        coords_to_publish = Float64MultiArray()
        coords_to_publish.data = data
        self.pub.publish(coords_to_publish)
        time.sleep(0.001)
        
        # Visualize only the last frame
        # self.visualize_3d(kpts_3d_array)
    
    
    def moving_average(self, keypoints, buffer, window_size):
        """Apply moving average smoothing to the 2D/3D keypoints, ignoring invalid points."""
        
        # Only append keypoints if they are valid (not None and not too close to 0)
        if keypoints is not None and np.any(np.abs(keypoints) > 1e-6):
            buffer.append(keypoints)
    
        if len(buffer) > window_size:
            buffer.pop(0)
    
        # If the buffer has valid points, compute the average; otherwise, use the current keypoints
        if len(buffer) > 0:
            average_kpts = np.mean(buffer, axis=0)
        else:
            average_kpts = keypoints  # Fallback to current keypoints if no valid data
    
        return average_kpts, buffer
    

    def run_mp(self, input_stream1, input_stream2, P0, P1):
        cap0 = cv.VideoCapture(input_stream1)
        cap1 = cv.VideoCapture(input_stream2)

        # Get the frame width and height of cap0
        frame_width0 = int(cap0.get(cv.CAP_PROP_FRAME_WIDTH))
        frame_height0 = int(cap0.get(cv.CAP_PROP_FRAME_HEIGHT))

        # Get the frame width and height of cap1
        frame_width1 = int(cap1.get(cv.CAP_PROP_FRAME_WIDTH))
        frame_height1 = int(cap1.get(cv.CAP_PROP_FRAME_HEIGHT))

        # Define the codec and create VideoWriter objects to save the videos
        fourcc = cv.VideoWriter_fourcc(*'XVID')  # You can use other codecs like 'mp4v', 'MJPG', etc.

        out0 = cv.VideoWriter('output_cap0.avi', fourcc, 30.0, (frame_width0, frame_height0))
        out1 = cv.VideoWriter('output_cap1.avi', fourcc, 30.0, (frame_width1, frame_height1))

        caps = [cap0, cap1]
        # Get size of the video stream from cap0
        width0 = int(cap0.get(cv.CAP_PROP_FRAME_WIDTH))
        height0 = int(cap0.get(cv.CAP_PROP_FRAME_HEIGHT))
        print(f"Stream 1 Size: {width0}x{height0}")

        # Get size of the video stream from cap1
        width1 = int(cap1.get(cv.CAP_PROP_FRAME_WIDTH))
        height1 = int(cap1.get(cv.CAP_PROP_FRAME_HEIGHT))
        print(f"Stream 2 Size: {width1}x{height1}")
        for cap in caps:
            cap.set(3, frame_shape[1])
            cap.set(4, frame_shape[0])

        hands = mp_hands.Hands(min_detection_confidence=0.5, max_num_hands=1, min_tracking_confidence=0.5)

        kpts_cam0, kpts_cam1, kpts_3d = [], [], []

        # Buffers for moving average smoothing
        buffer0, buffer1 = [], []

        while not rospy.is_shutdown():
            ret0, frame0 = cap0.read()
            ret1, frame1 = cap1.read()

            if not ret0 or not ret1:
                break
            out0.write(frame0)
            out1.write(frame1)
            frame0_rgb = cv.cvtColor(frame0, cv.COLOR_BGR2RGB)
            frame1_rgb = cv.cvtColor(frame1, cv.COLOR_BGR2RGB)

            results0 = hands.process(frame0_rgb)
            results1 = hands.process(frame1_rgb)
            
            if cv.waitKey(1) & 0xFF == ord('q'):
                break

            # Extract keypoints for each hand, or ignore if no landmarks detected
            if results0.multi_hand_landmarks:
                frame0_keypoints = [
                    [int(round(frame0.shape[1] * hand_landmarks.landmark[p].x)),
                     int(round(frame0.shape[0] * hand_landmarks.landmark[p].y))]
                    for hand_landmarks in results0.multi_hand_landmarks for p in range(21)]
            else:
                frame0_keypoints = None

            if results1.multi_hand_landmarks:
                frame1_keypoints = [
                    [int(round(frame1.shape[1] * hand_landmarks.landmark[p].x)),
                     int(round(frame1.shape[0] * hand_landmarks.landmark[p].y))]
                    for hand_landmarks in results1.multi_hand_landmarks for p in range(21)]
            else:
                frame1_keypoints = None

            # Proceed only if both frames have valid keypoints
            if frame0_keypoints and frame1_keypoints:
                # Apply moving average smoothing
                frame0_keypoints, buffer0 = self.moving_average(frame0_keypoints, buffer0, 3)
                frame1_keypoints, buffer1 = self.moving_average(frame1_keypoints, buffer1, 3)

                kpts_cam0.append(frame0_keypoints)
                kpts_cam1.append(frame1_keypoints)

                # Only compute 3D keypoints if both UV coordinates are valid
                frame_p3ds = [DLT(P0, P1, uv1, uv2) for uv1, uv2 in zip(frame0_keypoints, frame1_keypoints)]
                kpts_3d = np.array(frame_p3ds).reshape((21, 3))
                self.calculate_hand_joint_states(kpts_3d)  # Pass the 3D keypoints

            # If either of the keypoints is None, continue without appending or processing invalid data
            else:
                continue


        cv.destroyAllWindows()
        for cap in caps:
            cap.release()

        return np.array(kpts_cam0), np.array(kpts_cam1), np.array(kpts_3d)
    
    def run(self):
        while not rospy.is_shutdown():
            hand_join_state_publisher = HandJointStatePublisher()

            input_stream1 = 0
            input_stream2 = 2

            P0 = get_projection_matrix(0)
            P1 = get_projection_matrix(1)

            hand_join_state_publisher.run_mp(input_stream1, input_stream2, P0, P1)


if __name__ == '__main__':
    while not rospy.is_shutdown():
        hand_join_state_publisher = HandJointStatePublisher()

        input_stream1 = 0
        input_stream2 = 2

        # if len(sys.argv) == 3:
        #     input_stream1 = int(sys.argv[1])
        #     input_stream2 = int(sys.argv[2])

        P0 = get_projection_matrix(0)
        P1 = get_projection_matrix(1)

        kpts_cam0, kpts_cam1, kpts_3d = hand_join_state_publisher.run_mp(input_stream1, input_stream2, P0, P1)

        write_keypoints_to_disk('kpts_cam0.dat', kpts_cam0)
        write_keypoints_to_disk('kpts_cam1.dat', kpts_cam1)
        write_keypoints_to_disk('kpts_3d.dat', kpts_3d)

