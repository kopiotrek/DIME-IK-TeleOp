import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import JointState
from ik_teleop.teleop_utils.vectorops import *

class DevThumbIK:
    def __init__(self):
        self.articulation_chain = [{"position": 0.225} for _ in range(4)]
        self.position = np.zeros(3)
        self.rotation = np.eye(3)
        self.x_des = np.eye(4)
        self.dh_params = []
        self.TEE = np.eye(4)
        self.CurrentTEE = np.eye(4)
        self.J = np.zeros((6, 4))  # Jacobian is now 6x4 for 4 joints
        self.ee_vel_position = np.zeros(3)
        self.ee_vel_rotation = np.zeros(3)
        self.joint_positions = [np.zeros(3)]  # Start with the base position
        # Add joint limits
        self.q_min = np.array([-0.57, -0.296, -0.274, -0.327])  # Minimum joint angles
        self.q_max = np.array([0.57, 1.71, 1.809, 1.718])    # Maximum joint angles

        # Visualization attributes
        self.fig = None
        self.ax = None
        self.lines = None
        self.texts = []
        self.latest_joint_angles = None  # Store latest joint angles

    def get_current_state(self):
        return np.array([joint["position"] for joint in self.articulation_chain])

    def reset_to_joints(self, joint_angles):
        if len(joint_angles) != len(self.articulation_chain):
            raise ValueError(f"Expected {len(self.articulation_chain)} joint angles, but got {len(joint_angles)}")
        for i, angle in enumerate(joint_angles):
            self.articulation_chain[i]["position"] = angle
        self.set_dh_params(joint_angles)
        self.compute_TEE()

    # def set_dh_params(self, joint_angles):
    #     self.dh_params = [
    #         [0.0, 0.0,  1.57079,           joint_angles[0]],          # Joint 1
    #         [0.0, 0.0554,  -1.57079,       joint_angles[1]-np.pi/2], # Joint 2
    #         [0.0514, 0.0,  0.0,            joint_angles[2]-np.pi/2],          # Joint 3
    #         [0.0593, 0.0,  0.0,            joint_angles[3]]           # Joint 4 (End-Effector)
    #     ]
    def set_dh_params(self, joint_angles):
        self.dh_params = [
            # Trans X, Trans Z, Rot X, Rot Z
            [0.0,       0.0166,  -np.pi/2,          joint_angles[0]],          # Joint 1
            [0.054,     0.0,     0.0,               joint_angles[1]-np.pi/2],         # Joint 2
            [0.0384,    0.0,     0.0,               joint_angles[2]],          # Joint 3
            [0.0437,    0.0,     0.0,               joint_angles[3]]           # Joint 4
        ]

    def get_transformation_matrix(self, i, dh):
        a, d, alpha, theta = dh[i]
        return np.array([
            [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta)*np.sin(alpha), a * np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])

    def compute_TEE(self):
        self.TEE = np.eye(4)
        self.joint_positions = [np.zeros(3)]  # Start with the base position
        T = np.eye(4)
        for i in range(len(self.dh_params)):
            T = T @ self.get_transformation_matrix(i, self.dh_params)
            self.joint_positions.append(T[:3, 3])  # Store joint positions
        self.TEE = T

    def compute_jacobian(self):
        self.J = np.zeros((6, 4))
        T = np.eye(4)
        zs = []
        ps = [np.zeros(3)]
        for i in range(4):
            T = T @ self.get_transformation_matrix(i, self.dh_params)
            zs.append(T[:3, 2])
            ps.append(T[:3, 3])
        for i in range(4):
            p = self.TEE[:3, 3] - ps[i]
            z = zs[i]
            self.J[:3, i] = np.cross(z, p)
            self.J[3:, i] = z

    def compute_ik(self, desired_position):
        """
        Compute inverse kinematics for position-only control with joint limits.
        """
        self.x_des[:3, 3] = desired_position  # Set desired position
        q = self.get_current_state().astype(float)  # Initial joint angles
        max_iterations = 10000
        tolerance = 0.01
        for iteration in range(max_iterations):
            # Update DH parameters and compute the forward kinematics
            self.set_dh_params(q)
            self.compute_TEE()
            self.compute_jacobian()
            # Compute position error only
            pos_error = self.x_des[:3, 3] - self.TEE[:3, 3]
            error = pos_error  # Only consider positional errors
            # Check if the error is within the tolerance
            if np.linalg.norm(error) < tolerance:
                print(f"Converged in {iteration + 1} iterations.")
                break
            # Use only the translational part of the Jacobian (first 3 rows)
            J_translational = self.J[:3, :]  # 3xN matrix for N DOF
            # Compute change in joint angles using the pseudoinverse of the translational Jacobian
            dq = np.linalg.pinv(J_translational) @ error
            # Update joint angles
            q += dq
            # Enforce joint limits
            q = np.clip(q, self.q_min, self.q_max)
        else:
            print("IK did not converge within the maximum number of iterations.")
        return q

    def init_visualization(self):
        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.ax.set_title("4 DOF Robotic Arm")
        self.ax.set_xlabel("X-axis")
        self.ax.set_ylabel("Y-axis")
        self.ax.set_zlabel("Z-axis")
        # Set equal aspect ratio and fixed axis limits
        self.ax.set_box_aspect([1, 1, 1])
        max_range = 0.1  # Define a fixed range for all axes (adjust as needed)
        self.ax.set_xlim([-max_range, max_range])
        self.ax.set_ylim([-max_range, max_range])
        self.ax.set_zlim([-max_range, max_range])  # Typically, Z starts at 0 for robotic arms
        self.lines, = self.ax.plot([], [], [], marker="o", linestyle="-", color="b", label="Arm Links")
        self.texts = []
        self.ax.legend()
        axis_length = 0.2  # Length of the arrows
        self.ax.quiver(0, 0, 0, axis_length, 0, 0, color='r', linewidth=2, arrow_length_ratio=0.1, label='X (positive)')
        self.ax.quiver(0, 0, 0, -axis_length, 0, 0, color='r', linewidth=2, arrow_length_ratio=0.1, linestyle='dotted', label='X (negative)')
        self.ax.quiver(0, 0, 0, 0, axis_length, 0, color='g', linewidth=2, arrow_length_ratio=0.1, label='Y (positive)')
        self.ax.quiver(0, 0, 0, 0, -axis_length, 0, color='g', linewidth=2, arrow_length_ratio=0.1, linestyle='dotted', label='Y (negative)')
        self.ax.quiver(0, 0, 0, 0, 0, axis_length, color='b', linewidth=2, arrow_length_ratio=0.1, label='Z (positive)')
        self.ax.quiver(0, 0, 0, 0, 0, -axis_length, color='b', linewidth=2, arrow_length_ratio=0.1, linestyle='dotted', label='Z (negative)')

        plt.show()

    def update_visualization(self):
        xs, ys, zs = zip(*self.joint_positions)
        self.lines.set_data(xs, ys)
        self.lines.set_3d_properties(zs)
        # Remove old texts
        for txt in self.texts:
            txt.remove()
        self.texts = []
        # Annotate joints
        for i, (x, y, z) in enumerate(self.joint_positions):
            txt = self.ax.text(x, y, z, f"J{i}", color="red")
            self.texts.append(txt)
        plt.draw()
        plt.pause(0.001)

    # def get_hand_state(self):
    #     if self.allegro_joint_state is None:
    #         return None

    #     raw_joint_state = copy(self.allegro_joint_state)

    #     joint_state = dict(
    #         position = np.array(raw_joint_state.position, dtype = np.float32),
    #         velocity = np.array(raw_joint_state.velocity, dtype = np.float32),
    #         effort = np.array(raw_joint_state.effort, dtype = np.float32),
    #         timestamp = raw_joint_state.header.stamp.secs + (raw_joint_state.header.stamp.nsecs * 1e-9)
    #     )
    #     return joint_state

def main():
    rospy.init_node('thumb_ik_visualizer', anonymous=True)
    ik_control = DevThumbIK()
    ik_control.init_visualization()
    ik_control.latest_joint_angles = None
    def joint_state_callback(msg):
        # Map joint names to positions
        joint_positions = dict(zip(msg.name, msg.position))
        # Extract thumb joints
        thumb_joint_names = ["joint_0.0", "joint_1.0", "joint_2.0", "joint_3.0"]
        ik_control.latest_joint_angles = [joint_positions[name] for name in thumb_joint_names]
        # print(f"ik_control.latest_joint_angles {ik_control.latest_joint_angles}")
    rospy.Subscriber('allegroHand/joint_states', JointState, joint_state_callback)
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        if ik_control.latest_joint_angles is not None:
            ik_control.reset_to_joints(ik_control.latest_joint_angles)
            ik_control.compute_TEE()
            ee = ik_control.TEE[:3, 3]
            print("End-Effector Position (FK):", ee)
            rotation_angles = (np.deg2rad(5), 0, 0)
            ee = rotate_point(ee, rotation_angles)
            ee += [0, -0.045098, -0.014293]

            print("Transformed End-Effector Position (FK):", ee)
            ik_control.update_visualization()
        rate.sleep()

if __name__ == "__main__":
    main()
