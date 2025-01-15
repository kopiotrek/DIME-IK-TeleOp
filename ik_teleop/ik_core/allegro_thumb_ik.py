import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from scipy.optimize import minimize

from numpy.linalg import norm, solve
 
import pinocchio

class ThumbIK:
    def __init__(self):
        # self.articulation_chain = [{"position": 0.225} for _ in range(4)]
        self.articulation_chain = [{"position": 0.225} for _ in range(3)]
        self.position = np.zeros(3)
        self.rotation = np.eye(3)
        self.x_des = np.eye(4)
        self.dh_params = []
        self.TEE = np.eye(4)
        self.CurrentTEE = np.eye(4)
        self.J = np.zeros((6, 4))  # Jacobian is now 6x4 for 4 joints
        self.ee_vel_position = np.zeros(3)
        self.ee_vel_rotation = np.zeros(3)
        # self.joint_positions = []  # Start with the base position
        self.JOINT_COUNT = 3
        # Add joint limits
        self.q_min = np.array([0.225, -0.368, -0.281, -0.262])  # Minimum joint angles
        self.q_max = np.array([1.555, 1.152, 1.719, 1.799])    # Maximum joint angles
        self.model = pinocchio.buildModelFromUrdf("/home/mcw/RPL/DIME-IK-TeleOp/ik_teleop/robot/assets/thumb.urdf")
        self.data = self.model.createData()

    def get_current_state(self):
        return np.array([joint["position"] for joint in self.articulation_chain])

    def reset_to_joints(self, joint_angles):
        if len(joint_angles) != len(self.articulation_chain):
            raise ValueError(f"Expected {len(self.articulation_chain)} joint angles, but got {len(joint_angles)}")
        for i, angle in enumerate(joint_angles):
            self.articulation_chain[i]["position"] = angle
        self.set_dh_params(joint_angles)
        self.compute_TEE()

    def set_dh_params(self, joint_angles):
        self.dh_params = [
            [0.0, 0.0,  1.57079,           joint_angles[0]],          # Joint 1
            [0.0, 0.0554,  -1.57079,       joint_angles[1]-np.pi/2], # Joint 2
            [0.0514, 0.0,  0.0,            joint_angles[2]-np.pi/2],          # Joint 3
            # [0.0593, 0.0,  0.0,            joint_angles[3]]           # Joint 4 (End-Effector)
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
        # self.TEE = np.eye(4)
        # self.joint_positions = [np.zeros(3)]  # Start with the base position
        T = np.eye(4)
        for i in range(len(self.dh_params)):
            T = T @ self.get_transformation_matrix(i, self.dh_params)
            # self.joint_positions.append(T[:3, 3])  # Store joint positions
        self.TEE = T
        # print(f"self.joint_positions {self.joint_positions}")

    def compute_jacobian(self):
        self.J = np.zeros((6, 4))
        T = np.eye(4)
        zs = []
        ps = [np.zeros(3)]
        for i in range(self.JOINT_COUNT):
            T = T @ self.get_transformation_matrix(i, self.dh_params)
            zs.append(T[:3, 2])
            ps.append(T[:3, 3])
        for i in range(self.JOINT_COUNT):
            p = self.TEE[:3, 3] - ps[i]
            z = zs[i]
            self.J[:3, i] = np.cross(z, p)
            self.J[3:, i] = z


    def compute_ik(self, desired_position):
        """
        Compute inverse kinematics using optimization with joint limits.
        """
        def objective(q):
            # Update DH parameters and compute forward kinematics
            self.set_dh_params(q)
            self.compute_TEE()
            # Compute position error
            pos_error = desired_position - self.TEE[:3, 3]
            print(f"TEE: {self.TEE[:3, 3]}")
            # Return squared error norm
            return np.sum(pos_error**2)

        # Initial joint angles
        q0 = self.get_current_state().astype(float)

        # Bounds for joint limits as a sequence of (min, max) pairs
        bounds = [(self.q_min[i], self.q_max[i]) for i in range(len(q0))]

        # Solve IK using optimization
        result = minimize(
            objective,
            q0,
            method='SLSQP',
            bounds=bounds,
            options={'ftol': 0.0001, 'maxiter': 100}
        )

        if result.success:
            # print(f"Converged in {result.nit} iterations.")
            q = result.x
        else:
            print("IK did not converge.")
            q = q0  # Return initial guess or handle as needed

        return q

    # def compute_ik(self, desired_position):
    #     """
    #     Compute inverse kinematics using optimization with joint limits.
    #     """
    #     # Initial joint angles
    #     q = self.get_current_state().astype(float)

    #     eps    = 1e-3
    #     IT_MAX = 1000
    #     DT     = 1e-1
    #     damp   = 1e-12
    #     JOINT_ID = 4
    #     oMdes = pinocchio.SE3(np.eye(3), np.array([0., 1., 0.]))
    #     i=0
    #     q = pinocchio.neutral(self.model)
    #     print(f"{self.model}, {self.data}, {q}")
    #     while True:
    #         pinocchio.forwardKinematics(self.model,self.data,q)
    #         dMi = oMdes.actInv(self.data.oMi[JOINT_ID])
    #         err = pinocchio.log(dMi).vector
    #         if norm(err) < eps:
    #             success = True
    #             break
    #         if i >= IT_MAX:
    #             success = False
    #             break
    #         J = pinocchio.computeJointJacobian(self.model,self.data,q,JOINT_ID)
    #         v = - J.T.dot(solve(J.dot(J.T) + damp * np.eye(6), err))
    #         q = pinocchio.integrate(self.model,q,v*DT)
    #         # if not i % 10:
    #             # print('%d: error = %s' % (i, err.T))
    #         i += 1
        

    #     if success:
    #         print("Convergence achieved!")

    #     else:
    #         print("IK did not converge.")
    #         q = self.get_current_state().astype(float)  # Return initial guess or handle as needed

    #     return q

    def visualize(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        ax.set_title("4 DOF Robotic Arm")
        ax.set_xlabel("X-axis")
        ax.set_ylabel("Y-axis")
        ax.set_zlabel("Z-axis")
        # Extract joint positions for plotting
        xs, ys, zs = zip(*self.joint_positions)
        # Set equal aspect ratio and fixed axis limits
        ax.set_box_aspect([1, 1, 1])
        max_range = 0.1  # Define a fixed range for all axes (adjust as needed)
        ax.set_xlim([-max_range, max_range])
        ax.set_ylim([-max_range, max_range])
        ax.set_zlim([-max_range, max_range])  # Typically, Z starts at 0 for robotic arms
        # Plot the arm
        ax.plot(xs, ys, zs, marker="o", linestyle="-", color="b", label="Arm Links")
        # Annotate joints
        for i, (x, y, z) in enumerate(self.joint_positions):
            ax.text(x, y, z, f"J{i}", color="red")
        # Set equal aspect ratio
        ax.set_box_aspect([1, 1, 1])
        ax.legend()
        plt.show()

def main():
    ik_control = ThumbIK()
    # desired_position = [-0.00694682, -0.01076413, -0.02820065]
    desired_position = [0.038194, 0.0015385, -0.00045907]
    # Compute IK to find joint angles
    computed_joint_angles = ik_control.compute_ik(desired_position)
    # print("Computed Joint Angles (IK):", computed_joint_angles)
    # Reset to computed joint angles and visualize
    ik_control.reset_to_joints(computed_joint_angles)
    # print("End-Effector Position (FK):", ik_control.TEE[:3, 3])
    ik_control.visualize()

# def main():
#     ik_control = ThumbIK()
#     # Define joint angles for FK
#     joint_angles = [1, 1.57, 0, 1]
#     ik_control.reset_to_joints(joint_angles)
#     ik_control.compute_TEE()
#     # print("End-Effector Position (FK):", ik_control.TEE[:3, 3])
#     # print("End-Effector Orientation (FK):", ik_control.TEE[:3, :3])
#     ik_control.visualize()
#     # Define desired end-effector position for IK
#     desired_position = ik_control.TEE[:3, 3]  # Use FK result for testing
#     # desired_position = [-0.00694682, -0.01076413, -0.02820065]
#     desired_position = [-0.07697249, -0.05173028,  0.14050266]
#     # Compute IK to find joint angles
#     computed_joint_angles = ik_control.compute_ik(desired_position)
#     # print("Computed Joint Angles (IK):", computed_joint_angles)
#     # Reset to computed joint angles and visualize
#     ik_control.reset_to_joints(computed_joint_angles)
#     # print("End-Effector Position (FK):", ik_control.TEE[:3, 3])
#     ik_control.visualize()

if __name__ == "__main__":
    main()
