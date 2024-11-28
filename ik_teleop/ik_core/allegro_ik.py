import numpy as np
from numpy import sin, cos, pi
from scipy.optimize import least_squares

# DH Parameters
# Joint i : [theta_i (variable), d_i, a_{i-1}, alpha_{i-1}]
# DH_params = [
#     [None, -0.045987, 0.0265,  pi/2],   # Joint 1
#     [None, 0.005,    -0.027,   0],      # Joint 2
#     [None, 0.0177,    0,        0],     # Joint 3
#     [None, 0.0514,    0,        0]      # Joint 4
# ]
# DH_params = [
#     [None, 0.035,   0.0,     np.pi/2],   # Joint 1
#     [None, 0.018,   0.0,     np.pi/2],      # Joint 2
#     [None, 0,       0.052,   0],     # Joint 3
#     [None, 0,       0.06,    0]      # Joint 4
# ]
DH_params = [
    [None, 0.0,     0.0,     np.pi/2],   # Joint 1
    [None, 0.035,   0.0,     np.pi/2],      # Joint 2
    [None, 0.018,   0.0,     0],     # Joint 3
    [None, 0,       0.052,   0],      # Joint 4
    [None, 0,       0.06,    0]      # Joint 4
]
# DH Parameters: (markdown)
# |    | joint   | parent   | child   |        d |     theta |      r |     alpha |
# |---:|:--------|:---------|:--------|---------:|----------:|-------:|----------:|
# |  0 | joint0  | link0    | link1   | -0.26696 | -180      | 0.0182 | -175      |
# |  1 | joint1  | link1    | link2   | -0.19482 |  180      | 0.005  |   90.0002 |
# |  2 | joint2  | link2    | link3   |  0.0576  |   90.0002 | 0      |   90.0002 |
# |  3 | joint3  | link3    | link4   |  0       |   90.0002 | 0.0514 |    0      |
# |  4 | joint4  | link4    | link5   |  0       |   -0      | 0.0423 |  -90.0002 |

def DH_transform(theta, d, a, alpha):
    """Create the DH transformation matrix using modified DH parameters."""
    return np.array([
        [cos(theta),             -sin(theta),            0,              a],
        [sin(theta)*cos(alpha),  cos(theta)*cos(alpha), -sin(alpha), -d*sin(alpha)],
        [sin(theta)*sin(alpha),  cos(theta)*sin(alpha),  cos(alpha),  d*cos(alpha)],
        [0,                      0,                      0,              1]
    ])

def forward_kinematics(thetas):
    """Compute the forward kinematics for given joint angles."""
    T = np.eye(4)
    for i, params in enumerate(DH_params):
        theta_i = thetas[i]
        d_i = params[1]
        a_i = params[2]
        alpha_i = params[3]
        T_i = DH_transform(theta_i, d_i, a_i, alpha_i)
        T = np.dot(T, T_i)

    return T

def ik_solver(desired_pose):
    """
    Compute the inverse kinematics.
    desired_pose: 4x4 homogeneous transformation matrix representing the desired end-effector pose.
    Returns the joint angles [theta1, theta2, theta3, theta4].
    """
    def residuals(thetas):
        T = forward_kinematics(thetas)
        # Compute the position error
        position_error = T[:3, 3] - desired_pose[:3, 3]
        return position_error  # Length 3
    
    # Initial guess for the joint angles
    initial_guess = [0.263, 0.0, 0.0, 0.0, 0.0]
    bounds = ([-np.pi]*5, [np.pi]*5)  # Adjust according to joint limits
    bounds_lower = [0.263, -0.105, -0.189, -0.162, 0.0]
    bounds_upper = [1.396, 2, 1.644, 1.719, 0.1]
    bounds = (bounds_lower, bounds_upper)
    # Solve the least squares problem
    result = least_squares(residuals, initial_guess, bounds=bounds)
    
    if result.success:
        return result.x  # Return the joint angles
    else:
        raise ValueError("IK solution did not converge: " + result.message)

# Example usage
if __name__ == "__main__":
    # Desired end-effector pose (example)
    desired_position = np.array([-0.01, 0.0, 0.0])  # Adjusted for reachable position
    desired_orientation = np.eye(3)  # Identity matrix for simplicity

    # Construct the desired pose matrix
    desired_pose = np.eye(5)
    desired_pose[:3, :3] = desired_orientation
    desired_pose[:3, 3] = desired_position

    try:
        joint_angles = ik_solver(desired_pose)
        formatted_joint_angles_rad = [f"{angle:.4f}" for angle in joint_angles]
        print("Joint Angles (in radians):", formatted_joint_angles_rad)

        # Validate the solution
        T = forward_kinematics(joint_angles)
        computed_position = T[:3, 3]
        desired_position = desired_pose[:3, 3]
        # print("Position Error:", T[:3, 3] - desired_pose[:3, 3])
        # Computed Position
        formatted_computed_pos = [f"{coord:.4f}" for coord in computed_position]
        print("Computed End-Effector Position:", formatted_computed_pos)

        # Desired Position
        formatted_desired_pos = [f"{coord:.4f}" for coord in desired_position]
        print("Desired End-Effector Position:", formatted_desired_pos)

        # # Position Error
        # formatted_position_error = [f"{error:.4f}" for error in position_error]
        # print("Position Error:", formatted_position_error)
    except ValueError as e:
        print(e)
