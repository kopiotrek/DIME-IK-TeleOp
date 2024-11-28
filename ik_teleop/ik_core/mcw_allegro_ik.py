import numpy as np

class IKControl:
    def __init__(self):
        # Initialize joint angles (q)
        self.q = np.zeros(5)
        # Initialize DH parameters
        self.SetDHParams(self.q)
        # Compute initial TEE
        self.ComputeTEE()
        self.CurrentTEE = self.TEE.copy()
        self.EE_VEL_LIMIT = 1000000000000000.0  # Set appropriate values
        self.JOINT_VEL_LIMIT = 100000000000000.0  # Set appropriate values
        # Initialize desired position and orientation
        self.position = np.zeros(3)
        self.rotation = np.array([1, 0, 0, 0])  # Quaternion [w, x, y, z]
        self.x_des = np.eye(4)
        self.eeVelPosition = np.zeros(3)
        self.eeVelRotation = np.zeros(3)
        self.lastWasError = False

    def SetPosition(self, newPosition):
        self.position = newPosition

    def GetCurrentState(self):
        return self.q.copy()

    def SetDHParams(self, q):
# DH Parameters: (markdown)
# |    | joint   | parent   | child   |        d |     theta |      r |     alpha |
# |---:|:--------|:---------|:--------|---------:|----------:|-------:|----------:|
# |  0 | joint0  | link0    | link1   | -0.26696 | -180      | 0.0182 | -175      |
# |  1 | joint1  | link1    | link2   | -0.19482 |  180      | 0.005  |   90.0002 |
# |  2 | joint2  | link2    | link3   |  0.0576  |   90.0002 | 0      |   90.0002 |
# |  3 | joint3  | link3    | link4   |  0       |   90.0002 | 0.0514 |    0      |
# |  4 | joint4  | link4    | link5   |  0       |   -0      | 0.0423 |  -90.0002 |
        # DH parameters: [a_i, d_i, alpha_i, theta_i]
        # self.dhParams = [
        #     [0.0182,   -0.26696,   -3.05432619,     q[0]],
        #     [0.005,    -0.19482,   np.pi / 2,       q[1]],
        #     [0.0,      0.0576,     np.pi / 2,       q[2]],
        #     [0.0514,   0.0,        0.0,             q[3]],
        #     [0.0423,   0.0,        -np.pi / 2,      q[4]],
        # ]
        self.dhParams = [
            [-0.26696,  0.0182,   -3.05432619,     q[0]],
            [-0.19482,  0.005,   np.pi / 2,       q[1]],
            [0.0576,    0.0,     np.pi / 2,       q[2]],
            [0.0,       0.0514,        0.0,             q[3]],
            [0.0,       0.0423,        -np.pi / 2,      q[4]],
        ]

    def GetTransformationMatrix(self, i, dh):
        a = dh[i][0]
        d = dh[i][1]
        alpha = dh[i][2]
        theta = dh[i][3]

        ca = np.cos(alpha)
        sa = np.sin(alpha)
        ct = np.cos(theta)
        st = np.sin(theta)

        T = np.array([
            [ct, -st,  0,  a],
            [st*ca, ct*ca, -sa, -sa*d],
            [st*sa, ct*sa, ca,  ca*d],
            [0,     0,     0,   1]
        ])
        return T

    def ComputeTEE(self):
        self.TEE = np.eye(4)
        for i in range(5):
            T = self.GetTransformationMatrix(i, self.dhParams)
            self.TEE = np.dot(self.TEE, T)

    def ComputeJacobian(self):
        self.J = np.zeros((6, 5))
        T = np.eye(4)
        for i in range(5):
            T = np.dot(T, self.GetTransformationMatrix(i, self.dhParams))
            p = self.TEE[0:3, 3] - T[0:3, 3]
            z = T[0:3, 2]
            Jp = np.cross(z, p)
            Jo = z
            self.J[0:3, i] = Jp
            self.J[3:6, i] = Jo

    def ComputeIKControl(self, x_des):
        q = self.GetCurrentState()
        q_backup = q.copy()

        # Extract individual axes from the desired rotation matrix
        desiredNormal = x_des[0:3, 0]
        desiredTangent = x_des[0:3, 1]
        desiredBinormal = x_des[0:3, 2]

        error = 1000
        current_iter = 0
        while error > 0.05 and current_iter < 100:
            current_iter += 1

            self.SetDHParams(q)
            self.ComputeTEE()
            self.ComputeJacobian()

            # Extract individual axes from the current rotation matrix
            currentNormal = self.TEE[0:3, 0]
            currentTangent = self.TEE[0:3, 1]
            currentBinormal = self.TEE[0:3, 2]

            # Calculate position and orientation errors
            positionError = x_des[0:3, 3] - self.TEE[0:3, 3]
            orientationError = 0.5 * (
                np.cross(currentNormal, desiredNormal) +
                np.cross(currentTangent, desiredTangent) +
                np.cross(currentBinormal, desiredBinormal)
            )

            x_error = np.concatenate((positionError, 0.1 * orientationError))

            dq = np.linalg.pinv(self.J).dot(x_error)
            error = np.linalg.norm(x_error)

            q += dq

            # Check for joint velocity limits
            if np.max(np.abs((q - q_backup))) / 0.01 > self.JOINT_VEL_LIMIT:
                print("[IK] Joint velocity limit exceeded during IK")
                if not self.lastWasError:
                    self.lastWasError = True
                    # Handle the error as needed
                return q_backup

            # Check for end-effector velocity limits
            if np.linalg.norm(positionError) / 0.01 > self.EE_VEL_LIMIT:
                print("[IK] End-effector velocity limit exceeded during IK")
                if not self.lastWasError:
                    self.lastWasError = True
                    # Handle the error as needed
                return q_backup

        if current_iter == 10000:
            print("[IK] IK could not converge")
            # Handle the error as needed
            return q_backup

        self.lastWasError = False
        # Handle successful IK computation as needed

        return q

    def QuaternionToMatrix(self, quaternion):
        w, x, y, z = quaternion
        xx = x * x
        yy = y * y
        zz = z * z
        xy = x * y
        xz = x * z
        yz = y * z
        wx = w * x
        wy = w * y
        wz = w * z

        rotationMatrix = np.eye(4)
        rotationMatrix[0, 0] = 1 - 2 * (yy + zz)
        rotationMatrix[0, 1] = 2 * (xy - wz)
        rotationMatrix[0, 2] = 2 * (xz + wy)

        rotationMatrix[1, 0] = 2 * (xy + wz)
        rotationMatrix[1, 1] = 1 - 2 * (xx + zz)
        rotationMatrix[1, 2] = 2 * (yz - wx)

        rotationMatrix[2, 0] = 2 * (xz - wy)
        rotationMatrix[2, 1] = 2 * (yz + wx)
        rotationMatrix[2, 2] = 1 - 2 * (xx + yy)

        return rotationMatrix

    def Update(self):
        # Compute desired transformation matrix
        x_des = self.QuaternionToMatrix(self.rotation)
        x_des[0:3, 3] = self.position

        # Compute IK control
        q = self.ComputeIKControl(x_des)
        print(f"q {q*np.pi/180}")
        self.q = q
        self.SetDHParams(self.q)
        self.ComputeTEE()
        self.ComputeEEVelocity()
        self.CurrentTEE = self.TEE.copy()
        # Apply joint limits as needed

    def ComputeEEVelocity(self):
        dTEE = self.TEE - self.CurrentTEE
        delta_t = 0.01  # Assuming time step of 0.01 sec

        self.eeVelPosition = dTEE[0:3, 3] / delta_t
        # For rotation, compute the difference in rotation matrices
        dR = np.dot(self.TEE[0:3, 0:3], self.CurrentTEE[0:3, 0:3].T)
        angle_axis = self.rotation_matrix_to_angle_axis(dR)
        self.eeVelRotation = angle_axis / delta_t

    def rotation_matrix_to_angle_axis(self, R):
        angle = np.arccos((np.trace(R) - 1) / 2)
        if angle == 0:
            return np.zeros(3)
        else:
            rx = R[2,1] - R[1,2]
            ry = R[0,2] - R[2,0]
            rz = R[1,0] - R[0,1]
            axis = np.array([rx, ry, rz])
            axis = axis / (2 * np.sin(angle))
            return angle * axis


if __name__ == '__main__':
    ik_control = IKControl()

    # Set desired position and orientation
    ik_control.position = np.array([ 0.02698166,  0.16099207, -0.07196472])  # Example position
    ik_control.rotation = np.array([1, 0, 0, 0])     # Identity quaternion

    # Perform the IK update
    ik_control.Update()