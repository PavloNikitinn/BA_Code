import numpy as np
#Helper class to rotate with quaternions
class ObjectOrientation:
    def __init__(self):
        self.current_orientation = np.array([1.0, 0., 0., 0.])  # init rotation

    def quaternion_multiply(self, q1, q2):
        # Quat multiplication
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        return np.array([w, x, y, z])

    def rotate_acceleration(self, accel_data,qort):
        # accel to quat
        accel_quat = np.array([0, accel_data[0], accel_data[1], accel_data[2]])

        # Quat mult q_rotated = q_current * accel_quat * q_conjugate_current
        q_current = qort
        q_conjugate_current = np.array([q_current[0], -q_current[1], -q_current[2], -q_current[3]])

        q_rotated = self.quaternion_multiply(q_current, accel_quat)
        q_rotated = self.quaternion_multiply(q_rotated, q_conjugate_current)

        # get xyz from vector 
        rotated_acceleration = q_rotated[1:]

        return rotated_acceleration
    
