


class CmdVelScaler:
    def __init__(self, left_rate=1.0, right_rate=1.0, wheel_separation=0.5):
        """
        Initialize the CmdVelScaler with motor scaling rates and robot geometry.
        
        :param left_rate: Scaling factor for the left wheel (0.0-1.0)
        :param right_rate: Scaling factor for the right wheel (0.0-1.0)
        :param wheel_separation: Distance between wheels in meters
        """
        self.left_rate = left_rate
        self.right_rate = right_rate
        self.wheel_separation = wheel_separation

    def scale_cmd_vel(self, linear_x, angular_z):
        """
        Convert Twist command to scaled wheel velocities using differential drive kinematics.
        
        :param linear_x: Linear velocity in x-direction (m/s)
        :param angular_z: Angular velocity around z-axis (rad/s)
        :return: Tuple of (left_wheel_velocity, right_wheel_velocity) in m/s
        """
        # Calculate base wheel velocities
        left_base = linear_x - (self.wheel_separation / 2) * angular_z
        right_base = linear_x + (self.wheel_separation / 2) * angular_z
        
        # Apply motor scaling factors
        left_scaled = left_base * self.left_rate
        right_scaled = right_base * self.right_rate
        
        # Return scaled cmd velocities x and theta
        scaled_x = (left_scaled + right_scaled) / 2
        scaled_theta = (right_scaled - left_scaled) / self.wheel_separation
        
        return scaled_x, scaled_theta

  
        
        