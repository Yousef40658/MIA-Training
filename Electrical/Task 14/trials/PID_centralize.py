 #!/usr/bin/env python
import rospy 
from std_msgs.msg import Int16, Int16MultiArray, Float32

#take the distance from centralize.py
#use PID to centralize the robot
#change the velocity of the robot to centralize it
#publish on /cmd_vel topic
#ultrasonic[front, right, left]
from geometry_msgs.msg import Twist

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = 0.0  # Desired setpoint (centralized position)
        self.previous_error = 0
        self.integral = 0

    def compute(self, distance, dt):
            # Calculate error
            error = self.setpoint - distance

            # Proportional term
            P_out = self.Kp * error
            
            # Integral term
            self.integral += error * dt
            I_out = self.Ki * self.integral
            
            # Derivative term
            derivative = (error - self.previous_error) / dt
            D_out = self.Kd * derivative
            
            # Compute total output
            output = P_out + I_out + D_out
            
            # Update previous error
            self.previous_error = error
        
            return output


def centralize_robot(distance):
    pid = PIDController(Kp=1.0, Ki=0.0, Kd=0.1)
    control_signal = pid.compute(distance, dt=0.1)

    # scale / saturate control signal
    max_speed = 0.3   # m/s
    min_speed = -0.3  # m/s

    # Clip value inside range
    velocity_y = max(min(control_signal, max_speed), min_speed)

    # Create Twist message
    cmd_vel = Twist()
    cmd_vel.linear.y = velocity_y

    pub.publish(cmd_vel)


if __name__ == "__main__":
    rospy.init_node("PID_centralize", anonymous=True)
    pid = PIDController(1.0, 0.0, 0.1)
    last_time = rospy.Time.now()

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rospy.Subscriber("/PID_US", Float32, centralize_robot)

