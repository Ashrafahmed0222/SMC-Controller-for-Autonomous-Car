#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
import numpy as np
from utils import calculate_error, calculate_error_derivative, sliding_mode_control, saturate
from scipy.signal import lfilter
import RPi.GPIO as GPIO
import time

class SlidingModeControllerNode:
    def __init__(self):
        # Initialize GPIO resources early to ensure cleanup safety
        self.servo_pwm = None
        self.motor_pwm = None
         # Trajectory parameters
        self.trajectory = self.generate_trajectory()
        self.trajectory_index = 0

        try:
            rospy.init_node('sliding_mode_controller', anonymous=True)
            
            # Parameters
            self.L = rospy.get_param("~wheelbase", 0.27)  # Wheelbase length
            self.vr = rospy.get_param("~reference_velocity", 0.3)  # Max desired velocity
            self.omegad = rospy.get_param("~desired_angular_velocity", 0.2)
            
            # State variables
            self.current_x = 0.0
            self.current_y = 0.0
            self.current_theta = 0.0
            self.current_velocity = 0.0
            self.vr_dot = 0.0
            self.phi = 0.0  # Instance variable for phi
            self.v_c = 0.0
            # Transfer function coefficients for phi and v_c
            self.v_c_tf_den = [3, 1]  # Numerator of phi's transfer function
            self.v_c_tf_num = [1]     # Denominator of phi's transfer function
            self.phi_tf_num = [(2 * np.pi * 5) ** 2]  # Numerator of v_c's transfer function
            self.phi_tf_den = [1, 2 * 0.7 * 2 * np.pi * 5, (2 * np.pi * 5) ** 2]  # Denominator

            # Initialize filter states for transfer functions
            self.phi_filter_state = np.zeros(max(len(self.phi_tf_num), len(self.phi_tf_den)) - 1)
            self.v_c_filter_state = np.zeros(max(len(self.v_c_tf_num), len(self.v_c_tf_den)) - 1)
             # Initialize filtered outputs to a default value
            self.filtered_phi = np.zeros(1)  # Single-element array for compatibility with lfilter
            self.filtered_v_c = np.zeros(1)  # Single-element array for compatibility with lfilter

            # GPIO Setup
            self.servo_pin = 18
            self.motor_pin = 19

            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.servo_pin, GPIO.OUT)
            GPIO.setup(self.motor_pin, GPIO.OUT)

            self.servo_pwm = GPIO.PWM(self.servo_pin, 50)  # 50Hz for servo
            self.motor_pwm = GPIO.PWM(self.motor_pin, 100)  # 100Hz for motor
            self.servo_pwm.start(0)
            self.motor_pwm.start(0)
            
            # ROS Subscribers
            self.heading_sub = rospy.Subscriber('/heading_angle', Float32, self.heading_callback)
            self.position_sub = rospy.Subscriber('/car_position', Point, self.position_callback)
            self.velocity_sub = rospy.Subscriber('/car_velocity', Float32, self.velocity_callback)

            rospy.loginfo("Sliding Mode Controller Node Initialized")

        except Exception as e:
            rospy.logerr(f"Initialization error: {e}")
            self.cleanup()
            raise

    def heading_callback(self, msg):
        self.current_theta = np.radians(msg.data)

    def position_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y

    def velocity_callback(self, msg):
        self.current_velocity = msg.data

    def generate_trajectory(self):
        """
        Generates a circular trajectory with:
        - A circle of radius 1 meter.
        - A full circle (2*pi radians).
        """
        trajectory = []
        radius = 1.0           # Radius of the circle
        center_x = 0.0         # Center of the circle (x-coordinate)
        center_y = 0.0         # Center of the circle (y-coordinate)
        n_points = 100         # Number of points on the circle

        # Generate trajectory points for the full circle
        for i in range(n_points):
            theta = 2 * np.pi * i / n_points  # Angle for each point
            xd = center_x + radius * np.cos(theta)  # X position on the circle
            yd = center_y + radius * np.sin(theta)  # Y position on the circle
            theta_d = theta  # Desired orientation (angle) at this point

            trajectory.append((xd, yd, theta_d))

        return trajectory

    def control_loop(self):
        rate = rospy.Rate(10)
        p = 0.0
        max_distance = 3.0

        while not rospy.is_shutdown() and self.trajectory_index < len(self.trajectory):
            # Get the current target from the trajectory
            xd, yd, theta_d = self.trajectory[self.trajectory_index]

            # Calculate errors
            xe, ye, thetae = calculate_error(
                self.current_x, self.current_y, self.current_theta, xd, yd, theta_d
            )
            xe_dot, ye_dot, thetae_dot = calculate_error_derivative(
                self.vr, self.phi, self.filtered_v_c[0], self.filtered_phi[0], xe, ye, thetae
            )

            # Calculate control inputs
            self.phi, self.v_c = sliding_mode_control(
                self.vr_dot, xe_dot, ye_dot, thetae_dot, xe, ye, thetae, self.omegad, self.filtered_v_c[0]
            )

            # Apply transfer functions
            self.filtered_phi, self.phi_filter_state = lfilter(
                self.phi_tf_num, self.phi_tf_den, [self.phi], zi=self.phi_filter_state
            )
            self.filtered_v_c, self.v_c_filter_state = lfilter(
                self.v_c_tf_num, self.v_c_tf_den, [self.v_c], zi=self.v_c_filter_state
            )
            MAX_delta = np.radians(60) 
            self.filtered_phi[0] = saturate(self.filtered_phi[0],-MAX_delta,MAX_delta)
            print("satphi")
            print(self.filtered_phi[0])
            # Use filtered values for GPIO output
            self.publish_to_gpio(self.filtered_phi[0], self.filtered_v_c[0])

            # Check if the robot is close enough to the current waypoint
            distance_to_target = np.sqrt((xd - self.current_x) ** 2 + (yd - self.current_y) ** 2)
            if distance_to_target < 0.05:  # Threshold for waypoint completion
                self.trajectory_index += 1

            rate.sleep()

        # Stop the robot after completing the trajectory
        self.publish_to_gpio(0, 0)
        rospy.loginfo("Trajectory completed. Stopping the robot.")

    def publish_to_gpio(self, phi, v_c):
        servo_duty = saturate(phi / 18.0 + 2, 0.0, 100.0)
        motor_duty = saturate(v_c * 100, 0.0, 100.0)

        if np.isfinite(servo_duty) and np.isfinite(motor_duty):
            self.servo_pwm.ChangeDutyCycle(servo_duty)
            self.motor_pwm.ChangeDutyCycle(motor_duty)
            rospy.loginfo(f"Servo Duty: {servo_duty:.2f}%, Motor Duty: {motor_duty:.2f}%")
        else:
            rospy.logerr("Invalid PWM values encountered.")

    def cleanup(self):
        if self.servo_pwm:
            self.servo_pwm.stop()
        if self.motor_pwm:
            self.motor_pwm.stop()
        GPIO.cleanup()
        rospy.loginfo("GPIO cleaned up and node shut down.")

if __name__ == "__main__":
    try:
        node = SlidingModeControllerNode()
        node.control_loop()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.cleanup()
