#!/usr/bin/env python3

import rospy
import time
import math
import RPi.GPIO as GPIO
from geometry_msgs.msg import Point
from std_msgs.msg import Float32

ENCODER_PPR = 312
ENCODER_PIN = 17
WHEEL_RADIUS = 0.033  # Wheel radius in meters

# Global variables
encoder_value = 0
previous_time = time.time()
theta = 0.0
x_position = 0.0
y_position = 0.0

def update_encoder(channel):
    global encoder_value
    encoder_value += 1

def heading_angle_callback(msg):
    global theta
    theta = math.radians(msg.data) # Ensure the heading angle is in radians

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(ENCODER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(ENCODER_PIN, GPIO.RISING, callback=update_encoder)

def main():
    global encoder_value, previous_time, x_position, y_position
    rospy.init_node("position_node", anonymous=True)

    # Subscriber for heading angle
    rospy.Subscriber("heading_angle", Float32, heading_angle_callback)

    # Publishers for position and total velocity
    position_pub = rospy.Publisher("car_position", Point, queue_size=10)
    velocity_pub = rospy.Publisher("car_velocity", Float32, queue_size=10)

    rate = rospy.Rate(100)  # 25 Hz loop frequency

    while not rospy.is_shutdown():
        current_time = time.time()
        elapsed_time = current_time - previous_time

        if elapsed_time >= 0.1:  # Update every 0.04 seconds
            previous_time = current_time

            # Calculate RPM and reset encoder value
            rpm = (encoder_value * 60*(1/elapsed_time)) / ENCODER_PPR
            encoder_value = 0

            # Linear velocity
            V_d = rpm * WHEEL_RADIUS /30

            # Velocity components
            dx = V_d * math.cos(theta)
            dy = V_d * math.sin(theta)

            # Total velocity magnitude
            total_velocity = V_d

            # Update position using Euler discretization
            x_position += dx * elapsed_time
            y_position += dy * elapsed_time

            # Create and publish position message
            position_msg = Point()
            position_msg.x = x_position
            position_msg.y = y_position
            position_pub.publish(position_msg)

            # Publish total velocity
            velocity_pub.publish(Float32(total_velocity))

            #rospy.loginfo(f"Position -> x: {x_position:.2f}, y: {y_position:.2f}")
            #rospy.loginfo(f"Total Velocity -> V: {total_velocity:.2f}")

        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()
