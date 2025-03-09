#!/usr/bin/python

import rospy
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import smbus
import time
import math

# Initialize the ROS node
rospy.init_node('magnetometer_publisher', anonymous=True)

# Create a publisher for the bearing angle topic
bearing_pub = rospy.Publisher('bearing_angle', Float32, queue_size=10)

rev = GPIO.RPI_REVISION
if rev == 2 or rev == 3:
    bus = smbus.SMBus(1)
else:
    bus = smbus.SMBus(0)

address = 0x1e

def read_byte(adr):
    return bus.read_byte_data(address, adr)

def read_word(adr):
    high = bus.read_byte_data(address, adr)
    low = bus.read_byte_data(address, adr+1)
    val = (high << 8) + low
    return val

def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def write_byte(adr, value):
    bus.write_byte_data(address, adr, value)
    
if __name__ == "__main__":
    write_byte(0, 0b01110000)  # Set to 8 samples @ 15Hz
    write_byte(1, 0b00100000)  # 1.3 gain LSb / Gauss 1090 (default)
    write_byte(2, 0b00000000)  # Continuous sampling

    scale = 0.92
    x_offset = -876.0
    y_offset = 141.0
    
    # Loop to continuously read data and publish the bearing angle
    while not rospy.is_shutdown():
        x_out = (read_word_2c(3) - x_offset) * scale
        y_out = (read_word_2c(7) - y_offset) * scale
        z_out = (read_word_2c(5)) * scale

        bearing = math.atan2(y_out, x_out)
        if bearing < 0:
            bearing += 2 * math.pi

        bearing_angle = math.degrees(bearing)

        # Print the bearing angle
        #rospy.loginfo("Bearing: %f", bearing_angle)

        # Publish the bearing angle to the ROS topic
        bearing_pub.publish(bearing_angle)

        # Sleep to maintain loop rate (10 Hz)
        time.sleep(0.066667)

