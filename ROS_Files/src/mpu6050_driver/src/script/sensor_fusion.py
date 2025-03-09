#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

class HeadingKalmanFilter:
    def __init__(self):
        # Kalman filter state and uncertainty
        self.KalmanState = 0.0  # Initial heading
        self.KalmanUncertainty = 1.0

        # Global variables for sensor data
        self.gyro_z = 0.0  # Gyroscope angular velocity (z-axis)
        self.mag_bearing = 0.0  # Magnetometer bearing angle

        # Subscriber for IMU (gyroscope data)
        rospy.Subscriber('/imu/data_raw', Imu, self.imu_callback)

        # Subscriber for magnetometer (bearing angle data)
        rospy.Subscriber('/bearing_angle', Float32, self.mag_callback)

        # Publisher for heading angle
        self.heading_pub = rospy.Publisher('/heading_angle', Float32, queue_size=10)

        # Time tracking for integration
        self.last_time = rospy.Time.now()

    def kalman_1d(self, KalmanState, KalmanUncertainty, KalmanInput, KalmanMeasurement):
        # Prediction step
        KalmanState += KalmanInput
        KalmanUncertainty += 0.04 * 0.04 * 4 * 4  # Adjust process noise as needed

        # Update step
        KalmanGain = KalmanUncertainty / (KalmanUncertainty + 2 * 2)  # Measurement noise
        KalmanState += KalmanGain * (KalmanMeasurement - KalmanState)
        KalmanUncertainty *= (1 - KalmanGain)

        return KalmanState, KalmanUncertainty

    def imu_callback(self, msg):
        # Extract angular velocity around the z-axis (yaw rate)
        self.gyro_z = msg.angular_velocity.z

        # Calculate time delta
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        # Predict heading based on gyro data
        self.KalmanState, self.KalmanUncertainty = self.kalman_1d(
            self.KalmanState,
            self.KalmanUncertainty,
            self.gyro_z * -1 * 0.04,  # KalmanInput
            self.mag_bearing  # Placeholder for measurement
        )

        if self.mag_bearing <= 4 or self.mag_bearing >= 356:
           self.KalmanState =self.mag_bearing

        self.heading_pub.publish(Float32(self.KalmanState))

    def mag_callback(self, msg):
        # Magnetometer provides the bearing angle
        self.mag_bearing = msg.data

        # Update Kalman filter with magnetometer measurement
       
        

        # Normalize heading to [0, 360) or [-180, 180] as needed
       

        # Publish fused heading angle
       

if __name__ == "__main__":
    rospy.init_node('sensor_fusion_node')
    rospy.loginfo("Heading Kalman Filter Node Started")
    heading_filter = HeadingKalmanFilter()
    rospy.spin()
