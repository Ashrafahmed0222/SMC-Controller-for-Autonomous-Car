#include <ros/ros.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/Float32.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>  // For atan2 function

// HMC5883L I2C Address
#define HMC5883L_ADDR 0x1E

// Registers
#define HMC5883L_CONFIG_A 0x00
#define HMC5883L_CONFIG_B 0x01
#define HMC5883L_MODE 0x02
#define HMC5883L_OUT_X_MSB 0x03

class HMC5883L {
public:
    HMC5883L(const std::string& device, ros::NodeHandle& nh) : nh_(nh) {
        // Open the I2C device
        fd_ = open(device.c_str(), O_RDWR);
        if (fd_ < 0) {
            ROS_ERROR("Failed to open I2C device");
            ros::shutdown();
        }

        // Set I2C address
        if (ioctl(fd_, I2C_SLAVE, HMC5883L_ADDR) < 0) {
            ROS_ERROR("Failed to set I2C address");
            ros::shutdown();
        }

        // Initialize the HMC5883L sensor
        initializeSensor();

        // Publisher for magnetic field data
        mag_pub_ = nh_.advertise<sensor_msgs::MagneticField>("magnetic_field", 10);
        
        // Publisher for yaw angle
        yaw_pub_ = nh_.advertise<std_msgs::Float32>("yaw_angle", 10);
    }

    void readData() {
        uint8_t buf[6];
        sensor_msgs::MagneticField mag_data;

        // Read 6 bytes of data (X, Y, Z magnetometer readings)
        if (read(fd_, buf, 6) != 6) {
            ROS_ERROR("Failed to read data from HMC5883L");
            return;
        }

        // Combine the data into 16-bit signed values (little-endian)
        mag_data.magnetic_field.x = (float)((buf[0] << 8) | buf[1]);
        mag_data.magnetic_field.y = (float)((buf[2] << 8) | buf[3]);
        mag_data.magnetic_field.z = (float)((buf[4] << 8) | buf[5]);

        // Publish the magnetic field data
        mag_data.header.stamp = ros::Time::now();
        mag_pub_.publish(mag_data);

        // Calculate the yaw angle (heading)
        float yaw = calculateYaw(mag_data.magnetic_field.x, mag_data.magnetic_field.y);

        // Publish the yaw angle
        std_msgs::Float32 yaw_msg;
        yaw_msg.data = yaw;
        yaw_pub_.publish(yaw_msg);
    }

    ~HMC5883L() {
        close(fd_);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher mag_pub_;
    ros::Publisher yaw_pub_;
    int fd_;

    void initializeSensor() {
        // Set the sensor to continuous mode
        uint8_t config[2];
        config[0] = 0x70;  // 8-average, 15Hz, normal measurement
        config[1] = 0xA0;  // Gain (5 gauss)

        if (write(fd_, config, 2) != 2) {
            ROS_ERROR("Failed to configure HMC5883L");
        }

        // Set the mode to continuous
        uint8_t mode = 0x00;  // Continuous measurement mode
        if (write(fd_, &mode, 1) != 1) {
            ROS_ERROR("Failed to set HMC5883L to continuous mode");
        }
    }

    // Function to calculate yaw angle in degrees
    float calculateYaw(float x, float y) {
        float yaw_rad = atan2(y, x);  // atan2 returns the angle in radians
        float yaw_deg = yaw_rad * 180.0 / M_PI;  // Convert radians to degrees
        return yaw_deg;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "hmc5883l_node");
    ros::NodeHandle nh;

    HMC5883L hmc5883l("/dev/i2c-1", nh);  // I2C bus on Raspberry Pi

    ros::Rate rate(10);  // 10 Hz
    while (ros::ok()) {
        hmc5883l.readData();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

