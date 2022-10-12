#!/usr/bin/env python3
# Driver: SPDX-FileCopyrightText: 2020 Bryan Siepert, written for Adafruit Industries

import rospy
from sensor_msgs.msg import MagneticField,Imu
from std_msgs.msg import Float64
from diagnostic_msgs.msg import DiagnosticStatus
from ros_bno08x.srv import doCalibration, doCalibrationResponse
import time
import sys
import board
import busio
import adafruit_bno08x
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_GYRO_INTEGRATED_ROTATION_VECTOR,
    BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR
)
from adafruit_bno08x.i2c import BNO08X_I2C

class bno08x:
    
    def __init__(self):
        self.i2c = busio.I2C(board.SCL, board.SDA, frequency=800000)
        self.bno = BNO08X_I2C(self.i2c,address=0x4a) # BNO080 (0x4b) BNO085 (0x4a)
        self.bno08x_node_init()
        self.bno08x_node()

    def bno08x_node_init(self):
        # Initialize ROS node
        rospy.init_node('bno08x')
        # load covariance from parameter
        self.cov_linear = rospy.get_param('~cov_linear', -1)
        self.cov_angular = rospy.get_param('~cov_angular', -1)
        self.cov_orientation = rospy.get_param('~cov_orientation', -1)
        self.cov_magnetic = rospy.get_param('~cov_magnetic', -1)

        # load additional parameter
        self.frame_id = rospy.get_param('~frame_id', 'imu')
        self.rotation_vector = rospy.get_param('~rotation_vector', True)
        self.geomag_vector = rospy.get_param('~geomag_vector', True)
        
        # define publisher
        if self.rotation_vector == True:
            self.raw_pub = rospy.Publisher('bno08x/raw', Imu, queue_size=10)
        if self.geomag_vector == True:
            self.geo_pub = rospy.Publisher('bno08x/geo_raw', Imu, queue_size=10)

        self.mag_pub = rospy.Publisher('bno08x/mag', MagneticField, queue_size=10)
        self.status_pub = rospy.Publisher('bno08x/status', DiagnosticStatus, queue_size=10)

        
        self.rate = rospy.Rate(20) # frequency in Hz
        rospy.loginfo("bno08x node launched.")
        
        self.calib_status = 0

        self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
        self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
        if self.rotation_vector == True:
            self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        if self.geomag_vector == True:    
            self.bno.enable_feature(BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR)

        time.sleep(0.5) # ensure IMU is initialized

    def bno08x_node(self):
        while not rospy.is_shutdown():
      
            if self.rotation_vector == True:
                self.publishIMU(BNO_REPORT_ROTATION_VECTOR)

            if self.geomag_vector == True:
                self.publishIMU(BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR)

            mag_msg = MagneticField()
            mag_x, mag_y, mag_z = self.bno.magnetic
            mag_msg.header.stamp = rospy.Time.now()
            mag_msg.magnetic_field.x = mag_x
            mag_msg.magnetic_field.y = mag_y
            mag_msg.magnetic_field.z = mag_z

            mag_msg.magnetic_field_covariance[0] = -1
            if self.cov_magnetic != -1:
                mag_msg.magnetic_field_covariance[0] = self.cov_magnetic
                mag_msg.magnetic_field_covariance[4] = self.cov_magnetic
                mag_msg.magnetic_field_covariance[8] = self.cov_magnetic

            self.mag_pub.publish(mag_msg)
        
            self.calib_status = self.bno.calibration_status
            status_msg = DiagnosticStatus()
            status_msg.name = "bno08x IMU"

            status_msg.message ="Magnetometer Calibration quality:" + adafruit_bno08x.REPORT_ACCURACY_STATUS[self.calib_status]
            if self.calib_status == 0:
                status_msg.level = 1
            else:
                status_msg.level = 0

            self.status_pub.publish(status_msg)

            self.rate.sleep()   
    
        rospy.loginfo(rospy.get_caller_id() + "  bno08x node finished")

    def publishIMU(self, vector_type):
        raw_msg = Imu()
        raw_msg.header.stamp = rospy.Time.now()
        raw_msg.header.frame_id = self.frame_id

        accel_x, accel_y, accel_z = self.bno.acceleration
        raw_msg.linear_acceleration.x = accel_x
        raw_msg.linear_acceleration.y = accel_y
        raw_msg.linear_acceleration.z = accel_z

        gyro_x, gyro_y, gyro_z = self.bno.gyro
        raw_msg.angular_velocity.x = gyro_x
        raw_msg.angular_velocity.y = gyro_y
        raw_msg.angular_velocity.z = gyro_z

        raw_msg.orientation_covariance[0] = -1
        raw_msg.linear_acceleration_covariance[0] = -1
        raw_msg.angular_velocity_covariance[0] = -1

        if self.cov_orientation != -1:
            raw_msg.orientation_covariance[0] = self.cov_orientation
            raw_msg.orientation_covariance[4] = self.cov_orientation
            raw_msg.orientation_covariance[8] = self.cov_orientation


        if self.cov_linear != -1:
            raw_msg.linear_acceleration_covariance[0] = self.cov_linear
            raw_msg.linear_acceleration_covariance[4] = self.cov_linear
            raw_msg.linear_acceleration_covariance[8] = self.cov_linear

        if self.cov_angular != -1:
            raw_msg.angular_velocity_covariance[0] = self.cov_angular
            raw_msg.angular_velocity_covariance[4] = self.cov_angular
            raw_msg.angular_velocity_covariance[8] = self.cov_angular

        if vector_type == BNO_REPORT_ROTATION_VECTOR:
            quat_i, quat_j, quat_k, quat_real = self.bno.quaternion

        if vector_type == BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR:
            quat_i, quat_j, quat_k, quat_real = self.bno.geomagnetic_quaternion
        
        raw_msg.orientation.w = quat_real
        raw_msg.orientation.x = quat_i
        raw_msg.orientation.y = quat_j
        raw_msg.orientation.z = quat_k

        if vector_type == BNO_REPORT_ROTATION_VECTOR:
            self.raw_pub.publish(raw_msg)

        if vector_type == BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR:
            self.geo_pub.publish(raw_msg)



if __name__ == '__main__':
    try:
        bno08x()
    except rospy.ROSInterruptException:
        rospy.loginfo(rospy.get_caller_id() + "  bno08x node exited with exception.")
