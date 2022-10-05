#!/usr/bin/env python3
# Driver: SPDX-FileCopyrightText: 2020 Bryan Siepert, written for Adafruit Industries

import rospy
from sensor_msgs.msg import MagneticField,Imu
from std_msgs.msg import Float64
from diagnostic_msgs.msg import DiagnosticStatus
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

def bno08x_node():

    # Initialize ROS node
    raw_pub = rospy.Publisher('bno08x/raw', Imu, queue_size=10)
    geo_pub = rospy.Publisher('bno08x/geo_raw', Imu, queue_size=10)
    mag_pub = rospy.Publisher('bno08x/mag', MagneticField, queue_size=10)
    status_pub = rospy.Publisher('bno08x/status', DiagnosticStatus, queue_size=10)
    rospy.init_node('bno08x')
    rate = rospy.Rate(100) # frequency in Hz
    rospy.loginfo("bno08x node launched.")
    frame_id = rospy.get_param('~frame_id', 'imu')
    # load covariance from parameter
    cov_linear = rospy.get_param('~cov_linear', -1)
    cov_angular = rospy.get_param('~cov_angular', -1)
    cov_orientation = rospy.get_param('~cov_orientation', -1)
    cov_magnetic = rospy.get_param('~cov_magnetic', -1)

    i2c = busio.I2C(board.SCL, board.SDA, frequency=800000)
    bno = BNO08X_I2C(i2c,address=0x4a) # BNO080 (0x4b) BNO085 (0x4a)

    #bno.begin_calibration()
    time.sleep(0.5) # ensure IMU is initialized
    bno.enable_feature(BNO_REPORT_ACCELEROMETER)
    time.sleep(0.5) # ensure IMU is initialized
    bno.enable_feature(BNO_REPORT_GYROSCOPE)
    time.sleep(0.5) # ensure IMU is initialized
    bno.enable_feature(BNO_REPORT_MAGNETOMETER)
    time.sleep(0.5) # ensure IMU is initialized
    bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
    time.sleep(0.5) # ensure IMU is initialized
    bno.enable_feature(BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR)

    time.sleep(0.5) # ensure IMU is initialized

    while not rospy.is_shutdown():
       # if bno.calibration_status == 0:
       #     rospy.loginfo("bno08x begin calibration")
       #     bno.begin_calibration
            
       #     while bno.calibration_status == 0:
       #         rospy.loginfo("bno08x calibration failed, retry") 
       #         time.sleep(1.0)
       
        raw_msg = Imu()
        geo_msg = Imu()
        geo_msg.header.stamp = raw_msg.header.stamp = rospy.Time.now()
        geo_msg.header.frame_id = raw_msg.header.frame_id = frame_id

        accel_x, accel_y, accel_z = bno.acceleration
        geo_msg.linear_acceleration.x = raw_msg.linear_acceleration.x = accel_x
        geo_msg.linear_acceleration.y = raw_msg.linear_acceleration.y = accel_y
        geo_msg.linear_acceleration.z = raw_msg.linear_acceleration.z = accel_z

        gyro_x, gyro_y, gyro_z = bno.gyro
        geo_msg.angular_velocity.x = raw_msg.angular_velocity.x = gyro_x
        geo_msg.angular_velocity.y = raw_msg.angular_velocity.y = gyro_y
        geo_msg.angular_velocity.z =raw_msg.angular_velocity.z = gyro_z

        raw_msg.orientation_covariance[0] = -1
        raw_msg.linear_acceleration_covariance[0] = -1
        raw_msg.angular_velocity_covariance[0] = -1

        geo_msg.orientation_covariance[0] = -1
        geo_msg.linear_acceleration_covariance[0] = -1
        geo_msg.angular_velocity_covariance[0] = -1

        if cov_orientation != -1:
            geo_msg.orientation_covariance[0] = raw_msg.orientation_covariance[0] = cov_orientation
            geo_msg.orientation_covariance[4] =raw_msg.orientation_covariance[4] = cov_orientation
            geo_msg.orientation_covariance[8] =raw_msg.orientation_covariance[8] = cov_orientation


        if cov_linear != -1:
            geo_msg.linear_acceleration_covariance[0] = raw_msg.linear_acceleration_covariance[0] = cov_linear
            geo_msg.linear_acceleration_covariance[4] =raw_msg.linear_acceleration_covariance[4] = cov_linear
            geo_msg.linear_acceleration_covariance[8] =raw_msg.linear_acceleration_covariance[8] = cov_linear

        if cov_angular != -1:
            geo_msg.angular_velocity_covariance[0] = raw_msg.angular_velocity_covariance[0] = cov_angular
            geo_msg.angular_velocity_covariance[4] =raw_msg.angular_velocity_covariance[4] = cov_angular
            geo_msg.angular_velocity_covariance[8] =raw_msg.angular_velocity_covariance[8] = cov_angular

        #geo_msg = raw_msg
        
        geo_quat_i, geo_quat_j, geo_quat_k, geo_quat_real = bno.geomagnetic_quaternion
        geo_msg.orientation.w = geo_quat_real
        geo_msg.orientation.x = geo_quat_i
        geo_msg.orientation.y = geo_quat_j
        geo_msg.orientation.z = geo_quat_k

        quat_i, quat_j, quat_k, quat_real = bno.quaternion
        raw_msg.orientation.w = quat_real
        raw_msg.orientation.x = quat_i
        raw_msg.orientation.y = quat_j
        raw_msg.orientation.z = quat_k

        geo_pub.publish(geo_msg)
        raw_pub.publish(raw_msg)


        mag_msg = MagneticField()
        mag_x, mag_y, mag_z = bno.magnetic
        mag_msg.header.stamp = rospy.Time.now()
        mag_msg.magnetic_field.x = mag_x
        mag_msg.magnetic_field.y = mag_y
        mag_msg.magnetic_field.z = mag_z

        mag_msg.magnetic_field_covariance[0] = -1
        if cov_magnetic != -1:
            mag_msg.magnetic_field_covariance[0] = cov_magnetic
            mag_msg.magnetic_field_covariance[4] = cov_magnetic
            mag_msg.magnetic_field_covariance[8] = cov_magnetic

        mag_pub.publish(mag_msg)
        
        calib_status = bno.calibration_status
        status_msg = DiagnosticStatus()
        status_msg.name = "bno08x IMU"

        if calib_status == 0:
            status_msg.level = 1
            status_msg.message ="Magnetometer Calibration quality:" + adafruit_bno08x.REPORT_ACCURACY_STATUS[calib_status]
        else:
            status_msg.message ="calibrated: " + calib_status    
            status_msg.level = 0

        status_pub.publish(status_msg)

        rate.sleep()   
    
    rospy.loginfo(rospy.get_caller_id() + "  bno08x node finished")

if __name__ == '__main__':
    try:
        bno08x_node()
    except rospy.ROSInterruptException:
        rospy.loginfo(rospy.get_caller_id() + "  bno08x node exited with exception.")
