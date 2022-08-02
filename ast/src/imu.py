#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu






def callback(msg):
	global pub
	global Imu_data
	now = rospy.Time.now()
	Imu_data.header.stamp = now
	Imu_data.header.frame_id = "IMU_link"
	Imu_data.orientation.x = msg.orientation.x
	Imu_data.orientation.y = msg.orientation.y
	Imu_data.orientation.z = msg.orientation.z
	Imu_data.orientation.w = msg.orientation.w
	Imu_data.orientation_covariance[0] = -1.0
	Imu_data.angular_velocity.x = msg.angular_velocity.x
	Imu_data.angular_velocity.y = msg.angular_velocity.y
	Imu_data.angular_velocity.z = msg.angular_velocity.z
	Imu_data.angular_velocity_covariance[0] = 0.01
	Imu_data.angular_velocity_covariance[4] = 0.01
	Imu_data.angular_velocity_covariance[8] = 0.01
	Imu_data.linear_acceleration.x = msg.linear_acceleration.x
	Imu_data.linear_acceleration.y = msg.linear_acceleration.y
	Imu_data.linear_acceleration.z = msg.linear_acceleration.z
	Imu_data.linear_acceleration_covariance[0] = 0.01
	Imu_data.linear_acceleration_covariance[4] = 0.01
	Imu_data.linear_acceleration_covariance[8] = 0.01
	pub.publish(Imu_data)  









rospy.init_node("imu_converter")
pub = rospy.Publisher("imu",Imu,queue_size = 5)
sub = rospy.Subscriber("imu/data",Imu,callback)
Imu_data = Imu()
rospy.spin()
