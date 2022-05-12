# Test code to detect an angled indentation in surrounding.

import rospy
import numpy as np
from math import cos, sin
import time
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2

def lidar_listener():
    print("listener check ... ")
    lidar_sub = rospy.Subscriber("/scan", LaserScan, lidar_callback, queue_size=1)
    rospy.spin()

def lidar_callback(data):
    print("callback check ... ")
    cloud = PointCloud2()
    cloud.header.frame_id = "laser"
    cloud.header.stamp = time.now()
    min_angle = data.angle_min
    max_angle = data.angle_max
    angle_inc = data.angle_increment
    range_array = np.array(data.ranges, dtype=np.float32)

    for index, range in enumerate(range_array):
        angle = min_angle + index* angle_inc
        cloud.point.x = range*cos(angle)
        cloud.point.y = range*sin(angle)

    print(cloud)


def main():
    rospy.init_node('test_icp')

    while not rospy.is_shutdown():
        try:
            lidar_listener()
        except KeyboardInterrupt:
            print("[SHUTTING DOWN] Keyboard interrupt detected . . . ")

if __name__ == '__main__':
    main()