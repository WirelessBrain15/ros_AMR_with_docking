#! /usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
import math
import tf
from geometry_msgs.msg import PoseStamped
#from time import time

class chargingDock:
    flag2 = True
    trans = []
    rot = []
    def listener(self):
        dock_sub = rospy.Subscriber("/scan", LaserScan, chargingDock().callBack)
        rospy.spin()

    def cosineRule(self, a, b, angle):
        c2 = a*a + b*b - 2*a*b*math.cos(angle)
        return math.sqrt(c2)

    def callBack(self, data):
        range1 = []
        range2 = []
        index1 = []
        index2 = []
        flag = True
        coor1 = PoseStamped()
        coor2 = PoseStamped()
        tfListener = tf.TransformListener()
        cDarray = np.array(data.intensities, dtype=np.float32)  # Intensity
        rangeArray = np.array(data.ranges, dtype=np.float32)    # Ranges
        for index, value in enumerate(cDarray):
            if value > 0.5 and value < 1.5: # Intensity range
                if flag == True:        # Strip 1
                    range1.append(rangeArray[index])
                    index1.append(index)
                else:
                    range2.append(rangeArray[index])        # Strip 2
                    index2.append(index)
                if flag == True and cDarray[index+1] - cDarray[index] < 0:
                    flag = False

        # Calculating angles in radians # (data.angle_min +)
        angle1 = (sum(index1)/len(index1))*data.angle_increment   
        angle2 = (sum(index2)/len(index2))*data.angle_increment
        theta = angle2 - angle1 # angle btw ranges1 & 2 (radians)

        # Distance between strips
        avgRange1 = sum(range1)/len(range1)
        avgRange2 = sum(range2)/len(range2)
        dist = chargingDock().cosineRule(avgRange1, avgRange2, theta)

        # Coordinates of strips
        coor1.pose.position.x = avgRange1*math.cos(angle1)
        coor1.pose.position.y = avgRange1*math.sin(angle1)
        coor2.pose.position.x  = avgRange2*math.cos(angle2)
        coor2.pose.position.y  = avgRange2*math.sin(angle2)

        # Coordinate of dock
        while chargingDock.flag2 == True:
            try:
                (chargingDock.trans, chargingDock.rot) = tfListener.lookupTransform('/laser_link', '/base_link', rospy.Time(0))
                chargingDock.flag2 = False
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        print "trans : ", chargingDock.trans
        print "rot : ", chargingDock.rot
        
        print "dist btw : ", dist
        print "ranges : ", avgRange1, avgRange2
        #print range1, range2
        print "angles : ", math.degrees(angle1), math.degrees(angle2)
        print "x1, y1  : ", coor1.pose.position.x, coor1.pose.position.y 
        print "x2, y2  : ", coor2.pose.position.x, coor2.pose.position.y


def main():
    rospy.init_node('charging_dock')
    cD = chargingDock()
    
    while not rospy.is_shutdown():
        try:
            cD.listener()
        except KeyboardInterrupt:
            print "Shutting down"


if __name__ == '__main__':
    main()
