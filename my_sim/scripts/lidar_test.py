#! /usr/bin/env python

import rospy
import numpy as np
from math import degrees
from sensor_msgs.msg import LaserScan

class lidarTest:
    def __init__(self):
        self.angle1 = 0
        self.angle2 = 0
        sub = rospy.Subscriber('/r2000_node/scan', LaserScan, self.callBack, queue_size=1)

    def callBack(self, data):
        range1 = []
        range2 = []
        indexArray = []
        index1 = []
        index2 = []
        flag = True
        lidarArray = np.array(data.intensities, dtype=np.float32)
        rangeArray = np.array(data.ranges, dtype=np.float32)    # Ranges
        for index, value in enumerate(lidarArray):
            if value > 1000 and value < 1200: # Intensity range
                # if flag == True:        # Strip 1
                #     range1.append(rangeArray[index])
                #     index1.append(index)
                # else:
                #     range2.append(rangeArray[index])        # Strip 2
                #     index2.append(index)
                # if flag == True and (lidarArray[index+1] - lidarArray[index] < -200):
                #     flag = False
                # print lidarArray[index+1] - lidarArray[index]

                # alternate
                indexArray.append(index)
        index1.append(indexArray[0])
        for i in range(1,len(indexArray)):
            if indexArray[i-1]+1 == indexArray[i] and flag == True:
                range1.append(rangeArray[indexArray[i]])
                index1.append(indexArray[i])
            else:
                flag = False
                range2.append(rangeArray[indexArray[i]])
                index2.append(indexArray[i])


        # self.angle1 = (sum(index1)/len(index1))*data.angle_increment   
        # self.angle2 = (sum(index2)/len(index2))*data.angle_increment


        print "index1 : ", index1
        print "index2 : ", index2
        # print "angle1 : ", degrees(self.angle1)
        # print "angel2 : ", degrees(self.angle2)
        # print "range1 : ", range1
        # print "range2 : ", range2

def main():
    rospy.init_node('lidar_check')
    
    while not rospy.is_shutdown():
        try:
            ld = lidarTest()
            rospy.spin()    
        except KeyboardInterrupt:
            print "Shutting down"


if __name__ == '__main__':
    main()
