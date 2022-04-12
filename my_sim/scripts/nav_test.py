#! /usr/bin/env python

import rospy
import numpy as np
from math import pow, atan2, sqrt, degrees, radians, cos, sin
import tf
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped, Point, Twist, Pose, PoseStamped
from visualization_msgs.msg import Marker
from sympy import symbols, Eq, solve
import actionlib
import thread
from threading import Thread, active_count
import time

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class chargingDock:
    
    def __init__(self):
        self.flag1 = False
        self.flag2 = False
        self.angle_mid = 0
        self.angle1 = 0
        self.angle2 = 0
        self.avgRange1 = 0
        self.avgRange2 = 0
        self.range_mid = 0
        self.new = PointStamped()
        self.pt1 = PointStamped()
        self.pt2 = PointStamped()
        self.slope = 0
        self.radius = 1
        self.sol = {}
        self.mark = []
        self.constant = 20
        # self.constant = 0.9
        self.rate = rospy.Rate(20)
        self.pose = PoseStamped()
        dock_sub = rospy.Subscriber("/scan", LaserScan, self.callBack, queue_size=1)
        pose_sub = rospy.Subscriber("/odom", Odometry, self.update_pose, queue_size=1)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    def cosineRule(self, a, b, angle):
        c2 = a*a + b*b - 2*a*b*cos(angle)
        return sqrt(c2)
    
    def update_pose(self, data):
        self.pose.header.frame_id = data.header.frame_id
        self.pose.pose = data.pose.pose

    def linear_vel(self):
        return 0.8*(self.range_mid)

    def angular_vel(self):
        return self.constant * (self.avgRange1 - self.avgRange2)
        # return -self.constant * (self.angle_mid - 3.14159)

    def distance(self, x1, y1, x2, y2):
        return sqrt(pow((x2 - x1),2) + pow((y2 - y1),2))

    def move_to_dock(self, radius): # Docking algo
        rospy.sleep(1)
        vel_msg = Twist()
        distance_tolerance = 0.0001      #ooga booga
        distance_tolerance2 = 0.001      #ooga booga
        print "docking"

        # while abs(self.angle_mid - 3.14159) > distance_tolerance2:
        #     vel_msg.linear.x = 0
        #     vel_msg.angular.x = 0
        #     vel_msg.angular.y = 0
        #     vel_msg.angular.z = 1 * (self.angle_mid - 3.14159)
        #     # Publishing our vel_msg
        #     self.velocity_publisher.publish(vel_msg)
        # # Stopping our robot after the movement is over.
        # vel_msg.linear.x = 0
        # vel_msg.angular.z = 0
        # self.velocity_publisher.publish(vel_msg)
        # print "angle_mid : ", degrees(self.angle_mid)

        while self.range_mid > radius:
            while abs(self.avgRange1 - self.avgRange2) >= distance_tolerance and abs(self.range_mid - 3.14159) > distance_tolerance2:
            # while abs(self.angle_mid - 3.14159) > distance_tolerance2 and abs(self.avgRange1 - self.avgRange2) > distance_tolerance:
                # Angular velocity in the z-axis.
                vel_msg.linear.x = 0
                
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = -self.angular_vel() + 0.5 * (self.angle_mid - 3.14159)
                # print "diff : ", abs(self.avgRange1 - self.avgRange2)
                # print "range : ", self.avgRange1, self.avgRange2
                print "range vel : ",vel_msg.angular.z
                print "angle vel : ",(self.angle_mid - 3.14159)
                # Publishing our vel_msg
                self.velocity_publisher.publish(vel_msg)

                self.rate.sleep()

            # Stopping our robot after the movement is over.
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)
            # print self.avgRange1, self.avgRange2
            print "angle_mid : ", degrees(self.angle_mid)
            print "turn done"
            self.constant = 60
            # self.constant = 1

            while abs(self.avgRange1 - self.avgRange2) < distance_tolerance and abs(self.angle_mid - 3.14159) <= distance_tolerance2:
            # while abs(self.angle_mid - 3.14159) <= distance_tolerance2 and abs(self.avgRange1 - self.avgRange2) <= distance_tolerance:
                # Linear velocity in the x-axis.
                vel_msg.linear.x = -self.linear_vel()
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0

                vel_msg.angular.z = 0

                # Publishing our vel_msg
                self.velocity_publisher.publish(vel_msg)

                self.rate.sleep()
            
            print "move done"
            # Stopping our robot after the movement is over.
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)
            continue

        print "diff : ", abs(self.avgRange1 - self.avgRange2)
        print "all done"
        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        return 0

    def move_base_client(self, pose):
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # moving bot
        goal.target_pose.pose.position.x = pose.position.x
        goal.target_pose.pose.position.y = pose.position.y 


        goal.target_pose.pose.orientation.x = pose.orientation.x
        goal.target_pose.pose.orientation.y = pose.orientation.y
        goal.target_pose.pose.orientation.z = pose.orientation.z
        goal.target_pose.pose.orientation.w = pose.orientation.w

        client.send_goal(goal)

        wait = client.wait_for_result()

        if not wait:
            print "Action server not available"
            rospy.signal_shutdown()
        else:
            return client.get_result()

    def rot_conversion(self, roll, pitch, yaw, pose, flag):
        if flag == True:    # Euler to quaternion
            quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
            pose = Pose()
            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]

            return pose
        else:   # Quaternion to euler
            quaternion = (pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            roll = euler[0]
            pitch = euler[1]
            yaw = euler[2]

            return roll, pitch, yaw

    def calculate_point(self, xm, ym, slope, radius):   # Solve eqs to calc destination
        x, y = symbols('x y')
        eq1 = Eq(((xm - x)/slope) + ym - y) # eq of line
        eq2 = Eq((x - xm)**2 + (y - ym)**2 - radius**2) # eq of circle
        sol = solve((eq1, eq2), (x,y))
        return sol

    def get_transform(self, mid):
        tfListener = tf.TransformListener()
        # while self.flag1 == True:
        if mid.header.frame_id != "":
            # print "spaghet" 
            tfListener.waitForTransform(mid.header.frame_id, 'map', rospy.Time(), rospy.Duration(0.5))
            try:
                now = rospy.Time.now()
                tfListener.waitForTransform(mid.header.frame_id, 'map', now, rospy.Duration(0.1))
                # print "carbonara"
                # (trans, rot) = tfListener.lookupTransform('map',mid.header.frame_id, now)
                # print trans, rot
                if tfListener.canTransform(mid.header.frame_id, 'map', now):
                    # print "linguini"
                    now = rospy.Time.now()
                    tfListener.waitForTransform(mid.header.frame_id, 'map', now, rospy.Duration(0.1))
                    mid.header.stamp = now
                    new = tfListener.transformPoint('map', mid)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                # continue
                pass
        return new

    def show_point_in_rviz(self, xm, ym, x1, y1):
        marker = Marker()
        pt = Point()
        pt.x = xm
        pt.y = ym
        pt.z = 0.4
        marker.points.append(pt)
        pt = Point()
        pt.x = x1
        pt.y = y1
        pt.z = 0.4
        marker.points.append(pt)
        # pt = Point()
        # pt.x = x2
        # pt.y = y2
        # pt.z = 0.5
        # marker.points.append(pt)
        marker.type = Marker.POINTS
        marker.id = 0
        marker.action = marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a= 0.8
        marker.lifetime = rospy.Duration(0)##

        # return marker
        self.marker_publisher.publish(marker)

    def move_to_dest(self):
        self.slope = (self.pt2.point.y - self.pt1.point.y)/(self.pt2.point.x - self.pt1.point.x)

        # destination to line up
        self.sol = self.calculate_point(self.new.point.x, self.new.point.y, self.slope, self.radius)

        if (self.distance(self.pose.pose.position.x, self.pose.pose.position.y, self.sol[0][0], self.sol[0][1]) < self.distance(self.pose.pose.position.x, self.pose.pose.position.y, self.sol[1][0], self.sol[1][1])):
            x = self.sol[0][0]
            y = self.sol[0][1]
            # print "cabbage"

        else:
            x = self.sol[1][0]
            y = self.sol[1][1]
            # print "celery"
        
        print "Destination : ", x, y
        self.show_point_in_rviz(self.new.point.x, self.new.point.y, x, y)
        temp_pose = Pose()

        # if self.angle_mid > 3.14159:
        #     self.angle_mid = 3.14159*2 - self.angle_mid

        # temp_pose = self.rot_conversion(0,0,self.angle_mid,0,True)

        temp_pose.position.x = x
        temp_pose.position.y = y
        temp_pose.orientation.z = 0
        temp_pose.orientation.w = 1
        result = self.move_base_client(temp_pose)
        if result:
            self.flag2 = True
            print "move done"
            return result

    # def error_check(self):
    #     vel_msg = Twist()
    #     if 
    #     vel_msg.angular.z = 0.5
        

    def steering(self, x, y):
        roll,pitch,yaw = self.rot_conversion(0,0,0,self.pose.pose,False) # Roll,pitch,yaw,pose,flag
        print "roll, pitch, yaw : ",degrees(roll), degrees(pitch), degrees(yaw)
        print "angle_mid : ",degrees(self.angle_mid)
        if degrees(self.angle_mid) > 180:
            self.angle_mid = self.angle_mid - 6.28

        yaw = self.angle_mid
        # print degrees(yaw)
        temp_pose = self.rot_conversion(roll,pitch,yaw,0,True)
        # print temp_pose
        temp_pose.position.x = x
        temp_pose.position.y = y
        result = self.move_base_client(temp_pose)
        if result:
            print "angle_mid : ", degrees(self.angle_mid)
            print "rotation done"
            self.flag3 = True
            return result

    def callBack(self, data):
        range1 = []
        range2 = []
        index1 = []
        index2 = []
        flag = True
        coor1 = PointStamped()
        coor2 = PointStamped()
        mid = PointStamped()
        temp_pose = Pose()
        # tfListener = tf.TransformListener()
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

        # mid point
        index_mid = (index1[0] + index2[-1])/2
        self.range_mid = rangeArray[index_mid]

        # Calculating angles in radians # (data.angle_min +)
        self.angle1 = (sum(index1)/len(index1))*data.angle_increment   
        self.angle2 = (sum(index2)/len(index2))*data.angle_increment
        self.angle_mid = (index_mid)*data.angle_increment
        theta = self.angle2 - self.angle1 # angle btw ranges1 & 2 (radians)
        # print "angle_mid : ", degrees(self.angle_mid)

        # Distance between strips
        self.avgRange1 = sum(range1)/len(range1)
        self.avgRange2 = sum(range2)/len(range2)
        dist = self.cosineRule(self.avgRange1, self.avgRange2, theta)

        # Coordinates of strips
        coor1.point.x = self.avgRange1*cos(self.angle1)
        coor1.point.y = self.avgRange1*sin(self.angle1)
        coor1.header.frame_id = '/laser_link'
        coor1.header.stamp = rospy.Time.now()

        coor2.point.x  = self.avgRange2*cos(self.angle2)
        coor2.point.y  = self.avgRange2*sin(self.angle2)
        coor2.header.frame_id = '/laser_link'
        coor2.header.stamp = rospy.Time.now()

        # Coordinate of dock
        mid.point.x = self.range_mid*cos(self.angle_mid)
        mid.point.y = self.range_mid*sin(self.angle_mid)
    
        mid.header.frame_id = '/laser_link'
        mid.header.stamp = rospy.Time.now()

        # if self.flag1 == False and self.angle_mid >= 177.5 and self.angle_mid <= 182.5:
        #     self.error_check()

        if self.flag1 == False:
            print "Starting"
            self.new = self.get_transform(mid)
            # self.mark.append(self.show_point_in_rviz(self.new.point.x, self.new.point.y))
            
            self.pt1 = self.get_transform(coor1)
            self.pt2 = self.get_transform(coor2)
            print "test mid : ", (self.pt1.point.x + self.pt2.point.x)/2,(self.pt1.point.y + self.pt2.point.y)/2
            # print "mid global x,y : ", self.new.point.x, self.new.point.y
            self.flag1 = True
            self.new.point.x = (self.pt1.point.x + self.pt2.point.x)/2
            self.new.point.y = (self.pt1.point.y + self.pt2.point.y)/2
            thread.start_new_thread(self.move_to_dest, ())
            # self.thread1.start()

        if self.flag2 == True and self.range_mid > 0.5:
            thread.start_new_thread(self.move_to_dock, (0.4,))  
            self.flag2 = False
            # self.flag1 = False

        

        # Print
        # print "Active thread count :", active_count()
        # print self.flag2
        # roll,pitch,yaw = self.rot_conversion(0,0,0,self.pose.pose,False)
        # print "yaw", degrees(yaw)
        # print rospy.Time.now() - mid.header.stamp
        # print "dist btw : ", dist 
        # print "coor1 x,y : ", coor1.point.x, coor1.point.y
        # print "coor2 x,y : ", coor2.point.x, coor2.point.y
        # print "ranges : ", self.avgRange1, self.avgRange2
        # print "range_mid : ", self.range_mid
        # print "angle_mid : ", degrees(self.angle_mid)
        # print "mid. x,y : ", mid.point.x, mid.point.y
        # print "mid global x,y : ", self.new.point.x, self.new.point.y
        # print "slope : ", self.slope
        # print "angles : ", degrees(self.angle1), degrees(self.angle2)
        # print "sep : ", degrees(self.angle2 - self.angle1)

def main():
    rospy.init_node('charging_dock')
    
    while not rospy.is_shutdown():
        try:
            cD = chargingDock()
            rospy.spin()    
        except KeyboardInterrupt:
            print "Shutting down"


if __name__ == '__main__':
    main()
