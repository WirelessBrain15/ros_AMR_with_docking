# Auto docking using r2000 lidar and 3 reflective strips
# -Aditya Rawat

import rospy
import numpy as np
from math import pow, atan2, sqrt, degrees, radians, cos, sin
import tf
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped, Point, Twist, Pose, PoseStamped
from visualization_msgs.msg import Marker
from sympy import symbols, Eq, solve
import actionlib
#import threading as thread
from threading import Thread, active_count
import time
from scipy.optimize import fsolve
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class docking_sequence:
    
    def __init__(self):
        self.flag1 = False  # Flag to initiate move to docking position
        self.flag2 = False  # Flag to initiate docking after move
        self.flag3 = False  # Flag to limit lidar range
        self.angle_mid = 0
        self.angle1 = 0
        self.angle2 = 0
        self.avgRange1 = 0
        self.avgRange2 = 0
        self.range_mid = 0
        self.new = PointStamped()   # Global coor of mid point
        self.pt1 = PointStamped()
        self.pt2 = PointStamped()
        self.slope = 0
        self.radius = 1.5   # Range for inital docking position
        self.sol = {}
        self.mark = []
        self.constant = 1.2     # Proportional constant for angular velocity
        self.rate = rospy.Rate(20)
        self.pose = PoseStamped()   # Current pose from odom

        # Publishers
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        self.rwheel_publisher = rospy.Publisher("/rwheel_vtarget", Float64, queue_size=1)
        self.lwheel_publisher = rospy.Publisher("/lwheel_vtarget", Float64, queue_size=1)

    def listener(self):
        # Subscribers for lidar and odom
        dock_sub = rospy.Subscriber("/r2000_node/scan", LaserScan, self.callBack, queue_size=1)
        pose_sub = rospy.Subscriber("/odom", Odometry, self.update_pose, queue_size=1)

        rospy.spin()

    def cosine_rule(self, a, b, angle):
        c2 = a*a + b*b - 2*a*b*cos(angle)
        return sqrt(c2)
    
    def update_pose(self, data):
        self.pose.header.frame_id = data.header.frame_id
        self.pose.pose = data.pose.pose

    def linear_vel(self):
        return 0.1*self.range_mid

    def angular_vel(self):
        return self.constant * (self.avgRange1 - self.avgRange2)

    def distance(self, x1, y1, x2, y2):
        return sqrt(pow((x2 - x1),2) + pow((y2 - y1),2))

    def move_to_dock(self, radius):
        rospy.sleep(1)      # Delay 1 second
        vel_msg = Twist()
        distance_tolerance = 0.0001     # distance error tolerance for docking
        distance_tolerance2 = 0.01      # Angle error tolerance for initial turning
        print("docking")
        print("angle mid : ", degrees(self.angle_mid))

        # Orienting the angle, so robot faces the right way
        while abs(self.angle_mid) > distance_tolerance2:
            vel_msg.linear.x = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0.4*(abs(self.angle_mid))

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        print("angle_mid : ", degrees(self.angle_mid))

        print("Phase 1 done")
        self.flag3 = True

        # Final docking
        while self.range_mid > radius:  # radius = 0.4
            rVel = -self.linear_vel()
            lVel = -self.linear_vel()

            self.rwheel_publisher.publish(rVel)
            self.lwheel_publisher.publish(lVel)

            # Path correction
            while self.range_mid > radius and abs(self.avgRange1 - self.avgRange2) >= distance_tolerance:
                rVel = -self.linear_vel() + 0.5*self.angular_vel()
                lVel = -self.linear_vel() - 0.5*self.angular_vel()
                # print(self.angular_vel(), self.linear_vel())
                # print("Vel : ", rVel, lVel)

                self.rwheel_publisher.publish(rVel)
                self.lwheel_publisher.publish(lVel)

        # Stopping
        rVel = 0
        lVel = 0

        self.rwheel_publisher.publish(rVel)
        self.lwheel_publisher.publish(lVel)

        print( "diff : ", (self.avgRange1 - self.avgRange2))
        print("angle mid : ", self.angle_mid)
        print("Test done")
    
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
        # print("Test")
        wait = client.wait_for_result()
        # print("Test2")
        if not wait:
            print( "Action server not available")
            rospy.signal_shutdown()
        else:
            return client.get_result()

    # def calculate_point(self, xm, ym, slope, radius):   # Solve eqs to calc destination
    #     x, y = symbols('x y')
    #     eq1 = Eq(((xm - x)/slope) + ym - y, 0) # eq of line
    #     eq2 = Eq((x - xm)**2 + (y - ym)**2 - radius**2, 0) # eq of circle
    #     sol = solve((eq1, eq2), (x,y))
    #     return sol

    def calculate_point(self, xm, ym, slope, radius):
        # zGuess = np.array([self.pose.pose.position.x,self.pose.pose.position.y])
        zGuess = np.array([0,0])
        sol = fsolve(self.my_func,zGuess)
        return sol

    def my_func(self, z):
        x = z[0]
        y = z[1]
        F = np.empty(2)
        F[0] = ((self.new.point.x - x)/self.slope) + self.new.point.y - y
        F[1] = (x - self.new.point.x)**2 + (y - self.new.point.y)**2 - self.radius**2
        return F 

    def get_transform(self, mid):
        # Transform from frame_id to 'map'
        tfListener = tf.TransformListener()
        if mid.header.frame_id != "":
            tfListener.waitForTransform(mid.header.frame_id, 'map', rospy.Time(), rospy.Duration(3))
            try:
                now = rospy.Time.now()
                tfListener.waitForTransform(mid.header.frame_id, 'map', now, rospy.Duration(1))
                if tfListener.canTransform(mid.header.frame_id, 'map', now):
                    now = rospy.Time.now()
                    tfListener.waitForTransform(mid.header.frame_id, 'map', now, rospy.Duration(1))
                    mid.header.stamp = now
                    new = tfListener.transformPoint('map', mid)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
        return new

    def show_point_in_rviz(self, xm, ym, x1, y1):
        # Show marker in Rviz
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
        # Move to inital destination
        self.slope = (self.pt2.point.y - self.pt1.point.y)/(self.pt2.point.x - self.pt1.point.x)

        # destination to line up
        self.sol = self.calculate_point(self.new.point.x, self.new.point.y, self.slope, self.radius)
        # print(self.sol)

        if (self.distance(self.pose.pose.position.x, self.pose.pose.position.y, self.sol[0][0], self.sol[0][1]) < self.distance(self.pose.pose.position.x, self.pose.pose.position.y, self.sol[1][0], self.sol[1][1])):
            x = self.sol[0][0]
            y = self.sol[0][1]
            # print( "cabbage"

        else:
            x = self.sol[1][0]
            y = self.sol[1][1]
            # print( "celery"
        
        print( "Destination : ", x, y)
        self.show_point_in_rviz(self.new.point.x, self.new.point.y, x, y)
        temp_pose = Pose()

        temp_pose.position.x = x
        temp_pose.position.y = y
        temp_pose.orientation.z = 0
        temp_pose.orientation.w = 1
        result = self.move_base_client(temp_pose)
        if result:
            self.flag2 = True
            print( "move done")
            return result

    def callBack(self, data):
        range1 = []
        range2 = []
        index1 = []
        index2 = []
        indexArray = []
        testArray = []
        flag = True
        coor1 = PointStamped()
        coor2 = PointStamped()
        mid = PointStamped()
        lidarArray = np.array(data.intensities, dtype=np.float32)   # Intensities
        rangeArray = np.array(data.ranges, dtype=np.float32)    # Ranges
        for index, value in enumerate(lidarArray):
            if self.flag3 == False and value > 750 and value < 1250: # Intensity range
                indexArray.append(index)

            if self.flag3 == True and value > 750 and value < 1250 and (index < 200 or index > 1600): # Intensity range
                indexArray.append(index)

        # Checking array for gaps
        for i in range(1,len(indexArray)):
            if indexArray[i] - indexArray[i-1] >= 10:
                testArray.append(i)
        
        # Splitting array from the gaps
        indexArray = np.split(indexArray,testArray)
        index1 = indexArray[0]
        index2 = indexArray[2]
        index3 = indexArray[1]

        range1 = rangeArray[index1]
        range2 = rangeArray[index2]
        range3 = rangeArray[index3]

        # Checking in lidar strip is getting split due to angle (0/360)
        if len(testArray) > 2:
            if testArray[0] < testArray[len(testArray) - 1]:    # Choose bigger portion of strip
                range3 = rangeArray[indexArray[len(testArray) - 1]]

        # Mid point
        self.angle_mid = (sum(index3)/len(index3))*data.angle_increment 
        self.range_mid = sum(range3)/len(range3)

        # Calculating angles in radians # (data.angle_min +)
        self.angle1 = (sum(index1)/len(index1))*data.angle_increment   
        self.angle2 = (sum(index2)/len(index2))*data.angle_increment
        theta = self.angle2 - self.angle1    # angle btw ranges1 & 2 (radians)

        # Distance between strips
        self.avgRange1 = sum(range1)/len(range1)
        self.avgRange2 = sum(range2)/len(range2)
        dist = self.cosine_rule(self.avgRange1, self.avgRange2, theta)

        # Angle correction for reverse docking, *only correcting mid angle
        if (self.angle2 - self.angle1) > 5.23599:
            if self.angle_mid < 3.14159:
                temp = self.angle1
                self.angle1 = self.angle_mid
                self.angle_mid = temp
                temp = self.avgRange1
                self.avgRange1 = self.range_mid
                self.range_mid = temp
            else:
                temp = self.angle2
                self.angle2 = self.angle_mid
                self.angle_mid = temp
                temp = self.avgRange2
                self.avgRange2 = self.range_mid
                self.range_mid = temp


        # Coordinates of side strips
        coor1.point.x = self.avgRange1*cos(self.angle1)
        coor1.point.y = self.avgRange1*sin(self.angle1)
        coor1.header.frame_id = 'laser_link'
        coor1.header.stamp = rospy.Time.now()

        coor2.point.x  = self.avgRange2*cos(self.angle2)
        coor2.point.y  = self.avgRange2*sin(self.angle2)
        coor2.header.frame_id = 'laser_link'
        coor2.header.stamp = rospy.Time.now()

        # Coordinate of dock (center strip)
        mid.point.x = self.range_mid*cos(data.angle_min + self.angle_mid)
        mid.point.y = self.range_mid*sin(data.angle_min + self.angle_mid)
    
        mid.header.frame_id = 'laser_link'
        mid.header.stamp = rospy.Time.now()

        if self.flag1 == False:
            print( "Starting")
            # Transform from 'laser_link' to 'map' frame
            self.new = self.get_transform(mid)
            self.pt1 = self.get_transform(coor1)
            self.pt2 = self.get_transform(coor2)
            self.flag1 = True
            # Threading the move function, so it runs simultaneously
            thread1 = Thread(target = self.move_to_dest(), name= 'thread1')
            thread1.start()

        if self.flag2 == True and self.range_mid > 0.4:
            # Threading the docking function
            thread2 = Thread(target=self.move_to_dock, args=(0.4,)) # 0.4 is the stopping range for the robot
            # Joining the threads to ensure docking is initiated after the move
            thread1.join()
            thread2.start()
            self.flag2 = False
        
        # Print statements

        # print( "Threading")
        # print( "Active thread count :", active_count())
        # print( self.flag2)
        # print( "yaw", degrees(yaw))
        # print( rospy.Time.now() - mid.header.stamp)
        # print( "dist btw : ", dist )
        # print( "coor1 x,y : ", coor1.point.x, coor1.point.y)
        # print( "coor2 x,y : ", coor2.point.x, coor2.point.y)
        # print( "ranges : ", self.constant*self.avgRange1, self.constant*self.avgRange2)
        # print( "range_mid : ", self.range_mid)
        # print( "angle_mid : ", degrees(self.angle_mid))
        # print( "mid. x,y : ", mid.point.x, mid.point.y)
        # print( "mid global x,y : ", self.new.point.x, self.new.point.y)
        # print( "slope : ", self.slope)
        # print( "angles : ", degrees(self.angle1), degrees(self.angle2))
        # print( "angle sep : ", degrees(self.angle2 - self.angle1)
        # print(testArray)
        # print(indexArray)

def main():
    rospy.init_node('charging_dock')
    
    while not rospy.is_shutdown():
        try:
            dS = docking_sequence()
            dS.listener()   
        except KeyboardInterrupt:
            print( "Shutting down")


if __name__ == '__main__':
    main()