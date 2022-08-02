# Test code to implement icp
#!/usr/bin/env python3

import numpy as np
import open3d as o3d

from math import cos, sin
from sensor_msgs.msg import LaserScan
from tf.transformations import quaternion_matrix, quaternion_from_matrix
from geometry_msgs.msg import Point, Pose, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
from scipy.spatial.transform import Rotation as R
import copy
import tf2_ros
import rospy
import os


class icp_2d():
    def __init__(self):
        self.is_calib = False
        self.lidar_x = 0
        self.lidar_y = 0
        self.home_pose = np.zeros(shape=(4,4))
        self.pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1) 


    def lidar_listener(self):
        lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback, queue_size=1)
        rospy.spin()


    def calibration(self):
        print("[CALIBRATION] Started . . . ")
        self.calib_flag = True
        self.lidar_listener()


    def read_calibration(self):
        self.calib_flag = False
        self.lidar_listener()


    def save_default_pcd(self, pcd):
        try:
            o3d.io.write_point_cloud("test_pcd.ply", pcd)
            transform = self.get_transform('laser_link')
            transform = np.array(transform)
            np.save('home_pose.npy', transform)
            print("[SAVED]")
        except Exception as e:
            print("[ERROR] ", e.args)


    def read_default_pcd(self):
        try:
            print("[READ]")
            target_pcd = o3d.io.read_point_cloud("test_pcd.ply")
            self.home_pose = np.load('home_pose.npy')
            return target_pcd
        except Exception as e:
            pass

    
    def pcd_conv(self, pcd_list):  
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pcd_list)
        pcd.paint_uniform_color([0.5,0.5,0.5])
        pcd.estimate_normals()
        print(pcd)

        return pcd


    def get_transform(self, frame_id):

        # print("[TRANSFORM]")
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        now = rospy.Time()
        while not rospy.is_shutdown():
            try:
                trans = tf_buffer.lookup_transform('map', frame_id, now)
                bot = tf_buffer.lookup_transform('base_link', frame_id, now)
            except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue

            matrix = quaternion_matrix([trans.transform.rotation.x,trans.transform.rotation.y,
                                        trans.transform.rotation.z,trans.transform.rotation.w])
            
            self.lidar_x = bot.transform.translation.x
            self.lidar_y = bot.transform.translation.y

            matrix[0][3] = trans.transform.translation.x
            matrix[1][3] = trans.transform.translation.y
            matrix[2][3] = trans.transform.translation.z
            # matrix[3][3] = 1
            # print(matrix)
            return matrix

    def change_reference(self, transform, transform_base, flag):

        if flag==True:
            # print(flag)
            current_pose = np.dot(transform, np.linalg.inv(transform_base))
        else:
            # print(flag)
            current_pose = np.dot(transform, transform_base)
        # print('banana')
        # print(np.linalg.inv(transform_base))
        return current_pose

    def draw_registration_result(self, source, target, transformation):

        source_temp = copy.deepcopy(source)
        target_temp = copy.deepcopy(target)
        source_temp.paint_uniform_color([1, 0.706, 0])
        target_temp.paint_uniform_color([0, 0.651, 0.929])
        source_temp.transform(transformation)
        o3d.visualization.draw_geometries([source_temp, target_temp],
                                            window_name='ICP', width=640,
                                            height=480)

    def icp(self, source, target, trans_init, flag):
        threshold = 10
        if flag == True:
            max_iter = 1000
            max_range = 15
        else:
            max_iter = 1
            max_range = 1
        # -----------------------------

        est_method = o3d.registration.TransformationEstimationPointToPoint()
        criteria = o3d.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
                                                    relative_rmse=1e-6,
                                                    max_iteration=max_iter)

        for i in range(max_range):
            reg_p2p = o3d.registration.registration_icp(source, target,
                                                    max_correspondence_distance=threshold,
                                                    init=trans_init,
                                                    estimation_method=est_method,
                                                    criteria=criteria)

            trans_init = reg_p2p.transformation
            threshold = threshold - 0.66
        
            print("iteration {0:02d} fitness {1:.6f} RMSE {2:.6f}".format(i, reg_p2p.fitness, reg_p2p.inlier_rmse))

        # ------------------------------
        return reg_p2p


    def icp_point_to_point(self, source, target):

        pose = PoseWithCovarianceStamped()
        count = 0
        rmse_list = []
        trans_init = self.get_transform('laser_link')
        
        trans_init = self.change_reference(trans_init, self.home_pose, True)
        trans_init[0][3] = 0
        trans_init[1][3] = 0
        trans_init[2][3] = 0

        # print(trans_init)
        # trans_init = np.identity(4)
        print("Initial transformation . . . ")
        print(trans_init)

        # ------------------------------

        reg_p2p = self.icp(source, target, trans_init, True)
        print(reg_p2p)
        print("[TRANSFORMATION]")
        print(reg_p2p.transformation)

        # self.draw_registration_result(source, target, reg_p2p.transformation)

        # # ------------------------------
        if reg_p2p.inlier_rmse > 0.1 or reg_p2p.fitness < 0.8:
        #     # trans_init = self.get_transform('laser_link')
            rotate = [[0,-1,0,0],[1,0,0,0],[0,0,1,0],[0,0,0,1]]

            trans_init = np.dot(reg_p2p.transformation, rotate)
            trans_init_test = trans_init

            while (count < 4):
                trans_init_test[0][3] = 0
                trans_init_test[1][3] = 0
                trans_init_test[2][3] = 0
                eval = self.icp(source, target, trans_init_test, False)
                rmse_list.append(eval.inlier_rmse)
                count += 1
                trans_init_test = np.dot(trans_init_test, rotate)
            
            print(rmse_list)
            min_index = rmse_list.index(min(rmse_list))
        #     print(rmse_list[min_index])
            for i in range(min_index):
                trans_init = np.dot(trans_init, rotate)

            reg_p2p = self.icp(source, target, trans_init, True)

        # # # ------------------------------
        
        # # trans_init = self.change_reference(trans_init, self.home_pose, True)
        # rotate = [[0,-1,0,0],[1,0,0,0],[0,0,1,0],[0,0,0,1]]
        # trans_init = np.dot(reg_p2p.transformation, rotate)
        # # trans_init = np.dot(reg_p2p.transformation, np.linalg.inv(trans_init))

        # while reg_p2p.fitness < 0.8 or reg_p2p.inlier_rmse > 0.1:

        #     trans_init[0][3] = 0
        #     trans_init[1][3] = 0
        #     trans_init[2][3] = 0
        
        #     reg_p2p = self.icp(source, target, trans_init, True)
        #     print(reg_p2p)

        #     trans_init = np.dot(reg_p2p.transformation, rotate)

        #     self.draw_registration_result(source, target, reg_p2p.transformation)

        # # # ------------------------------

        current_pose = self.change_reference(self.home_pose, reg_p2p.transformation, False)
        print(reg_p2p)
        print("[TRANSFORMATION]")
        print(current_pose)
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x = current_pose[0][3]
        pose.pose.pose.position.y = current_pose[1][3]

        quaternion = quaternion_from_matrix(current_pose)

        pose.pose.pose.orientation.x = quaternion[0]
        pose.pose.pose.orientation.y = quaternion[1]
        pose.pose.pose.orientation.z = quaternion[2]
        pose.pose.pose.orientation.w = quaternion[3] 

        self.draw_registration_result(source, target, reg_p2p.transformation)

        for i in range(50):
            self.pose_pub.publish(pose)


    def lidar_callback(self, data):

        print("[CALLBACK]")
        angle_min = data.angle_min
        angle_max = data.angle_max
        angle_inc = data.angle_increment
        range_array = np.array(data.ranges, dtype=np.float32)
        point_list = []

        for index, range in enumerate(range_array):
            angle = angle_min + index*angle_inc
            x = range*cos(angle)
            y = range*sin(angle)
            point_list.append([x, y, 0])

        # List to numpy array
        point_array = np.array(point_list)
        # print(point_array)

        # Numpy array to pointcloud format for open3d
        pcd = self.pcd_conv(point_array)
        # --------------------------------
        # Calibration, so spin once
        if self.calib_flag == True:
            self.save_default_pcd(pcd)
            self.calib_flag = False
            print("[CALIBRATION] Done . . . ")
            
            is_continue = raw_input("Continue ? (y/n) : ")
            if(is_continue.lower() != "y"):
                print("[ENDING]")
                os._exit(0)

        else:
            target_pcd = self.read_default_pcd()
            print("[CALIBRATION] Read . . . ")
            # print(target_pcd)
            self.icp_point_to_point(pcd, target_pcd)
            os._exit(0)
        # --------------------------------



def main():
    rospy.init_node('icp_2d')
    icp = icp_2d()
    is_calib = raw_input("Calibrate ? (y/n) : ")
    while not rospy.is_shutdown():
        try:
            if(is_calib.lower() == "y"): 
                icp.calibration()

            icp.read_calibration()
            
        except KeyboardInterrupt:
            print("[SHUTTING DOWN] Keyboard interrupt detected . . . ")


if __name__ == '__main__':
    main()