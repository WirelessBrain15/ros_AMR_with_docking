# Test code to implement icp
#!/usr/bin/env python3
import os
import rospy
import numpy as np
from math import cos, sin
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Pose, PoseWithCovarianceStamped
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import Marker
# from laser_geometry import LaserProjection
import laser_geometry.laser_geometry as lg
import open3d as o3d
import copy
import tf2_ros
from tf.transformations import quaternion_matrix, quaternion_from_matrix


class lidar_to_pc():
    def __init__(self):
        print("[STARTING]")
        self.laser_proj = lg.LaserProjection()
        self.flag = False
        self.calib_flag = False
        self.target = []
        self.pose = Pose()
        self.x = 0
        self.y = 0
        self.home_pose = np.zeros(shape=(4,4))

        self.lidar_sub = None
        self.pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback, queue_size=1)
        self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        # self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback, queue_size=1)
        self.pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1) 
    
    def lidar_listener(self):
        print("[LISTENER] ")
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback, queue_size=1)
        rospy.spin()

    
    def pose_callback(self, data):
        self.pose.position.x = data.pose.pose.position.x
        self.pose.position.y = data.pose.pose.position.y


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


    def show_point_in_rviz(self, xm, ym):
        # Show marker in Rviz
        marker = Marker()
        pt = Point()
        pt.x = xm
        pt.y = ym
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


    def pcd_conv(self, pcd_list):  
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pcd_list)
        pcd.paint_uniform_color([0.5,0.5,0.5])
        pcd.estimate_normals()
        print(pcd)

        return pcd

    def get_transform(self, frame_id):
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        now = rospy.Time()
        while not rospy.is_shutdown():
            try:
                trans = tf_buffer.lookup_transform('map', frame_id, now)
                bot = tf_buffer.lookup_transform('base_link',frame_id,now)
            except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue

            matrix = quaternion_matrix([trans.transform.rotation.x,trans.transform.rotation.y,
                                        trans.transform.rotation.z,trans.transform.rotation.w])
            # print("quat_to_mat")
            # print(matrix)
            # matrix = o3d.geometry.get_rotation_matrix_from_quaternion([trans.transform.rotation.x,trans.transform.rotation.y,
            #                             trans.transform.rotation.z,trans.transform.rotation.w])
            # print("o3d_rot")
            
            # matrix[0][0] = matrix[1][1]
            # matrix[0][1] = matrix[1][2]
            # matrix[0][2] = 0
            # matrix[1][0] = matrix[2][1]
            # matrix[1][1] = matrix[2][2]
            # matrix[1][2] = 0
            # matrix[2][0] = 0
            # matrix[2][1] = 0
            # matrix[2][2] = 1
            # print(matrix)
            self.x = bot.transform.translation.x
            self.y = bot.transform.translation.y
            
            
            # matrix = np.append(matrix,[[trans.transform.translation.x],[trans.transform.translation.y],
            #                     [trans.transform.translation.z]], axis=1)
            # matrix = np.vstack((matrix, [0,0,0,1]))

            matrix[0][1] = -matrix[0][1]
            matrix[1][0] = -matrix[1][0]
            
            matrix[0][3] = trans.transform.translation.x
            matrix[1][3] = trans.transform.translation.y
            # matrix[0][3] = self.pose.position.x
            # matrix[1][3] = self.pose.position.y
            matrix[2][3] = trans.transform.translation.z
            matrix[3][3] = 1
            return matrix

    def draw_registration_result(self, source, target, transformation):
        source_temp = copy.deepcopy(source)
        target_temp = copy.deepcopy(target)
        source_temp.paint_uniform_color([1, 0.706, 0])
        target_temp.paint_uniform_color([0, 0.651, 0.929])
        source_temp.transform(transformation)
        o3d.visualization.draw_geometries([source_temp, target_temp])

    def icp_point_to_plane(self, source, target):
        pose = PoseWithCovarianceStamped()
        threshold = 100
        trans_init = self.get_transform('laser_link')

        trans_init = np.dot(trans_init, np.linalg.inv(self.home_pose))
        # print(np.dot(self.home_pose, np.linalg.inv(self.home_pose)))
        # trans_init = np.divide(trans_init, self.home_pose)

        print("Initial transformation . . . ")
        print(trans_init)

        # self.draw_registration_result(source, target, trans_init)

        # -----------------------------

        est_method = o3d.registration.TransformationEstimationPointToPoint()
        criteria = o3d.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
                                                    relative_rmse=1e-6,
                                                    max_iteration=1000)

        for i in range(10):
            reg_p21 = o3d.registration.registration_icp(source, target,
                                                    max_correspondence_distance=threshold,
                                                    init=trans_init,
                                                    estimation_method=est_method,
                                                    criteria=criteria)

            trans_init = reg_p21.transformation
            threshold = threshold - 11.1
        
            print("iteration {0:02d} fitness {1:.6f} RMSE {2:.6f}".format(i, reg_p21.fitness, reg_p21.inlier_rmse))

        # ------------------------------

        print(reg_p21)
        print("[TRANSFORMATION]")
        print(reg_p21.transformation)
        # self.draw_registration_result(source, target, reg_p21.transformation)

        pose.header.frame_id = 'map'

        pose.pose.pose.position.x = -reg_p21.transformation[0][3] + self.x
        pose.pose.pose.position.y = -reg_p21.transformation[1][3] + self.y

        # reg_p21.transformation[0][3] = 0
        # reg_p21.transformation[1][3] = 0
        # reg_p21.transformation[2][3] = 1
        matrix = [[0]*4 for i in range(4)]
        matrix[0][0] = reg_p21.transformation[0][0]
        matrix[0][1] = reg_p21.transformation[0][1]
        matrix[0][2] = reg_p21.transformation[0][2]
        matrix[0][3] = pose.pose.pose.position.x
        matrix[1][0] = reg_p21.transformation[1][0]
        matrix[1][1] = reg_p21.transformation[1][1]
        matrix[1][2] = reg_p21.transformation[1][2]
        matrix[1][3] = pose.pose.pose.position.y
        matrix[2][0] = reg_p21.transformation[2][0]
        matrix[2][1] = reg_p21.transformation[2][1]
        matrix[2][2] = reg_p21.transformation[2][2]
        matrix[3][3] = 1

        print("BEFORE")
        print(self.home_pose)
        new_array = np.array(matrix)
        new_array = np.dot(new_array, self.home_pose)

        matrix = [[0]*4 for i in range(4)]
        matrix[0][0] = new_array[0][0]
        matrix[0][1] = new_array[0][1]
        matrix[0][2] = new_array[0][2]
        # matrix[0][3] = pose.pose.pose.position.x
        matrix[1][0] = new_array[1][0]
        matrix[1][1] = new_array[1][1]
        matrix[1][2] = new_array[1][2]
        # matrix[1][3] = pose.pose.pose.position.y
        matrix[2][0] = new_array[2][0]
        matrix[2][1] = new_array[2][1]
        matrix[2][2] = new_array[2][2]
        matrix[3][3] = 1
        print("AFTER")
        print(matrix)

        # quaternion = quaternion_from_matrix(reg_p21.transformation)
        quaternion = quaternion_from_matrix(matrix)

        pose.pose.pose.orientation.x = quaternion[0]
        pose.pose.pose.orientation.y = quaternion[1]
        pose.pose.pose.orientation.z = quaternion[2]
        pose.pose.pose.orientation.w = quaternion[3] 

        # dest_x = self.pose.position.x + pose.position.x
        # dest_y = self.pose.position.y + pose.position.y
        
        
        dest_x = new_array[0][3]
        dest_y = new_array[1][3]
        pose.pose.pose.position.x = dest_x
        pose.pose.pose.position.y = dest_y

        print("Destination : ", dest_x, dest_y)  
        # print("static diff : ", self.x, self.y)
        self.show_point_in_rviz(dest_x, dest_y) 

        for i in range(50):
            self.pose_pub.publish(pose)

    def lidar_callback(self, data):
        print("[CALLBACK]")

        cloud = self.laser_proj.projectLaser(data)
        point_generator = pc2.read_points_list(cloud)
        pcd = o3d.geometry.PointCloud()
        target_pcd = o3d.geometry.PointCloud()

        point = Point()
        angle_min = data.angle_min
        angle_max = data.angle_max
        angle_inc = data.angle_increment
        point_list = []

        range_array = np.array(data.ranges, dtype=np.float32)
        for index, range in enumerate(range_array):
            angle = index*angle_inc
            # if angle > -0.523599/2 and angle < 0.523599/2:
            x = range*cos(angle)
            y = range*sin(angle)
            point_list.append((x,y,0.0))
            
        test_list = np.array(point_list)

        pcd = self.pcd_conv(test_list)
        # --------------------------------
        # Calibration, so spin once
        if self.calib_flag == True:
            self.save_default_pcd(pcd)
            self.calib_flag = False
            print("[CALIBRATION] Done . . . ")
            target_pcd = pcd
            
            is_continue = raw_input("Continue ? (y/n) : ")
            if(is_continue.lower() != "y"):
                print("[ENDING]")
                self.lidar_sub.unregister()
                os._exit(0)

        else:
            target_pcd = self.read_default_pcd()
            print("[CALIBRATION] Read . . . ")
            self.icp_point_to_plane(pcd, target_pcd)
            os._exit(0)
        # --------------------------------


def main():
    rospy.init_node('test_icp')
    l2pc = lidar_to_pc()
    is_calib = raw_input("Calibrate ? (y/n) : ")
    while not rospy.is_shutdown():
        try:
            if(is_calib.lower() == "y"): 
                l2pc.calibration()

            l2pc.read_calibration()
            
        except KeyboardInterrupt:
            print("[SHUTTING DOWN] Keyboard interrupt detected . . . ")

if __name__ == '__main__':
    main()