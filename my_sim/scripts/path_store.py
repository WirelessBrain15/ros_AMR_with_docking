# Python code to store the current pose of bot to a .yaml file for path generation

#!/usr/bin/env python3

import yaml
import os
import string
from rospy_message_converter import message_converter
from pickle import NONE
import rospy
import time
from geometry_msgs.msg import PoseWithCovarianceStamped

save_flag = False
class path_gen():
    def __init__(self):
        # self.addr = '/home/bose/catkin_dock/src/path_tracking_pid-main/trajectories/path.yaml'
        self.addr = 'test_path.yaml'
        self.pose_sub = NONE
        

    def pose_listener(self):

        print("[LISTENER]")
        self.pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback, queue_size=1)
        
    def write_data(self, list):
        # with open(self.addr, 'w') as file:
        #     list = [str(x) for x in list]
        #     list = str(list)
        #     list = string.replace(list, "[", "")
        #     list = string.replace(list, "]", "")
        #     list = string.replace(list, "'", "")
        #     with open(self.addr, 'w') as f:
        #         f.write((list))
        with open(self.addr, 'w') as file:
            yaml.safe_dump(list, file, encoding='utf-8', default_flow_style=False, explicit_start=True)
            print("[SAVED]")

    def pose_callback(self, data):
        print("[CALLBACK]")
        pos = data.pose.pose.position
        ori = data.pose.pose.orientation

        if save_flag ==  True:
            dict = message_converter.convert_ros_message_to_dictionary(data)
            del dict['pose']['covariance']
            print(dict)
            self.write_data(dict)
        

        # print(list)
        # self.write_data(list)



def main():
    rospy.init_node('path_gen')
    str1 = path_gen()
    while not rospy.is_shutdown():
        str1.pose_listener()
        key = input('Enter save or exit (s/q): ')
        if key == 's':
            # Save pose in txt file
            # path_gen().write_data(dict)
            # list.append([pos.x,pos.y,pos.z,ori.x,ori.y,ori.z,ori.w])
            save_flag = True
        elif key == 'q':
            print("to exit")
            # path_gen().write_data(list)
            # return
            os._exit(0)
        else:
            print("invalid command")

        save_flag = False
        time.sleep(0.3)
        # rospy.spin()


if __name__ == '__main__':
    main()

