
#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include <vector>
#include <iostream>
#include <fstream>
// #include <yaml-cpp/yaml.h>

using namespace std;

// Global variables
vector<geometry_msgs::PoseStamped> pose_array;
nav_msgs::Path path;
ofstream fio;
int seq = 0;

// Function declaration
void pose_to_path();
void write_data();

void write_data()
{
    pose_to_path();

    fio.open("/home/bose/catkin_dock/src/tracking_pid-master/trajectories/path.yaml", ios::trunc);

    fio << "header: " << endl;
    fio << "  seq: 0" << endl;
    fio << "  stamp: " << endl;
    fio << "    secs: 0" << endl;
    fio << "    nsecs:         0" << endl;
    fio << "  frame_id: 'map'" << endl;

    fio << "poses:" << endl;
    for(int i = 0; i < pose_array.size(); i++)
    {
        fio << "  -" << endl;
        fio << "    header: " << endl;
        fio << "      seq: " << i << endl;
        fio << "      stamp: " << endl;
        fio << "        secs: 0" << endl;
        fio << "        nsecs:         0" << endl;
        fio << "      frame_id: "<< pose_array[i].header.frame_id << endl;

        fio << "    pose:" << endl;
        fio << "      position: " << endl; 
        fio << "        x: " << pose_array[i].pose.position.x << endl;
        fio << "        y: " << pose_array[i].pose.position.y << endl;
        fio << "        z: " << pose_array[i].pose.position.z << endl;

        fio << "      orientation: " << endl; 
        fio << "        x: " << pose_array[i].pose.orientation.x << endl;
        fio << "        y: " << pose_array[i].pose.orientation.y << endl;
        fio << "        z: " << pose_array[i].pose.orientation.z << endl;
        fio << "        w: " << pose_array[i].pose.orientation.w << endl;
    }

    fio.close();
}

// NOT NEEDED ANYMORE**
void pose_to_path()
{
    nav_msgs::Path curr_path;
    curr_path.header.seq = 0;
    curr_path.header.stamp = ros::Time::now();
    curr_path.header.frame_id = "'map'";

    for(int i = 0; i < pose_array.size(); i++)
    {
        curr_path.poses.push_back(pose_array[i]);
    }
    path = curr_path;
}

void path_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    ROS_INFO("[CALLBACK]");
    geometry_msgs::PoseWithCovarianceStamped msg_pose;
    geometry_msgs::PoseStamped curr_pose;

    msg_pose = *msg;
    curr_pose.pose.position = msg_pose.pose.pose.position;
    curr_pose.pose.orientation = msg_pose.pose.pose.orientation;
    curr_pose.header.stamp = ros::Time::now();
    curr_pose.header.frame_id = "'map'";
    curr_pose.header.seq = seq;

    seq += 1;

    pose_array.push_back(curr_pose);

    cout<<curr_pose<<endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_create_bananas");
    ros::NodeHandle nh;
    ros::Rate r(20);

    ros::Subscriber pose_sub = nh.subscribe("/amcl_pose", 1, path_callback);
    
    while(1)
    {
        cout<<"Save or Exit (s/q) : " << endl;
        char key;
        cin >> key;

        if(key == 's')
        {
            ros::spinOnce();
        }
        else if(key == 'q')
        {
            // for(int i = 0; i < pose_array.size(); i++)
            // {
            //     cout << pose_array[i]<< endl;
            // }
            write_data();
            cout<<"Exiting"<<endl;
            return 0;
        }
        else
        {
            cout<<"Invalid keypress!"<<endl;
        }
        r.sleep();
    }
    
    
    // ros::spinOnce();
    ROS_INFO("[MAIN]");
    r.sleep();

    return 0;
}