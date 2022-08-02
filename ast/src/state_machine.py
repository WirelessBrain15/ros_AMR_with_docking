#!/usr/bin/env python

import rospy
import smach
import smach_ros
from smach_ros import SimpleActionState
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from mbf_msgs.msg import ExePathAction
from mbf_msgs.msg import GetPathAction
from mbf_msgs.msg import RecoveryAction


def main():

	rospy.init_node('mbf_state')
	
	sm = smach.StateMachine(outcomes = ['succeeded','aborted','preempted'])
	sm.userdata.goal = None
	sm.userdata.path = None
	sm.userdata.error = None
	sm.userdata.clear_costmap_flag = None
	sm.userdata.error_status = None


	with sm:
		
		def goal_cb(userdata,msg):
			userdata.goal = msg
			rospy.loginfo(msg)

			return False
		
		smach.StateMachine.add('WAIT_FOR_GOAL',smach_ros.MonitorState('/move_base_simple/goal',PoseStamped,goal_cb,output_keys = ['goal']),transitions = {'invalid':'GET_PATH','valid':'WAIT_FOR_GOAL','preempted':'preempted'})


		smach.StateMachine.add('GET_PATH',SimpleActionState('/move_base_flex/get_path',GetPathAction,goal_slots=['target_pose'],result_slots = ['path']),transitions = {'succeeded': 'EXE_PATH','aborted':'WAIT_FOR_GOAL','preempted':'preempted'},remapping = {'target_pose':'goal'})

		smach.StateMachine.add('EXE_PATH',SimpleActionState('/move_base_flex/exe_path',ExePathAction,goal_slots =['path']),transitions = {'succeeded':'succeeded' , 'aborted' : 'RECOVERY','preempted':'preempted'})

		def recovery_path(userdata,goal):
			if userdata.clear_costmap_flag == False:
				goal.behavior = 'clear_costmap'
				userdata.clear_costmap_flag = True
			else:
				goal.behavior = 'rotate_recovery'
				userdata.clear_costmap_flag = False

		smach.StateMachine.add('RECOVERY',SimpleActionState('/move_base_flex/recovery',RecoveryAction,goal_cb = recovery_path,input_keys = ["error","clearmap_flag"],output_keys = ["error_status","clear_costmap_flag"]),transitions = {'succeeded' : 'GET_PATH','aborted' : 'aborted' ,'preempted':'preempted'})


	sis = smach_ros.IntrospectionServer('smach_server',sm,'/SM_ROOT')
	sis.start()
	sm.execute()

	rospy.spin()

	sis.stop()


if __name__ =='__main__':
	main()















