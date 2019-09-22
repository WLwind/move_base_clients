#!/usr/bin/env python
#coding:utf-8
from __future__ import print_function
import os
import roslib
import rospy
import smach
import smach_ros

import move_base_msgs.msg
import geometry_msgs.msg
import tf
from actionlib import *
from actionlib_msgs.msg import *
import std_srvs.srv

class InputOption(smach.State):
    """option menu"""
    def __init__(self):
        super(InputOption,self).__init__(outcomes=['go','go_out','coordinate'],output_keys=['input_numb'])
    
    def execute(self,userdata):
        dnum=raw_input('Please select the options:\n0.Go to position0;\n1.Go to position1;\n2.Go to position2;\n3.Move to coordinate;\n4.Exit\n>')
        if dnum=='4':
            print('Exit now.')
            return 'go_out'
        elif dnum=='3':
            print('->(x,y)')
            return 'coordinate'
        else:
            userdata.input_numb=int(dnum)
            print("\nLet's go")
            return 'go'

class MoveToCoordinate(smach.State):
    """run the C++ move_base client"""
    def __init__(self):
        super(MoveToCoordinate,self).__init__(outcomes=['succeeded','aborted','preempted'])
    
    def execute(self,userdata):
        coor_point=raw_input('Plase inpute the coordinate:x y theta\n')
        command='rosrun move_base_clients simple_client '+coor_point
        int_result=os.system(command)
        if int_result==0:
            return 'succeeded'
        elif int_result==1:
            return 'aborted'
        elif int_result==2:
            return 'preempted'
        else:
            return 'preempted'

def main():
    rospy.init_node('smach_example_actionlib')

    # Create a SMACH state machine
    sm0 = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
    # Set a list of goals
    sm0.userdata.goal_list=[[0.0,0.0,0.0],[-0.165,-0.69,0.9],[2.65,-1.34,2.2]] #coordinates of positions
    sm0.userdata.goal_index=0

    with sm0:
        def move_goal_cb(userdata,goal):
            move_GO=move_base_msgs.msg.MoveBaseGoal()
            move_GO.target_pose.header.frame_id='map'
            move_GO.target_pose.header.stamp=rospy.Time.now()
            move_GO.target_pose.pose.position.x=userdata.list[userdata.index][0]
            move_GO.target_pose.pose.position.y=userdata.list[userdata.index][1]
            move_GO.target_pose.pose.position.z=0.0
            q_angle = tf.transformations.quaternion_from_euler(0.0,0.0,userdata.list[userdata.index][2])
            q = geometry_msgs.msg.Quaternion(*q_angle)
            move_GO.target_pose.pose.orientation=q
            print('Going to ',userdata.list[userdata.index])
            return move_GO

        def move_result_cb(userdata, status, result):
            status_list=['pending','active','preempted','succeeded','aborted','rejected','preempting','recalling','recalled','lost']
            print('Status is ',status_list[status])
            print('Index is ',userdata.index)
            return status_list[status]

        smach.StateMachine.add('OPTION',
                               InputOption(),
                               transitions={'go':'MOVE_BASE','go_out':'preempted','coordinate':'GO_TO_COORDINATE'},
                               remapping={'input_numb':'goal_index'})
        smach.StateMachine.add('MOVE_BASE',
                               smach_ros.SimpleActionState('move_base', move_base_msgs.msg.MoveBaseAction,
                               goal_cb=move_goal_cb,
                               result_cb=move_result_cb,
                               input_keys=['list','index'],
                               output_keys=['index']),
                               transitions={'succeeded':'OPTION','aborted':'OPTION','preempted':'preempted'},
                               remapping={'list':'goal_list','index':'goal_index'})
        smach.StateMachine.add('GO_TO_COORDINATE',
                               MoveToCoordinate(),
                               transitions={'succeeded':'OPTION','aborted':'OPTION','preempted':'preempted'})

    # Execute SMACH plan
    outcome = sm0.execute()

    rospy.signal_shutdown('All done.')

if __name__ == '__main__':
    main()