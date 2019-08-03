#! /usr/bin/env python
from __future__ import print_function
import sys
import rospy
# Brings in the SimpleActionClient
import actionlib
import move_base_msgs.msg
import geometry_msgs.msg
import tf

def move_base_action_client(goalx=0,goaly=0,goaltheta=0):
    client=actionlib.SimpleActionClient('move_base',move_base_msgs.msg.MoveBaseAction)
    client.wait_for_server(rospy.Duration(20))
    goal=move_base_msgs.msg.MoveBaseGoal()
    goal.target_pose.header.frame_id='map'
    goal.target_pose.header.stamp=rospy.Time.now()
    goal.target_pose.pose.position.x=goalx
    goal.target_pose.pose.position.y=goaly
    goal.target_pose.pose.position.z=0.0
    q_angle = tf.transformations.quaternion_from_euler(0.0,0.0,goaltheta)
    q = geometry_msgs.msg.Quaternion(*q_angle)
    goal.target_pose.pose.orientation=q
    rospy.loginfo("sending goal")
    client.send_goal(goal,done_cb=final_cb,active_cb=start_cb,feedback_cb=fb_cb)
    client.wait_for_result()
    client.get_result()
    return

def final_cb(status, result):
    """move_base action done callback"""
    rospy.loginfo("Python client has received the result: %d",status)
    if status==3:
        rospy.loginfo("The robot has got the destination!")
    else:
        rospy.logwarn("The robot failed to get the goal!")
    return

def start_cb():
    """move_base action active callback"""
    rospy.loginfo("The robot start navigating.")
    return

def fb_cb(feedback):
    """move_base active callback"""
    rospy.loginfo("The robot is at (%.3f,%.3f)",feedback.base_position.pose.position.x,feedback.base_position.pose.position.y)
    return

if __name__ == '__main__':
    print(len(sys.argv),str(sys.argv))
    X=0.0
    Y=0.0
    Theta=0.0
    if len(sys.argv)>1:
        X=float(sys.argv[1])
        if len(sys.argv)>2:
            Y=float(sys.argv[2])
            if len(sys.argv)>3:
                Theta=float(sys.argv[3])
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('action_client_py')
        move_base_action_client(X,Y,Theta)
        rospy.loginfo("received goal")
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)