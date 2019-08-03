#include <stdlib.h>//for converting argv to int
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>//move_base action
#include <actionlib/client/simple_action_client.h>//actionlib
#include <tf/transform_broadcaster.h>
#include <sstream>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void done_cbf(const actionlib::SimpleClientGoalState& state,const move_base_msgs::MoveBaseResultConstPtr& result_ptr)//action done callback
{    
    ROS_INFO("The action has finished.");
    if(state==actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("The robot has completed the task!");
        ros::shutdown();
    }
    else
    {
        ROS_INFO("The robot has failed to approch the goal.");
        if(state==actionlib::SimpleClientGoalState::ABORTED)
            ROS_INFO("The mission has been aborted.");
        ros::shutdown();
    }
}

void active_cbf()//action active callback
{
    ROS_INFO("The robot starts working.");
}

void feedback_cbf(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback_ptr)//action feedback callback
{
    ROS_INFO("The robot is at position (%.3f,%.3f)",feedback_ptr->base_position.pose.position.x,feedback_ptr->base_position.pose.position.y);
}

int main(int argc, char* argv[])
{
    ros::init(argc,argv,"move_base_simple_client");
    ros::NodeHandle nh;
    MoveBaseClient ac("move_base",true);//action name
    ROS_INFO("Wait for server start.");
    ac.waitForServer(ros::Duration(20.0));//wait for 20 seconds
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id="map";
    goal.target_pose.header.stamp=ros::Time::now();
    goal.target_pose.pose.position.x=0.0;
    goal.target_pose.pose.position.y=0.0;
    goal.target_pose.pose.position.z=0.0;
    tf::Quaternion tfq;
    geometry_msgs::Quaternion msgq;
    tfq.setRPY(0.0, 0.0, 0.0);//rpy to quaternion
    if(argc>=2)
    {
        goal.target_pose.pose.position.x=atof(argv[1]);
        if(argc>=3)
        {
            goal.target_pose.pose.position.y=atof(argv[2]);
            if(argc>=4)
            {
                tfq.setRPY(0.0, 0.0, atof(argv[3]));//set rpy to quaternion
            }
        }
    }
    tf::quaternionTFToMsg(tfq,msgq);//tf quaternion to msgs quaternion
    goal.target_pose.pose.orientation=msgq;
    ROS_INFO("Sending goal...");
    ac.sendGoal(goal,done_cbf,active_cbf,feedback_cbf);//send goal
    ros::spin();
    return 0;
}