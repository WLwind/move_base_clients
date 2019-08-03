#pragma once//c++11

#include <iostream>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>//navigation mesages
#include <actionlib/client/simple_action_client.h>//actionlib
#include <tf/transform_listener.h>//tf
//#include <turtlebot3_msgs/Sound.h>//turtlebot3 buzzer
//#include <std_srvs/Empty.h>//amcl or clearcostmap service

using MoveBaseClient=actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;//C++11

class MoveBaseClientNode
{
public:
    MoveBaseClientNode();//constructor
    void setGoal(double,double,double);//set navigation goal (x,y,θ) (m,m,rad)
    void doneCb(const actionlib::SimpleClientGoalState& state,const move_base_msgs::MoveBaseResultConstPtr& result_ptr);//action done callback
    void activeCb();//action active callback
    void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback_ptr);//action feedback callback
    move_base_msgs::MoveBaseGoal getGoal() const;//get navigation goal
    bool waitResult(int=60);//wait for action result
    int getResult() const;//get action result

private:
    ros::NodeHandle nh;
    MoveBaseClient ac{"move_base", true};//action name
    move_base_msgs::MoveBaseGoal goal;//move_base navigation goal
    actionlib::SimpleClientGoalState final_state{actionlib::SimpleClientGoalState::ABORTED};//the result of the action
    double journey{0.1};//the distance from the starting point to the goal
    tf::TransformListener tflistener;
    tf::StampedTransform start_point_transform;//starting point
//    ros::Publisher buzzer_pub{nh.advertise<turtlebot3_msgs:Sound>("test_topic",10)};//publisher for turtlebot3 buzzer
//    ros::ServiceClient nmu_sc{nh.serviceClient<std_srvs::Empty>("/request_nomotion_update")};//amcl nomotion update service client
//    ros::ServiceClient clear_costmap{nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps")};//move_base clear costmap client
    double dist{0.1},dist_last{0.2};//distance to the goal，distance to the goal obtain from last feedback callback
    int request_timer{50};//the times of feedback callback activated, it will decrease progressiely, used for amcl nomotion update
//    bool bfast{false},bclearmap{false};//fast speed sign for dwa
    double begin{ros::Time::now().toSec()};//starting moment
    double last_position[2];//last feedback position
};