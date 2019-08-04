#pragma once//c++11

#include <iostream>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>//navigation mesages
#include <actionlib/client/simple_action_client.h>//actionlib
#include <tf/transform_listener.h>//tf
#include <turtlebot3_msgs/Sound.h>//turtlebot3 buzzer
#include <std_srvs/Empty.h>//amcl or clearcostmap service

using MoveBaseClient=actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;//C++11

class MoveBaseClientNode
{
public:
    MoveBaseClientNode(std::string node_name=std::string("move_base"),std::string action_name=std::string("move_base"));//constructor with action name
    virtual ~MoveBaseClientNode(){}
    void setGoal(double,double,double);//set navigation goal (x,y,θ) (m,m,rad)
    void doneCb(const actionlib::SimpleClientGoalState& state,const move_base_msgs::MoveBaseResultConstPtr& result_ptr);//action done callback
    void activeCb();//action active callback
    void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback_ptr);//action feedback callback
    move_base_msgs::MoveBaseGoal getGoal() const;//get navigation goal
    bool waitResult(int sec=60);//wait for action result, 60 seconds as default
    int getResult() const;//get action result. 0(succeeded), 1(aborted), 2(preempted), 3(others)

private:
    ros::NodeHandle nh;
    ros::NodeHandle nh_private{"~"};//private node handle for reading parameters
    MoveBaseClient ac{"move_base", true};//action client, default name is move_base
    move_base_msgs::MoveBaseGoal goal;//move_base navigation goal
    actionlib::SimpleClientGoalState final_state{actionlib::SimpleClientGoalState::ABORTED};//the result of the action
    tf::TransformListener tflistener;
    tf::StampedTransform start_point_transform;//starting point
    std::unique_ptr<ros::ServiceClient> clear_costmap, nmu_sc;//move_base clear costmap client, amcl nomotion update service client
    std::unique_ptr<ros::Publisher> buzzer_pub;//publisher for turtlebot3 buzzer
    std::string move_base_node_name;//name of the node, it's necessary if you want to use clear_costmap
    double journey{0.1};//the distance from the starting point to the goal
    double dist{0.1},dist_last{0.2};//distance to the goal，distance to the goal obtain from last feedback callback
    int request_timer{50};//the times of feedback callback activated, it will decrease progressiely, used for amcl nomotion update
    bool bclearmap{false};//clear costmap sign
    double begin{ros::Time::now().toSec()};//starting time
    double last_position[2]{0.0,0.0};//last feedback position coordinate
    bool enable_clear_costmap{false}, use_amcl{false}, use_turtlebot3_buzzer{false};
};