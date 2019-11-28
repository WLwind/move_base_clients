#pragma once

#include <iostream>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>//navigation mesages
#include <actionlib/client/simple_action_client.h>//actionlib
#include <tf/transform_listener.h>//tf
#include <turtlebot3_msgs/Sound.h>//turtlebot3 buzzer
#include <std_srvs/Empty.h>//amcl or clearcostmap service
#include <dynamic_reconfigure/server.h>
#include <move_base_clients/MoveBaseClientNodeConfig.h>//for ros dynamic_reconfigure

using MoveBaseClient=actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;
/**
* This class implements an action client for standard ROS move_base node.
*/
class MoveBaseClientNode
{
public:
    /**
    * @brief Constructor
    */
    MoveBaseClientNode(std::string node_name=std::string("move_base"),std::string action_name=std::string("move_base"));//constructor with action name
    /**
    * @brief Destructor
    */
    virtual ~MoveBaseClientNode(){}
    /**
    * @brief Set navigation goal (x,y,θ) (m,m,rad)
    * @param x Coordinate X of the goal pose
    * @param y Coordinate Y of the goal pose
    * @param theta Yaw of the goal pose
    */
    void setGoal(double,double,double);
    /**
    * @brief Get navigation goal
    */
    move_base_msgs::MoveBaseGoal getGoal() const;
    /**
    * @brief Wait for action result, 60 seconds as default
    * @param sec Seconds to wait for the action to finish
    * @return True if the action finishes in time, false if the action runs out of time
    */
    bool waitResult(int sec=60);
    /**
    * @brief Get action result
    * @return 0(succeeded), 1(aborted), 2(preempted), 3(others)
    */
    int getResult() const;//get action result. 0(succeeded), 1(aborted), 2(preempted), 3(others)

private:
    /**
    * @brief Action done callback function
    */
    void doneCb(const actionlib::SimpleClientGoalState& state,const move_base_msgs::MoveBaseResultConstPtr& result_ptr);
    /**
    * @brief Action active callback function
    */
    void activeCb();
    /**
    * @brief Action feedback callback function
    */
    void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback_ptr);
    /**
    * @brief Dynamic_reconfigure callback function
    */
    void reconfigureCB(move_base_clients::MoveBaseClientNodeConfig& config, uint32_t level);

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
    boost::shared_ptr< dynamic_reconfigure::Server<move_base_clients::MoveBaseClientNodeConfig> > dynamic_recfg_;//!< Dynamic reconfigure server to allow config modifications at runtime
    double journey{0.1};//the distance from the starting point to the goal
    double dist{0.1},dist_last{0.2};//distance to the goal，distance to the goal obtain from last feedback callback
    int request_timer{50};//the times of feedback callback activated, it will decrease progressiely, used for amcl nomotion update
    bool bclearmap{false};//clear costmap sign
    double begin{ros::Time::now().toSec()};//starting time
    double last_position[2]{0.0,0.0};//last feedback position coordinate
    bool enable_clear_costmap{false}, use_amcl{false}, use_turtlebot3_buzzer{false};//sign of param
    double clear_costmap_threshold_dist{0.3}, clear_costmap_active_time{5.0};//clear_costmap reconfigure param
    int nmu_request_timer{50};//no motion update timer for reconfigure
    bool shutdown_when_finish{true};//whether you want the node to be shut down when the goal finish
};//end class