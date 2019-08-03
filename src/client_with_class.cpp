#include <stdlib.h>//for converting argv to int
#include <move_base_clients/MoveBaseClientNode.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "move_base_client_with_class");
    MoveBaseClientNode movebaseclient;
    movebaseclient.setGoal(argc>1?atof(argv[1]):0.0,argc>2?atof(argv[2]):0.0,argc>3?atof(argv[3]):0.0);
    move_base_msgs::MoveBaseGoal read_goal(movebaseclient.getGoal());//for print
    ROS_INFO("The destination is (%.2f,%.2f)",read_goal.target_pose.pose.position.x,read_goal.target_pose.pose.position.y);
    if(movebaseclient.waitResult(60))//wait for 60 seconds
        return movebaseclient.getResult();//0 is succeed
    else
        return 3;//run out of time
}