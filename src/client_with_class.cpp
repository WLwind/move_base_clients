#include <stdlib.h>//for converting argv to int
#include <move_base_clients/MoveBaseClientNode.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "move_base_client_with_class");
    std::unique_ptr<MoveBaseClientNode> node_ptr;
    if(argc<=4)//without name of node or action
    {
        node_ptr.reset(new MoveBaseClientNode());
    }
    else if(argc==5)//with name of node
    {
        node_ptr.reset(new MoveBaseClientNode(std::string(argv[4])));
    }
    else if(argc==6)//with name of node and action
    {
        node_ptr.reset(new MoveBaseClientNode(std::string(argv[4]),std::string(argv[5])));
    }
    else//error
    {
        ROS_ERROR("The argc should be less than 7! Your argc is %d",argc);
        return 3;
    }
    node_ptr->setGoal(argc>1?atof(argv[1]):0.0,argc>2?atof(argv[2]):0.0,argc>3?atof(argv[3]):0.0);
    move_base_msgs::MoveBaseGoal read_goal(node_ptr->getGoal());//for print
    ROS_INFO("The destination is (%.2f,%.2f)",read_goal.target_pose.pose.position.x,read_goal.target_pose.pose.position.y);
    ros::spin();
    return node_ptr->getResult();//return the result
}