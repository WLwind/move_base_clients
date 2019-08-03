#include <move_base_clients/MoveBaseClientNode.h>

MoveBaseClientNode::MoveBaseClientNode()// : ac("move_base", true),journey(0.1),nmu_sc(nh.serviceClient<std_srvs::Empty>("/request_nomotion_update")),dist(0.1),dist_last(0.2),request_timer(50),bfast(false),clear_costmap(nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps")),begin(ros::Time::now().toSec()),bclearmap(false),final_state(actionlib::SimpleClientGoalState::ABORTED)
{
    std::cout<<"MoveBaseClientNode constructor..."<<std::endl;
    ROS_INFO("Waiting for move_base node to start...");
    if(ac.waitForServer(ros::Duration(20.0)))//wait for 20 seconds
        ROS_INFO("move_base node has started. The robot is ready to work.");
    else
    {
        ROS_ERROR("move_base node doesen't start during 20s. Timeout.");
        ros::shutdown();
    }
}

void MoveBaseClientNode::setGoal(double x=0.0,double y=0.0,double theta=0.0)
{
    try
    {
        tflistener.waitForTransform("/map", "/base_footprint", ros::Time(0), ros::Duration(10.0) );//wait for tf
        tflistener.lookupTransform("/map", "/base_footprint",ros::Time(0), start_point_transform);//save tf
        ROS_INFO("The transform from /map to /base_footprint has been heard.");
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::shutdown();
    }
    journey=sqrt(pow(start_point_transform.getOrigin().x()-x,2)+pow(start_point_transform.getOrigin().y()-y,2));//计算起点到终点的距离
    ROS_INFO("The starting point is (%.2f,%.2f).The target point is(%.2f,%.2f). The distance between them is %.2fm",start_point_transform.getOrigin().x(),start_point_transform.getOrigin().y(),x,y,journey);
    last_position[0]=start_point_transform.getOrigin().x();//save the position as the last one
    last_position[1]=start_point_transform.getOrigin().y();
    goal.target_pose.header.frame_id="map";
    goal.target_pose.header.stamp=ros::Time::now();
    goal.target_pose.pose.position.x=x;
    goal.target_pose.pose.position.y=y;
    goal.target_pose.pose.position.z=0.0;
    tf::Quaternion tfq;
    geometry_msgs::Quaternion msgq;
    tfq.setRPY(0.0, 0.0, theta);//rpy to quaternion
    tf::quaternionTFToMsg(tfq,msgq);//tf quaternion to msg quaternion
    goal.target_pose.pose.orientation=msgq;
    ROS_INFO("Sending goal...");
    ac.sendGoal(goal,std::bind(&MoveBaseClientNode::doneCb,this,std::placeholders::_1,std::placeholders::_2),std::bind(&MoveBaseClientNode::activeCb,this),std::bind(&MoveBaseClientNode::feedbackCb,this,std::placeholders::_1));//send goal
}

void MoveBaseClientNode::doneCb(const actionlib::SimpleClientGoalState& state,const move_base_msgs::MoveBaseResultConstPtr& result_ptr)//action结束时回调函数
{
//    turtlebot3_msgs::Sound buzzer;
    ROS_INFO("The action has finished.");
    final_state=state;//save the result
    if(state==actionlib::SimpleClientGoalState::SUCCEEDED)
    {
//        buzzer.value=1u;
        ROS_INFO("The robot has completed the task!");//succeeded
    }
    else
    {
 //       buzzer.value=0u;
        ROS_WARN("The robot has failed to approch the goal.");//failed
        if(state==actionlib::SimpleClientGoalState::ABORTED)//aborted
            ROS_INFO("The mission has been aborted.");
        else if(state==actionlib::SimpleClientGoalState::PREEMPTED)//preempted
            ROS_INFO("The mission has been preempted.");
        else if(state==actionlib::SimpleClientGoalState::RECALLED)//recalled
            ROS_INFO("The mission has been recalled.");
    }
//    ros::Duration(1.0).sleep();
//    buzzer_pub.publish(buzzer);//make buzzer phonate
}

void MoveBaseClientNode::activeCb()
{
    begin=ros::Time::now().toSec();//save the starting moment
    ROS_INFO("The robot starts working.");
}

void MoveBaseClientNode::feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback_ptr)
{
    dist_last=dist;
    dist=sqrt(pow(feedback_ptr->base_position.pose.position.x-goal.target_pose.pose.position.x,2)+pow(feedback_ptr->base_position.pose.position.y-goal.target_pose.pose.position.y,2));
    ROS_INFO("coordinate:(%.2f,%.2f).%.2fm to the goal.%.1f%%",feedback_ptr->base_position.pose.position.x,feedback_ptr->base_position.pose.position.y,dist,100.0-dist/journey*100);
/*
    if(journey-dist<0.3&&bclearmap==false&&ros::Time::now().toSec()-begin>6.0)//clear cost map if the robot doesn't get 30cm away from the starting point
    {
        std_srvs::Empty cl_cm;
        if(clear_costmap.call(cl_cm))
        {
            ROS_INFO("move_base:Costmaps have been cleared!");
            bclearmap=true;
        }
        else
        {
            ROS_WARN("move_base:Fail to clear costmaps!");
        }
    }
*/
/*
    if(request_timer==0&&dist>0.35&&fabs(dist_last-dist)<0.001&&sqrt(pow(feedback_ptr->base_position.pose.position.x-last_position[0],2)+pow(feedback_ptr->base_position.pose.position.y-last_position[1],2))<0.001)//如果两次反馈之间移动距离过短，启动amcl不运动定位
    {
        std_srvs::Empty nmu;
        if(nmu_sc.call(nmu))
        {
            ROS_INFO("AMCL: Request nomotion update!%f",fabs(dist_last-dist));
            request_timer=30;//set timer to 30，avoiding the service to be called too frequently
        }
        else
            ROS_WARN("AMCL: Fail to update position without motion!");
    }
    else if(request_timer!=0)
        request_timer--;//timer -1
*/
/*
    if(dist/journey<0.85&&dist/journey>0.7&&bfast==false)//触发高速前进
    {
        ros::param::set("/move_base/DWAPlannerROS/max_trans_vel",0.22);
        bfast=true;
    }
    else if(bfast==true&&dist/journey<0.35)//触发低速前进
    {
        ros::param::set("/move_base/DWAPlannerROS/max_trans_vel",0.15);
        bfast=false;
    }
*/
    last_position[0]=feedback_ptr->base_position.pose.position.x;//update the last feedback position
    last_position[1]=feedback_ptr->base_position.pose.position.y;
}

move_base_msgs::MoveBaseGoal MoveBaseClientNode::getGoal() const
{
    return goal;
}

bool MoveBaseClientNode::waitResult(int sec)
{
    return ac.waitForResult(ros::Duration(sec));
}

int MoveBaseClientNode::getResult() const
{
    if(final_state==actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        std::cout<<"return 0(succeeded)"<<std::endl;
        return 0;
    }
    else if(final_state==actionlib::SimpleClientGoalState::ABORTED)
    {
        std::cout<<"return 1(aborted)"<<std::endl;
        return 1;
    }
    else if(final_state==actionlib::SimpleClientGoalState::PREEMPTED)
    {
        std::cout<<"return 2(preempted)"<<std::endl;
        return 2;
    }
    else
    {
        std::cout<<"return 3"<<std::endl;
        return 3;
    }
}