#include <move_base_clients/MoveBaseClientNode.h>

MoveBaseClientNode::MoveBaseClientNode(std::string node_name,std::string action_name):ac(action_name, true)
{
    std::cout<<"MoveBaseClientNode constructor with the name of "<<action_name<<std::endl;
    ROS_INFO("Waiting for %s action to start...",action_name.c_str());
    if(ac.waitForServer(ros::Duration(20.0)))//wait for 20 seconds
    {
        ROS_INFO("%s action has started. The robot is ready to work.",action_name.c_str());
        std::cout<<"Initiate pointers"<<std::endl;
        nh_private.param<bool>("enable_clear_costmap",enable_clear_costmap,false);
        nh_private.param<bool>("use_amcl",use_amcl,false);
        nh_private.param<bool>("use_turtlebot3_buzzer",use_turtlebot3_buzzer,false);
        ROS_INFO("enable_clear_costmap:%d use_amcl:%d use_turtlebot3_buzzer:%d",enable_clear_costmap,use_amcl,use_turtlebot3_buzzer);
        if(enable_clear_costmap)//set client for clear_costmap
        {
            std::string service_name("/");
            clear_costmap.reset(new ros::ServiceClient(nh.serviceClient<std_srvs::Empty>(service_name.append(node_name).append("/clear_costmaps"))));
        }
        if(use_amcl)//set client for nomotion_update
        {
            nmu_sc.reset(new ros::ServiceClient(nh.serviceClient<std_srvs::Empty>("/request_nomotion_update")));
        }
        if(use_turtlebot3_buzzer)//set publisher for buzzer
        {
            buzzer_pub.reset(new ros::Publisher(nh.advertise<turtlebot3_msgs::Sound>("/sound",10)));
        }
        dynamic_recfg_=boost::make_shared< dynamic_reconfigure::Server<move_base_clients::MoveBaseClientNodeConfig> >(nh_private);
        dynamic_reconfigure::Server<move_base_clients::MoveBaseClientNodeConfig>::CallbackType cb = boost::bind(&MoveBaseClientNode::reconfigureCB, this, _1, _2);
        dynamic_recfg_->setCallback(cb);//setup dynamic_reconfigure
    }
    else
    {
        ROS_ERROR("%s action doesen't start during 20s. Timeout.",action_name.c_str());
        ros::shutdown();
    }
}

void MoveBaseClientNode::setGoal(double x=0.0,double y=0.0,double theta=0.0)
{
    try
    {
        tflistener.waitForTransform("map", "base_footprint", ros::Time(0), ros::Duration(10.0) );//wait for tf
        tflistener.lookupTransform("map", "base_footprint",ros::Time(0), start_point_transform);//save tf
        ROS_INFO("The transform from map to base_footprint has been heard.");
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::shutdown();
    }
    journey=sqrt(pow(start_point_transform.getOrigin().x()-x,2)+pow(start_point_transform.getOrigin().y()-y,2));//calculate the distance from starting point ot the goal
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
    return;
}

void MoveBaseClientNode::doneCb(const actionlib::SimpleClientGoalState& state,const move_base_msgs::MoveBaseResultConstPtr& result_ptr)
{
    turtlebot3_msgs::Sound buzzer;
    ROS_INFO("The action has finished.");
    final_state=state;//save the result
    if(state==actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        buzzer.value=1u;
        ROS_INFO("The robot has completed the task!");//succeeded
    }
    else
    {
        buzzer.value=0u;
        ROS_WARN("The robot has failed to approch the goal.");//failed
        if(state==actionlib::SimpleClientGoalState::ABORTED)//aborted
            ROS_INFO("The mission has been aborted.");
        else if(state==actionlib::SimpleClientGoalState::PREEMPTED)//preempted
            ROS_INFO("The mission has been preempted.");
        else if(state==actionlib::SimpleClientGoalState::RECALLED)//recalled
            ROS_INFO("The mission has been recalled.");
    }
    if(buzzer_pub.get()!=nullptr)
    {
        ROS_INFO("Play sound.");
        buzzer_pub->publish(buzzer);//buzzer plays sound
    }
    if(shutdown_when_finish)//shut down the node or not
    {
        ros::shutdown();
    }
    return;
}

void MoveBaseClientNode::activeCb()
{
    begin=ros::Time::now().toSec();//save the starting time
    ROS_INFO("The robot starts working.");
    return;
}

void MoveBaseClientNode::feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback_ptr)
{
    dist_last=dist;
    dist=sqrt(pow(feedback_ptr->base_position.pose.position.x-goal.target_pose.pose.position.x,2)+pow(feedback_ptr->base_position.pose.position.y-goal.target_pose.pose.position.y,2));
    ROS_INFO("coordinate:(%.2f,%.2f).%.2fm to the goal.%.1f%%",feedback_ptr->base_position.pose.position.x,feedback_ptr->base_position.pose.position.y,dist,100.0-dist/journey*100);

    if(clear_costmap.get()!=nullptr&&journey-dist<clear_costmap_threshold_dist&&bclearmap==false&&ros::Time::now().toSec()-begin>clear_costmap_active_time)//clear cost map if the robot doesn't get 30cm away from the starting point with in 5 seconds
    {
        std_srvs::Empty cl_cm;
        if(clear_costmap->call(cl_cm))
        {
            ROS_WARN("move_base:Costmaps have been cleared!");
            bclearmap=true;
        }
        else
        {
            ROS_WARN("move_base:Fail to clear costmaps!");
        }
    }


    if(nmu_sc.get()!=nullptr&&request_timer==0&&dist>0.35&&fabs(dist_last-dist)<0.0005&&sqrt(pow(feedback_ptr->base_position.pose.position.x-last_position[0],2)+pow(feedback_ptr->base_position.pose.position.y-last_position[1],2))<0.001)//if the distance between this feedback and the last one is too short then activate the nomotion_update service.
    {
        std_srvs::Empty nmu;
        if(nmu_sc->call(nmu))
        {
            ROS_WARN("amcl: Request nomotion update!");
            request_timer=nmu_request_timer;//set timer to 50，avoiding the service to be called too frequently
        }
        else
            ROS_WARN("amcl: Fail to update position without motion!");
    }
    else if(request_timer!=0)
    {
        request_timer--;//timer -1
    }
    last_position[0]=feedback_ptr->base_position.pose.position.x;//update the last feedback position
    last_position[1]=feedback_ptr->base_position.pose.position.y;
    return;
}

move_base_msgs::MoveBaseGoal MoveBaseClientNode::getGoal() const
{
    return goal;
}

bool MoveBaseClientNode::waitResult(int sec)
{
    shutdown_when_finish=false;//do not shut down when finished
    return ac.waitForResult(ros::Duration(sec));//blocking
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

void MoveBaseClientNode::reconfigureCB(move_base_clients::MoveBaseClientNodeConfig& config, uint32_t level)
{
    ROS_INFO("dynamic_reconfigure updates.");
    clear_costmap_threshold_dist=config.clear_costmap_threshold_dist;
    clear_costmap_active_time=config.clear_costmap_active_time;
    nmu_request_timer=config.nmu_request_timer;
    ROS_INFO("clear_costmap_threshold_dist = %.3f m. clear_costmap_active_time = %.1f s. nmu_request_timer = %d.",clear_costmap_threshold_dist,clear_costmap_active_time,nmu_request_timer);
    return;
}