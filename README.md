# move_base_clients
A ROS package which contains several clients for move_base navigation.  
This package provides move_base clients for different lauguages, such as C++, Python, HTML. You can access move_base in different ways. And the launch files can help you to addapt parameters of the clients with your unique robot.  
## Launch files
* move_base_clients.launch  
It's a launch file for C++ client with class. Set the arguments for your robot. e.g.  
`roslaunch move_base_clients move_base_clients.launch move_base_node:=move_base move_base_action:=move_base use_amcl:=false enable_clear_costmap:=false use_turtlebot3_buzzer:=false x:=1.0 y:=1.5 theta:=-1.57`  
### Arguments
1. move_base_node  
The name of move_base ROS node.
2. move_base_client  
The name of move_base action client.  
3. enable_clear_costmap  
Set this true if you want the client to call clear costmap service when the planner is not able to get a trajectory at the beginning. **This requires the name of the move_base node.**  
4. use_amcl  
Set this true if you want to update the localization of AMCL when the robot moves slow. Sometimes because of incorrect localization, the global planner is not albe to make an appropriate trajectory for the robot and that makes the robot move slow or oscillate. So in this case, this option can make the AMCL update without moving a certain distance or angle. **This option should only be used when you run AMCL as the localization module.**  
5. use_turtlebot3_buzzer  
If you are working with a [Turtlebot3](http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/), you can set this argument true. The robot can play a sound when it finishes the job. Have fun!  
6. x y theta  
The position and orientation of the goal.  
## Dynamic reconfigure
1. clear_costmap_threshold_dist  
2. clear_costmap_active_time  
If the robot doesn't get clear_costmap_threshold_dist meters away from the start position within clear_costmap_active_time seconds, the clear_costmaps service will be called.  
3. nmu_request_timer  
Reset number for the timer of nomotion update of amcl.  
## Executables
* simple_client  
A simple C++ client you can easily use with rosrun command. e.g.  
`rosrun move_base_clients simple_client 3.5 -0.34 -3.14`
* client.py  
A simple Python client you can easily use with rosrun command. e.g.  
`rosrun move_base_clients client.py 3.5 -0.34 -3.14`
* client_with_class  
A complex C++ client. You'd better start it with **move_base_clients.launch**.  
* web_nav.html  
A webpage navigation client which you can open it with a web browser. You can interact with the webpage like RViz. For more details, please visit [this tutorial](http://wiki.ros.org/nav2djs/Tutorials/CreatingABasicNav2DWidget).  
* client_with_smach.py  
A Python client with state machine smach, which contains a menu of options. e.g.  
`rosrun move_base_clients client_with_smach.py`  