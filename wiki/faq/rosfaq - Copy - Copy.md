



##<a name="Q:_How_to_handle_multiple_URDF_files_in_a_single_moveit_package"></a>Q: How to handle multiple URDF files in a single moveit package?


[<u>here</u>](https://gcc02.safelinks.protection.outlook.com/?url=https%3A%2F%2Fanswers.ros.org%2Fquestion%2F298024%2Fhow-to-handle-multiple-urdf-files-in-a-single-moveit-package%2F&amp;data=04%7C01%7Cjohn.michaloski%40nist.gov%7Cd29b6381afc64472620908d89abf4c33%7C2ab5d82fd8fa4797a93e054655c61dec%7C1%7C1%7C637429493705595297%7CUnknown%7CTWFpbGZsb3d8eyJWIjoiMC4wLjAwMDAiLCJQIjoiV2luMzIiLCJBTiI6Ik1haWwiLCJXVCI6Mn0%3D%7C3000&amp;sdata=z3%2BsvlIsvEKkR4UvK3Qe%2FKyN2RblicKAgH47XkDOmcw%3D&amp;reserved=0)


The moveit_setup_assistant doesn't support doing that, but you can do it yourself by editing the files inside your moveit config package.  THIS IS VERY TRICKY.


Assuming the only thing that needs to change between your moveit packages is the SRDF, and you already have one SRDF for each robot-tool configuration. Then you can add "robot" and "tool" args to your planning_config.launch to look something like this:



<launch> 
	  [...] 
	 
<arg name="robot" default="myrobot1"/> 
<arg name="tool" default="mytool1"/> 
	 
<!-- Load universal robot description format (URDF) --> 
<param if="$(arg load_robot_description)" name="$(arg robot_description)" command="$(find xacro)/xacro --inorder $(find myrobot_description)/urdf/$(robot)_$(tool).urdf.xacro" /> 
	 
<!-- The semantic description that corresponds to the URDF --> 
<param name="$(arg robot_description)_semantic" textfile="$(find myrobot_moveit_config)/config/$(robot)_$(tool).srdf" /> 
	 
	   [...] 
</launch> 

... and then of course you need to edit the launch files that include planning_context.launch and pass through the robot and tool args. If you do a diff between your current "robot+tool" moveit configs and find things that are different between them other than the SRDF, you need to make them parametrizable as well, perhaps doing something like this in the launch files:



<include if="$(eval robot == 'myrobot1')" [...] /> 







##<a name="Q:_Where_can_I_find_documentation_on_moveit_assistant?"></a>Q: Where can I find documentation on moveit assistant?


[<u>here</u>](http://docs.ros.org/en/hydro/api/moveit_setup_assistant/html/doc/tutorial.html)


##<a name="Q:_More_on_robot_model"></a>Q: More on robot model

A: [<u>here</u>](https://programmersought.com/article/86995403628/)




##<a name="Q:_How_do_I_handle_Multi-robot_controller-manager_for_different_robot_description?"></a>Q: How do I handle Multi-robot controller-manager for different robot_description?


[<u>here</u>](https://answers.gazebosim.org/question/9403/multi-robot-controller-manager-for-different-robot_descripion/)


##<a name="Q:_What_about_Robot_model_parameter_not_found"></a>Q: What about Robot model parameter not found


[<u>here</u>](A:) [<u>here</u>](https://answers.ros.org/question/317217/robot-model-parameter-not-found/) 




##<a name="Q:_Is_there_an_environmental_variable_for_current_namespace_in_launch_files?"></a>Q: Is there an environmental variable for current namespace in launch files?


A:  [<u>here</u>](https://answers.ros.org/question/212382/is-there-a-environmental-variable-for-current-namespace-in-launchfiles/) 





##<a name="Q:_Where_is_some_documentation_on_Robot_Model_and_Robot_State"></a>Q: Where is some documentation on Robot Model and Robot State


A:  [<u>here</u>](http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/robot_model_and_robot_state/robot_model_and_robot_state_tutorial.html)





##<a name="Q:_How_can_I_run_multiple_move_group_for_different_robots_in_single_system?"></a>Q: How can I run multiple move_group for different robots in single system?


A: [<u>here</u>](https://stackoverflow.com/questions/54926822/how-run-multiple-move-group-for-different-robots-in-single-system)


tf discussion in robot_state_publisher  answer is discouraging, however, jives with what I have found.














##<a name="Q:_Passing_arg_to_rosrun"></a>Q: Passing arg to rosrun


[<u>here</u>](https://answers.ros.org/question/299014/command-line-argument-passing-in-rosrun/) 


rosrun package node_name _param:=blue



##<a name="Q:_Where_are_the_OSRF_ARIAC_Code_repositories_found?
[<u>here</u>](https://bitbucket.org/osrf/ariac/wiki/browse/)"></a>Q: Where are the OSRF ARIAC Code repositories found?
[<u>here</u>](https://bitbucket.org/osrf/ariac/wiki/browse/) 


OSRF is the "owner" of ROS/Gazebo so they know the ins/outs of ROS machinations.




##<a name="Q:_How_do_I_get_the_ROS_namespace_that_the_C++_package_was_spawned?
Programmatically_get_node's_Namespace:_[<u>here</u>](https://answers.ros.org/question/62620/programmatically-get-nodes-namespace/)"></a>Q: How do I get the ROS namespace that the C++ package was spawned?
Programmatically get node's Namespace: [<u>here</u>](https://answers.ros.org/question/62620/programmatically-get-nodes-namespace/)



	std::string ns = ros::this_node::getNamespace(); 

and



	ns = rospy.get_namespace() 




##<a name="Q:_How_do_I_"find"_a_package_in_a_rosrun_argument?"></a>Q: How do I "find" a package in a rosrun argument?


ROS answers for arg with find in eval [<u>here</u>](http://wiki.ros.org/roslaunch/XML)



<arg name="foo" value="$(eval + find('pkg')"/> 

Can be done with param. Basically you can set the package path ONCE at the beginning and pass it through nested roslaunch.


##<a name="Q:_Getting_rosparam_from_node_namespace_within_bash_script?"></a>Q: Getting rosparam from node namespace within bash script?


[<u>here</u>](https://answers.ros.org/question/220941/getting-rosparam-from-node-namespace-within-bash-script/)







##<a name="Q:_How_can_I_get_my_cout_output_on_the_console_when_using_roslaunch?"></a>Q: How can I get my cout output on the console when using roslaunch?


Plus, I'm using ROS_DEBUG and am not seeing any output on the ROS console. Also no std::cout either.


In the Roslaunch XML add the following within your node tags:



	output="screen" 

otherwise only ROS_ERROR will be printed to the screen.


For example, I added output="screen" 



<node name="movemove" pkg="move_it" type="move_it" output="screen"  
	args="armgroup:=$(arg armgroup)" 

and now I see ROS debug messages and std::cout streamed output without annoying timestamp!


##<a name="Q:_How_do_I_set_a_delay_between_starting_nodes_within_launch_file"></a>Q: How do I set a delay between starting nodes within launch file


[<u>here</u>](https://answers.ros.org/question/233353/set-delay-between-starting-nodes-within-launch-file/) 



<arg name="node_start_delay" default="1.0" /> 
<node name="listener" pkg="roscpp_tutorials" type="listener"  
	launch-prefix="bash -c 'sleep $(arg node_start_delay); $0 $@' " /> 

I agree, its ugly.







##<a name="Q:_Can_I_read_and_modify_rosparam_?"></a>Q: Can I read and modify rosparam ?


[<u>here</u>](https://gcc02.safelinks.protection.outlook.com/?url=https%3A%2F%2Fwiki.ros.org%2Froscpp%2FOverview%2FParameter%2520Server%23Retrieving_Lists&data=04%7C01%7Cjohn.michaloski%40nist.gov%7C441e1a64d1c64f15fd1b08d8a2c3866d%7C2ab5d82fd8fa4797a93e054655c61dec%7C1%7C0%7C637438307947362129%7CUnknown%7CTWFpbGZsb3d8eyJWIjoiMC4wLjAwMDAiLCJQIjoiV2luMzIiLCJBTiI6Ik1haWwiLCJXVCI6Mn0%3D%7C1000&sdata=IBQ%2FMUHt8frtLh%2BmuYFHPAkz0TLxX09JrHf03iFWT%2FM%3D&reserved=0)







##<a name="Q:_How_can_I_pass_pose_messages_for_moving_arm_in_a_straight_line?"></a>Q: How can I pass pose messages for moving arm in a straight line?


[<u>here</u>](https://answers.ros.org/question/96331/how-can-i-pass-pose-messages-for-moving-arm-in-a-stright-line/)


##<a name="Q:_Cartesian_controller_for_ROS"></a>Q: Cartesian controller for ROS


This is a very good treatise on Cartesian controller development in ROS.


[<u>here</u>](https://answers.ros.org/question/74776/cartesian-controller-for-ros/) 






