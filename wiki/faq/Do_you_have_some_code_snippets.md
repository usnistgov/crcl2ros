
# Code snippets in ROS

## Get the path to a ROS package (assume in ROS PATH)

	packagepath = ros::package::getPath(ROSPACKAGENAME);

# What are the catkin variables (not cmake)
https://docs.ros.org/jade/api/catkin/html/user_guide/variables.html

## Get the path to a ROS package (assume in ROS PATH)

	packagepath = ros::package::getPath(ROSPACKAGENAME);

# Check if master running if not delay

	if(! ros::master::check())
	{
		std::cerr << "ROS Master Not Running\n";
		ros::Duration(1.0).sleep()         
	}

https://levelup.gitconnected.com/ros-spinning-threading-queuing-aac9c0a793f