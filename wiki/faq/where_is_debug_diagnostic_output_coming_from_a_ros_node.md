



## <a name="Q:_How_come_I_don't_see_any_debug_diagnostic_output_coming_from_a_ros_node?"></a>Q: How come I don't see any debug diagnostic output coming from a ros node?


You need to enable output to the screen in the roslaunch by adding output="screen" argument:



	output="screen" 

I would prefer to send diagnostics output to the log file

	output="log" 

The ROS console (which is a centralized diagnostic output/logging facility) handles the redirection to either the console or a log file.








