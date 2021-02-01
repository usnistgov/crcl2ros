## <a name="Q:_How_do_I_set_a_delay_between_starting_nodes_within_launch_file"></a>Q: How do I set a delay between starting nodes within launch file


Answer found [<u>here</u>](https://answers.ros.org/question/233353/set-delay-between-starting-nodes-within-launch-file/) 

Code is:

	<arg name="node_start_delay" default="1.0" /> 
	<node name="listener" pkg="roscpp_tutorials" type="listener"  
	launch-prefix="bash -c 'sleep $(arg node_start_delay); $0 $@' " /> 

I agree, its ugly.
