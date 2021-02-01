## <a name="Q:_Can_I_forward_the_joint_states_to_another_topic_(e.g.,_that_which_has_a_ns_attribute)"></a>Q: Can I forward the joint states to another topic (e.g., that which has a ns attribute)?


topic_tools package found in /opt/ros/kinetic/share/topic_tools





Tools for directing, throttling, selecting, and otherwise messing with ROS topics at a meta level. None of the programs in this package know about the topics whose streams they are altering; instead, these tools deal with messages as generic binary blobs. This means they can be applied to any ROS topic.


Useful for joint_states published to a qualified namespace topic. Namespaces are not easy in ROS. Example in roslaunch follows:


	<!-- run a relay to make joint_states available on ariac/joint_states --> 
	<node name="relay_joint_states" pkg="topic_tools" type="relay" args="/joint_states /ariac/joint_states" required="true" output="log" /> 

topic_tools has a lot of other capabilities for mutating contents of ROS topic communication.
