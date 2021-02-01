
## <a name="Q:_What_is_the_difference_between_private_param_and_global_param?"></a>Q: What is the difference between private rosparam and global rosparam?

From: https://answers.ros.org/question/174099/string-argument-roslaunch/

These are private  rosparam (as they are embedded inside the &lt;node&gt; tag):

	  <node name="right_motor" pkg="driver" type="right" output="screen"> 
	       <param name="KP" type="string" value="$(arg my_KP)" /> 
	        <param name="KI" type="string" value="$(arg my_KI)"/> 
	        <param name="KD" type="string" value="$(arg my_KD)"/> 
	        <param name="EL" type="string" value="$(arg my_EL)"/> 
	        <param name="ADT" type="string" value="$(arg my_ADT)"/> 
	   </node> 

which are either accesses by a PRIVATE NODE HANDLE:


	ros::NodeHandle p_nh("~"); 
	p_nh.param<string>("KP", KP, "1000"); 
	... 

or you could use the "bare" versions from the API i.e. your parameter calls would read something like


	ros::param::param<string>("~KI", KI, "240"); 

More ROS param information can be found at: [<u>here</u>](http://wiki.ros.org/roscpp/Overview/Parameter%20Server)


Or you could make all the ros params global by defining them OUTSIDE of the &lt;node&gt; branch:


	<param name="KP" type="string" value="$(arg my_KP)" /> 
	<param name="KI" type="string" value="$(arg my_KI)"/> 
	<param name="KD" type="string" value="$(arg my_KD)"/> 
	<param name="EL" type="string" value="$(arg my_EL)"/> 
	<param name="ADT" type="string" value="$(arg my_ADT)"/> 
	
	<node name="right_motor" pkg="driver" type="right" output="screen"> 
	</node> 



It is important to understand the distinction.
