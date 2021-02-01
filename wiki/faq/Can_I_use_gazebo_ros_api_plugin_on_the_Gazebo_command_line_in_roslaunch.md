## <a name="Q:_Can_I_use_gazebo_ros_api_plugin_on_the_Gazebo_command_line_in_roslaunch_like_I_can_when_I_invoke_gazebo_from_the_command_line?"></a>Q: Can I use gazebo_ros_api plugin on the Gazebo command line in roslaunch like I can when I invoke gazebo from the command line?


A: Yes. 

From:  [<u>here</u>](https://answers.gazebosim.org/question/4981/how-can-i-pass-a-system-plugin-in-my-launch-file/)


ROS can't directly run gazebo because gazebo is a stand alone package. The gazebo_ros package works around this limitation by wrapping the gazebo executable inside a shell script. Only a few command line arguments are exposed if you use the  empty_world.launch file.


You can pass in your own command line arguments using:



	<node name="gazebo" pkg="gazebo_ros" type="gazebo"
	 respawn="false" output="screen"
	 args="-s<put_your_system_plugin_here.so>" /> 

The following sketches the crcl2ros gz.launch file with gazebo_ros_api plugin and specifying a Gazebo SDF world that contains multiple robots: :


	<!-- start gazebo server-->
	<arg name="world_name" default="$(find fanuc_lrmate200id_support)/world/aprs-fanuc.world" />
	<arg name="physics" default="ode" />
	
	<node name="gazebo" pkg="gazebo_ros" type="$(arg script_type)" respawn="$(arg respawn_gazebo)" output="screen" 
	args= "-s libgazebo_ros_api_plugin.so --verbose -e $(arg physics) $(arg world_name)">
		<env name="GAZEBO_PLUGIN_PATH" value="/home/michalos/ros/agility/devel/lib:$(optenv GAZEBO_PLUGIN_PATH)" />
		<env name="GAZEBO_MODEL_PATH" value="/home/michalos/src/gzgwendolen/gzdatabase/models:$(optenv GAZEBO_MODEL_PATH)" />
	</node>
