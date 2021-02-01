## <a name="Q:_How_do_I_set_an_environment_variable_in_roslaunch?"></a>Q: How do I set an environment variable in roslaunch?


[<u>here</u>](https://answers.gazebosim.org/question/15200/cant-load-plugin-with-roslaunch/)



	<launch> 
	<!-- Set up env variable so plugin are found --> 
	<env name="GAZEBO_PLUGIN_PATH" value="$(find car_gazebo)/plugins"/> 

Which in a roslaunch gazebo server<node> XML 



	<!-- start gazebo server--> 
	<node name="gazebo" pkg="gazebo_ros" type="$(arg script_type)" . . ."> 
		<env name="GAZEBO_PLUGIN_PATH" value="/home/michalos/ros/agility/devel/lib:$(optenv GAZEBO_PLUGIN_PATH)" /> 
		<env name="GAZEBO_MODEL_PATH" value="/home/michalos/src/gzgwendolen/gzdatabase/models:$(optenv 
	GAZEBO_MODEL_PATH)" /> 
</node> 

This works.

The package.xml also can have export environment variables defined:

	. . .
		<export> 
			<gazebo_ros plugin_path="${prefix}/plugins"/> 
		</export> 
	</package> 

Have not tried the package.xml approach.