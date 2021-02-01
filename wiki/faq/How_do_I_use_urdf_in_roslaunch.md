
## <a name="Q:_How_do_I_use_urdf_in_roslaunch?"></a>Q: How do I use urdf in roslaunch?


Using xacro in roslaunch






	<param  name="robot_description" command="$(find xacro)/xacro --inorder '$(find rospkg)/urdf/myrobot.xacro'" /> 
	 

using urdf file in roslaunch



	<param name="robot_description" textfile="$(arg rospkg)/urdf/myrobot.urdf" /> 




