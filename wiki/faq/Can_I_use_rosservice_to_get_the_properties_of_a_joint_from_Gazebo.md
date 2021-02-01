
## <a name="Q:_Can_I_use_rosservice_to_get_the_properties_of_a_joint_from_Gazebo,_including_the_current_position?"></a>Q: Can I use rosservice to get the properties of a joint from Gazebo, including the current position?


A: Yes.  You need to query the model name ( fanuc_lrmate200id) and the joint name ( fanuc_joint_1) as described in the URDF and as shown below:



	> rosservice call /gazebo/get_joint_properties "fanuc_lrmate200id::fanuc_joint_1" 
	type: 0 
	damping: [] 
	position: [-1.0302869668521453e-13] 
	rate: [2.049300806658447e-20] 
	success: True 
	status_message: "GetJointProperties: got properties" 
	 
	Current pose 
	position:  
	  x: 0.464905 
	  y: 0 
	  z: 0.695066 
	orientation:  
	  x: 0 
	  y: -7.95692e-05 
	  z: 0 
	  w: 1 
	 
	Goal pose 
	position:  
	  x: 0.4 
	  y: -0.06 
	  z: 0.05 
	orientation:  
	  x: 0 
	  y: 0.707107 
	  z: 0 
	  w: 0.707107 
	 




Since service calls are blocking, it will return once the call is done. Of course each service call could invoke a thread so its asynchronous under the hood.


