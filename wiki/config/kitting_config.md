

The xform Yaml subbranch defines various Homogeneous transform between coordinate frames. 



	xform: 
		# retract transform from destination pose length in meters, orientation rpy	 
	  	retract: 	[0.0,0.0,0.04,0.0,0.0,0.0] 
		# base transform from world coordinate frame to robot origin		 
	  	base: 	[-0.169, -1.140, 0.934191,  0.0,0.0,0.0] 
	  	# Quaternion for robot orientation to point down at kitting gears 
	  	qbend: 	[0,0.707107,0,0.707107] 
		# transform from final robot transform (e.g., T6) to end effector coordinate 
	  	tool: 	[0.0, 0.0, .182, 0.0,0.0,0.0] 
	       # ibid tool transform 
	  	gripper: 	[0.0, 0.0, .182, 0.0,0.0,0.0] 

Offset defines a Yaml subbranch for Z offsets from the base location of a kitting model. For example, offset above base of gear bottom to grasping peg location.



	offset: 
		# offset from bottom location to above vessel slot 
	  	vesselslot: [0.00,0.00, 0.035, 0.0, 0.0.,0.0] 
		# offset from bottom location of large gear to grasping peg point 
	  	largegear:  [0.0,0.0, 0.015, 0.0, 0.0.,0.0] 
		# offset from bottom location of medium gear to grasping peg point 
	  	mediumgear: [0.0,0.0, 0.015, 0.0, 0.0.,0.0] 
		# offset from bottom location of small gear to grasping peg point 
	  	smallgear:  [0.0,0.0, 0.015, 0.0, 0.0.,0.00] 




The nc subbranch defines the names of the links for the base and the tip of the robot series of transformations in URDF.

	nc: 
	  baselink: fanuc_base_link 
	  tiplink: fanuc_link_6 


MOveit covers all the robot implementation parameters used in moveit!. The "armgroup" is currently sent in as an arg to crclapp.
	moveit:
		# name of topic for joint state publisher
		# must match name in roslaunch:
		#
		joint_state_publisher_topic: /lrmate/joint_states
		
		# gripper ROS topic name in SDF file
		gripper_topic: /fanuc_lrmate200id/control
		
		# robot model name in gazebo (outermost model name)
		robot_model_name: lrmate
		
		# flag to send joint updates to gazebo 
		use_gazebo: 1


The gazebo subbranch defines gazebo related parameters. Need to double check use of some of the variable definitions.


	gazebo: 
	  modeltopicname: /gazebo/default/ariac/model 
	  gzJointPrefix: 'lrmate::fanuc_lrmate200id::' 
	  gzLeftFingerContactTopic: /gazebo/default/lrmate/fanuc_lrmate200id/motoman_left_finger/left_finger_contact 
	  gzRightFingerContactTopic: /gazebo/default/lrmate/fanuc_lrmate200id/motoman_right_finger/right_finger_contact 
	  gzRobotCmdTopicName: /gazebo/default/fanuc/robotcmd 
	  gzRobotStatusTopicName: /gazebo/default/fanuc/robotstatus 
	  gzGripperCmdTopicName: /gazebo/default/gripper/fanuc_lrmate200id/control 
	  gzGripperStatusTopicName: /gazebo/default/gripper/fanuc_lrmate200id/state 




The model subbranch defines kitting model related parameters. In theory this subbranch is not necessary as a higher level module would generate command and control of models. However, for configuratin various kitting tests, the model subbranch tweaks the diagnostics and/or improves efficancy.



	model: 
		# kitting parts within the reach of the robot 
	 	 parts: [sku_kit_m2l1_vessel14,sku_kit_m2l1_vessel15,sku_medium_gear_vessel16,sku_part_medium_gear17,sku_part_medium_gear18,sku_part_medium_gear19,sku_part_medium_gear20,sku_large_gear_vessel21,sku_part_large_gear22,sku_part_large_gear23] 
		# debug model subranch 
	  	debug:  
			# log model CRCl diagnostics related to model basic information 
	    		model: 0 
			# log model CRCl diagnostics related to model inferences 
	    		inferences: 0 




The app subbranch defines the ROS node application parameters.



	app: 
		# run threads (1) or as single sequence (0) 
	bSingleThread: 1 
	# enable command line interface for input 
	bCmdLineInterface: 0 
	# enable (1) Crcl streaming as defined in crcl.yaml 
	bCrclStreaming: 1 
	# moveit subbranch 
	moveit:  
		# use moveit it or simple s curve 
		use: 0 
		# moveit end effector max distance 
			eef_step: 0.01 
			# moveit jump threshold between robot configurations 
			jump_threshold: 0.0 
			#  
			avoid_collisions: 0 
	debug:  
		# log full demo test diagnostics 
			demo: 1 
			# log pre/post diagnostics of transform operations 
			transforms: 0 
		closest: 
			free: 0 
			openslot: 0 
		dwell: 
			# duration of dwell after motion 
			time: 1.0 
			# duration of dwell after grasping operation 
			grasping: 2.0 
		# the application has various progressively invovled test modes 
		test: 
			# test full model inference-based kitting 
			bDemoTest: 1 
			# test raw move/grasp of gazebo and moveit trajectories 
			bRawTest: 0 
			# verify that model data from Gazebo is reported correctly 
			bModelTest: 0 
			# Test high level api pre demo test to verify correct threading 
			bHighLevelCrclApiTest: 0 

The three major rosparm definitions of concern:


1. are the base transform from world coordinate frame to robot origin, which HAS to correspond between the world coordinate origin and the robot frame. Often the transformation is embedded inside of URDF so it can be tricky to determine the correct base transform pose. For SDF users it is just the xyzrpy definition for the robot model. If you ar using gazebo_ros_api plugin to launch 


2. qbend determines the transform from the end effector in the home URDF position to pointing down in the z direction. For robots pointing straight up when the joints are all zero the bend quaternion is ( 1,0,0,0) assuming (x,y,z,w) representation. This is basically pointing in the -z direction (or opposite of (0,0,0,1) which is pointing is the positive Z direction.) For the Fanuc which points out along the X axis,  the bend quaternion is (0,0.707107,0,0.707107) or pointing 90o down in the X axis (?). 


3. Finally, the tool length (x,y,z) and orientation (rpy) must be correct as for every approach and grasp the end-effector is "subtracted" off, the trajectory is computed using Ik to give joint values. The end effector is essentially "added" back into the transformation for each point in the trajectory since the 0T6 transformation plus the end effector provides the correct final pose.



	xform: 
		# base transform from world coordinate frame to robot origin		 
	  	base: 	[-0.169, -1.140, 0.934191,  0.0,0.0,0.0] 
	  	# Quaternion for robot orientation to point down at kitting gears 
	  	qbend: 	[0,0.707107,0,0.707107] 
		# transform from final robot transform (e.g., T6) to end effector coordinate 
	  	tool: 	[0.0, 0.0, .182, 0.0,0.0,0.0] 


