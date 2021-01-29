# Gripper Testing

This package contains a gazebo plugin for controlling a parallel gripper in gazebo simulation.

## Requirements

Gazebo 9 + SDK
For QT: Gazebo ignition math 4, ignition msgs1, ignition transport4 and sdformat-6.2 Or modify the qmake pro file.
Only tested with ROS kinetic and Ubuntu 16.04 LTS


## Build

To build, either 
a) compile top level with ROS "catkin_make install" and dlls will be copied to plugins9
b) invoke Qt 5.9.1 IDE and compile. 



## Contents

### Gazebo Plugins

The gzparallelgripper (or ParallelGripper) plugin is used to monitor the pose error of a grasped object while the gripper is subject to various user-defined forces. The plugin can be used with any parallel gripper. The plugin is not purely physics based but is a very efficient implementation of a kludge.  The plugin attaches the grasped object by detecting when both fingers collide with the object and helps to avoid problems with physics engines (e.g., alternating collision reporting) and to help the object staying in the robot gripper without slipping out during motion travel.

The parallel gripper fingertips are actuated using the custom protobuf Gazebo GripCommand command defining the state of the gripper (enabled/disabled). Enabled means closed. The plugin uses a simplified control scheme: a constant user-defined force is applied through both finger joints with a proportional corrective force used to maintain the symmetrical position of the gripper's fingertips. This scheme was used to better reflect the functionality of actual pneumatic grippers.

An object is detected as "grasped" as soon as two opposing forces are applied by the gripper links on an object and the plugin detects a collision between the grasping fingers and the grasped object.. Then a virtual joints is created attaching the object to each finger join. As soon as the grasp command to release is issued (e.g. the gripper opens), the object virutal joints are removed so that the object is detached.


### Gazebo World
The gazebo sdf world includes the gzparallelgripper as plugins for both the motoman sia20d and the fanuc 200id. The control and status topic names are different for each robot.

## Description of the arguments:

    <grip_force_close> contains the desired force to apply to gripper closing, default 10N
    <grip_force_open> contains the desired force to apply to gripper opening, default 1/2 of closing force.
    <grip_kp> describes the p of pid for force calculation per cycle
    <joint1> contains the name of the finger joint 1
    <joint2> contains the name of the finger joint 2
    <control_topic> contains the gazebo command topic name, default ~/gripper/control
    <state_topic> contains the gazebo s topic name, default ~/gripper/state
    <debug> 0 turns off debugging print statements, 1 turns on
    <collisions> 0 turns off collision detection for attaching gripper fingers to object, 1 turns on
   
### Example
Note the finger links are deduced from the joint names. Further you must fully instantiate the joint names. 
The joint names are fanuc_prism1 and fanuc_prism2, but are part of the fanuc_lrmated200id robot and part of the lrmate model..
So the fully qualified name for joint1 would be lrmate::fanuc_lrmate200id::fanuc_prism1 shown below:

	<model name="lrmate">
            <include>
                <static>false</static>
                <uri>model://fanuc_lrmate200id</uri>
                <pose>-0.169 -1.140 0.934191 0 0 0</pose>
            </include>
            . . .
	    <plugin name="ParallelGripperPlugin" filename="libgzparallelgripperplugin.so">
		<grip_force_close>5</grip_force_close>
		<joint1>fanuc_lrmate200id::fanuc_prism1</joint1>
		<joint2>fanuc_lrmate200id::fanuc_prism2</joint2>
		<grip_kp>10000</grip_kp>
		<control_topic> ~/gripper/fanuc_lrmate200id/control </control_topic>
		<state_topic> ~/gripper/fanuc_lrmate200id/state </state_topic>
		<debug> 0 </debug>
		<collisions> 1 </collisions>
	    </plugin>
	</model>
