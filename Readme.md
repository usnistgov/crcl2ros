






# <a name="Contents"></a>Contents


The repository contains a workspace for ROS to handle CRCL streaming, CRCL to ROS conversion, and ROS to Gazebo simulation.  This repository contains C++ code that implements CRCL XML streaming and parsing component, that maps command and status motion primitives from CRCL to ROS,  then uses ROS moveit! to plan motion trajectories that are then simulated in Gazebo.



Here are some notes regarding the implementation:



- Only the Fanuc LRMate 200id has been thoroughly tested. There is a folder for Motoman SIA 20d, but has not been fully tested.



- The CRCL has been extended to provide Gazebo or Sensor (e.g., vision) feedback regarding model information. Specifically, the name and pose of a world model object (e.g., kitting gear or kitting tray). In addition, inferences about the kitting models is performed and included in the extended CRCL status model report as properties.  Such inference knowledge includes information about the tray slots, and the state the tray slot - empty or filled with gear.



- There is no Rviz. 



- roslaunch  files were merged and stripped of unused components so as to understand why multiple robot namespaces caused issues.



- The gazebo_ros_api plugin is loaded upon launch of gazebo 



- A lot of effort went into allowing multiple robots with separate URDF. ROS makes it rather straightforward to bundle multiple robots into one URDF (for instance like a 2 armed robot with a base) and then each arm is controlled in world coordinate space. In the end, existing SDF was used as the Gazebo world and identical URDF with a qualified namespace was used as ROS param.








# <a name="Status"></a>Status


The packages in this repository are currently NIST supported. CRCL is itself a standard which can be found [<u>here</u>](https://github.com/ros-industrial/crcl). The repository works for a Fanuc (which does not support ROS) robot  that satisfies the following requirements. 



## <a name="Requirements"></a>Requirements






- Ubuntu 16.04 Xenial



- Gazebo 7 (to allow gazebo_ros_api packages)



- ROS  1 Kinetic



- CodeSynthesis and Xerces XML tools



- Gnu C++ compiler



- Moveit!




# <a name="Installation"></a>Installation


Clone the crcl2ros repository from the github repository. You may need to install the following packages if you have not done so already:



- ros-kinetic-desktop-full



- gazebo7



- libgazebo7-dev



- gazebo-ros-pkgs



- moveit-full



You will need Xerces C library installed, as the catkin uses it to link against for CodeSynthesis. The CodeSynthesis xsd compilation of the CRCL xsd has already been done and the cpp/h files are included in the repository.  See the wiki folder codesynthesis for further details.



If a missing ROS dependency causes the build to fail, run rosdep.



## <a name="Building"></a>Building


Change to the crcl2ros main subfolder to build.




	$ cd $HOME 
	 
	# retrieve the latest development version.  
	$ git clone -b master https://github.com/usnistgov/crcl2ros.git  
	 
	# change folder 
	$ cd crcl2ros 
	 
	# build the workspace (using catkin_tools) 
	$ catkin build -DCMAKE_BUILD_TYPE=Debug 





It is suggested to use a Debug compilation so you can then attach to the running crclapp in Qt IDE to debug. There is a QT pro file to assist in debugging the ROS C++ source code. In the crcl2ros wiki folder is a description on how to debug the crclapp using QT IDE under FAQ.



## <a name="Running"></a>Running



	$ source devel/setup.bash 
	$ roslaunch fanuc_lrmate200id_support  top.launch 





## <a name="Configuring"></a>Configuring


The configuration for the crcl2ros workspace is defined in the Wiki under the config folder.




# <a name="Disclaimer"></a>Disclaimer


This software was produced by the National Institute of Standards and Technology (NIST), an agency of the U.S. government, and by statute is not subject to copyright in the United States.  Recipients of this software assume all responsibility associated with its operation, modification, maintenance, and subsequent redistribution.



See NIST Administration Manual 4.09.07 b and Appendix I.









