#!/bin/bash

# assumes you did source devel/setup.bash

source /opt/ros/indigo/setup.bash
source /usr/local/michalos/nistcrcl_ws/devel/setup.bash
crcl="run"

cmd=( gnome-terminal )
# This roslaunch will load a robot description so there are actual joint names. Otherwise no joint names!
# It also loads the controller_joint_names yaml file
cmd+=( --tab-with-profile=Default --title="roslaunch" -e "/opt/ros/indigo/bin/roslaunch testcrcl simplelaunch.launch " )
#cmd+=( --tab-with-profile=Default --title="roscore" -e "/opt/ros/indigo/bin/roscore " )

# This is the ROS package to translate CRCL to/from ROS message topics
# For debugging, netbeans is used as a IDE
if [ "$crcl" = "run" ]
then
cmd+=( --tab-with-profile=Default --title="nistcrcl" -e "/usr/local/michalos/nistcrcl_ws/devel/lib/nistcrcl/nistcrcl " )
fi

# This tab just echos the crcl command that is being published
#cmd+=( --tab-with-profile=Default --title="rostopic echo"  -e "/opt/ros/indigo/bin/rostopic echo crcl_command" )

# This canned python script will keep attempting to connect until connected, then will move joint 0 +90 to -90 even 6 seconds
cmd+=( --tab-with-profile=Default --title="python canned test"  -e 'python /usr/local/michalos/nistcrcl_ws/src/testcrcl/scripts/cannedcrclclient.py' )

#cmd+=( --tab-with-profile=Default --title="python canned test"  -e 'python /usr/local/michalos/nistcrcl_ws/src/testcrcl/scripts/crcltest.py' )

# This python program will read ROS crcl command and echo status
cmd+=( --tab-with-profile=Default --title="python feedback test"  -e 'python /usr/local/michalos/nistcrcl_ws/src/testcrcl/nodes/crclfeedbacktest.py' )

"${cmd[@]}"
