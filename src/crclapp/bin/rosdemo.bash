#!/bin/bash

#    ^^ this has to be bash, not /bin/sh, for arrays to work
# run dos2unix ./runmultiterm.bash

 
cmd=( gnome-terminal )

cmd+=( --tab  --working-directory="$q" -e 'bash -c "printf \"\e]2;ROSBRIDGE\a\";source /opt/ros/kinetic/setup.bash;roslaunch rosbridge_server rosbridge_websocket.launch;exec bash"')

cmd+=( --tab  --working-directory="$q" -e 'bash -c "printf \"\e]2;ROS TOPIC\a\";source /opt/ros/kinetic/setup.bash;sleep 20;rostopic list;rostopic echo java_to_ros;exec bash"')


"${cmd[@]}"

