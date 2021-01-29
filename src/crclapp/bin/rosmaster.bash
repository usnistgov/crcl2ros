#!/bin/bash

#    ^^ this has to be bash, not /bin/sh, for arrays to work
# run dos2unix ./runmultiterm.bash

 
cmd=( gnome-terminal )
E
cmd+=( --tab  --working-directory="$q" -e 'bash -c "printf \"\e]2;ROSMASTER\a\";source /opt/ros/kinetic/setup.bash;roscore;exec bash"')

cmd+=( --tab  --working-directory="$q" -e 'bash -c "printf \"\e]2;ROS TOPIC\a\";source /opt/ros/kinetic/setup.bash;source $HOME/src/gzaprsros-xenial/devel/setup.bash;sleep 20;rostopic list;exec bash"')


"${cmd[@]}"

