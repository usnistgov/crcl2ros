#!/bin/bash

#    ^^ this has to be bash, not /bin/sh, for arrays to work
# run dos2unix ./runmultiterm.bash

#    ^^ this has to be bash, not /bin/sh, for arrays to work
# run dos2unix ./runmultiterm.bash
root=`pwd`/../../../..
export root


aprs=$root/Worlds/aprs-lab.world
gzver=7
export gzver
export aprs

p=`pwd`
export p
q=`pwd`/launch
r=$root/install/lib/gzrosrcs


# Definition of environment variables for bash
models=$root/gzdatabase/models
export models

plugins=$root/plugins$gzver
export plugins


libs=$root/install/lib
export libs


gzrosrcs=$root/install/lib/gzrosrcs
export gzrosrcs

crclapp=$root/install/lib/crclapp
export crclapp


aprs_objects=$root/install/lib/aprs_objects
export aprs_objects


 
cmd=( gnome-terminal )

cmd+=( --tab  --working-directory="$r" -e 'bash -c "printf \"\e]2;crclapp\a\";export LD_LIBRARY_PATH=/opt/ros/kinetic/lib:$libs:$LD_LIBRARY_PATH; echo $LD_LIBRARY_PATH; source /opt/ros/kinetic/setup.bash; source $root/devel/setup.bash; cd $crclapp; ./crclapp -r fanuc_;exec bash"')


#cmd+=( --tab  --working-directory="$q" -e 'bash -c "printf \"\e]2;CRCL Status\a\";source /opt/ros/kinetic/setup.bash;source $HOME/src/gzaprsros-xenial/devel/setup.bash;source $root/devel/setup.bash; cd $p/Python;python  cannedcrclclient.py ;exec bash"')


cmd+=( --tab  --working-directory="$q" -e 'bash -c "printf \"\e]2;ROS TOPIC\a\";source /opt/ros/kinetic/setup.bash;source $HOME/src/gzaprsros-xenial/devel/setup.bash;source $root/devel/setup.bash; rostopic list;exec bash"')



"${cmd[@]}"

