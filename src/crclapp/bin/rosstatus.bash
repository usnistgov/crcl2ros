#!/bin/bash

 
source /opt/ros/kinetic/setup.bash
source $HOME/src/gzaprsros-xenial/devel/setup.bash
rostopic list
#rostopic pub /fanuc_crcl_status geometry_msgs/PoseMsg "[position: 0.0,0.0,0.0,  orientation: 0.0,0.0,0.0,1.0]"

rostopic pub /fanuc_crcl_status crcl_rosmsgs/CrclStatusMsg "
header:  
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
crclcommandnum: 3
crclstatusnum: 2
crclcommandstatus: 2
statuspose:
  position:
    x: 0.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
statusjoints:
  header: auto
  name: []
  position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  velocity: []
  effort: []
eepercent: 1.0"



