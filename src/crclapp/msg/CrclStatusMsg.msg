# CrclStatusMsg.msg
# Crcl status contains information for canonical robot command language exchange.
# CRCL is expressed in XML.
# At a primite level, a CRCL message is a framed xml string.
# So, the ROS status message contains a xml string and the originating ip from where the CRCL message came.
# https://github.com/ros/common_msgs 


std_msgs/Header header

uint8 crclcommandnum
uint8 crclstatusnum
uint8 crclcommandstatus
################
uint8 done=0
uint8 error=1
uint8 working=2
################

geometry_msgs/Pose  statuspose
sensor_msgs/JointState statusjoints
float64 eepercent

