# Crcl command contains information for canonical robot command language exchange.
# CRCL is expressed in XML.
# At a primite level, a CRCL message is a framed xml string.
# So, the ROS message contains a xml string and the originating ip from where the CRCL message came.
# 

std_msgs/Header header

# enumeration of CRCL commands
uint8 crclcommand
########################################
uint8 initCanon=1
uint8 endCanon=2
uint8 actuatejoints=3
uint8 moveto=4
uint8 dwell=5
uint8 message=6
uint8 moveThroughTo=7
uint8 setCoordinatedMotion=8
uint8 stopMotion=9
uint8 setEndEffector=10
uint8 openToolChange=11
uint8 closeToolChanger=12
uint8 drawObject = 13
uint8 eraseObject = 14
uint8 setGripperPose = 15
uint8 pick = 16
uint8 place = 17
########################################
uint64 crclcommandnum

# https://github.com/ros/common_msgs 
geometry_msgs/Pose  finalpose
geometry_msgs/Pose[] waypoints
# Below joint info could be  trajectory_msgs/JointTrajectoryPoint
sensor_msgs/JointState joints
uint64[] jointnum
float64[] hint
bool bStraight
float64   dwell_seconds
string opmessage
string partname
string partfeature
std_msgs/ColorRGBA  partcolor
bool bCoordinated
float64 eepercent
CrclMaxProfileMsg[] profile # maximum profile 
string[] parameter_names
string[] parameter_values


