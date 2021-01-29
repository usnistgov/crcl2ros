
#include <string>
#include <sys/stat.h>
#include <fcntl.h>
#include <iostream>
#include <sstream>


#include "rcs/Debug.h"
#include "rcs/RCSThreadTemplate.h"
#include "rcs/Conversions.h"

#include "crclapp/CrclWm.h"
#include "crclapp/Globals.h"
#include "crclapp/Ros.h"
#include "crclapp/CommandLineInterface.h"
#include "crclapp/CrclPrimitives.h"
#include "crclapp/CrclSocketServer.h"
#include "crclapp/RosMsgHandler.h"
#include "crclapp/CrclRobotImpl.h"

using namespace RCS;
rcs_robot_type rcs_robot;
rcs_world_type rcs_world;
rcs_status_type rcs_status;
int rcs_robot_type::_crclcommandnum = 1;

namespace WorldModel
{
WorldModel::CInstances instances;
std::mutex shapemutex;

}
namespace crcl
{
 std::mutex mutex;
 std::string crclIp;
 int crclPort;
 int dbgCrclCommand;
 int dbgCrclStatus;
 int dbgCrclStatusMsg;
 int dbgCrclCommandMsg;
 int bCrclStopIgnore;
 int bFlywheel;
 int bProcessAllCrclMessages;
 int bDebug;
 int bAutoCrclStatus;
 int bSynchronousCmds;

 std::shared_ptr<crcl::CSocketSession> crclCommServer;
 std::shared_ptr<crcl::CCrcl2RosMsgApi>  crclApi;
 std::shared_ptr<crcl::CRosCmdStatusHandler> rosCmdStatMsgHandler;
 std::shared_ptr<crcl::CRosCmdMsgHandler> rosCmdMsgHandler;
 std::shared_ptr<crcl::CRobotImpl> crclRobotImpl;

 /**
  * @brief configure
  * @param nh node handle for ROS
  * @param ns namespace used for this crclapp defining robot
  */
 void configure(ros::NodeHandle *nh, std::string ns)
 {
     // ROS Param


     if(!nh->getParam(Globals.ns + "/crcl/Ip", crclIp))
         ROS_ERROR_STREAM("Missing ROS param" << Globals.ns  << "/crcl/Ip");
     if(!nh->getParam(Globals.ns + "/crcl/Port", crclPort))
         ROS_ERROR_STREAM("Missing ROS param" << Globals.ns  << "/crcl/Port");
     if(!nh->getParam(Globals.ns + "/crcl/Debug/General", bDebug))
         ROS_ERROR_STREAM("Missing ROS param" << Globals.ns  << "/crcl/Debug/General");
     if(! nh->getParam(Globals.ns + "/crcl/AutoStatus", bAutoCrclStatus))
         ROS_ERROR_STREAM("Missing ROS param" << Globals.ns  << "/crcl/AutoStatus");

     if(!nh->getParam(Globals.ns + "/crcl/length_units", CRosCmdStatusHandler::length_units))
        ROS_ERROR_STREAM("Missing ROS param" << Globals.ns  << "/crcl/length_units");
     if(!nh->getParam(Globals.ns + "/crcl/angle_units", CRosCmdStatusHandler::angle_units ))
        ROS_ERROR_STREAM("Missing ROS param" << Globals.ns  << "/crcl/angle_units");
     if(!nh->getParam(Globals.ns + "/crcl/DebugStatusMsg", crcl::dbgCrclStatusMsg))
        ROS_ERROR_STREAM("Missing ROS param" << Globals.ns  << "/crcl/DebugStatusMsg");
     if(!nh->getParam(Globals.ns + "/crcl/DebugCommandMsg", crcl::dbgCrclCommandMsg))
        ROS_ERROR_STREAM("Missing ROS param" << Globals.ns  << "/crcl/DebugCommandMsg");
     if(!nh->getParam(Globals.ns + "/crcl/StopIgnore", crcl::bCrclStopIgnore))
        ROS_ERROR_STREAM("Missing ROS param" << Globals.ns  << "/crcl/Ip");
     if(!nh->getParam(Globals.ns + "/crcl/DebugXML", crcl::CSocketSession::dbgCrclXML))
        ROS_ERROR_STREAM("Missing ROS param" << Globals.ns  << "/crcl/DebugXML");
     if(!nh->getParam(Globals.ns + "/crcl/flywheel", crcl::bFlywheel ))
        ROS_ERROR_STREAM("Missing ROS param" << Globals.ns  << "/crcl/flywheel");
     if(!nh->getParam(Globals.ns + "/crcl/processAllCrclMessages", crcl::bProcessAllCrclMessages))
        ROS_ERROR_STREAM("Missing ROS param" << Globals.ns  << "/crcl/processAllCrclMessages");
}
 void printConfiguration()
 {
     ROS_DEBUG( " crcl robot= %s", rcs_robot.robotName.c_str());
     ROS_DEBUG( " crcl Ip= %s", crcl::crclIp.c_str());
     ROS_DEBUG( " crcl Port= %d", crcl::crclPort);
     ROS_DEBUG( " crcl Debug general= %d", crcl::bDebug);
     ROS_DEBUG( " crcl Debug bAutoCrclStatus= %d", crcl::bAutoCrclStatus);
     ROS_DEBUG( " crcl bDebugCrclStatusMsg= %d", crcl::dbgCrclStatusMsg);
     ROS_DEBUG( " crcl bDebugCrclCommandMsg= %d", crcl::dbgCrclCommandMsg);
     ROS_DEBUG( " crcl bCrclStopIgnore= %d", crcl::bCrclStopIgnore);
     ROS_DEBUG( " crcl bDebugCrclXML in socket server= %d", crcl::CSocketSession::dbgCrclXML);
     ROS_DEBUG( " crcl bFlywheel= %d", crcl::bFlywheel);
     ROS_DEBUG( " crcl bProcessAllCrclMessages= %d", crcl::bProcessAllCrclMessages);
     ROS_DEBUG( " crcl socket reader Ip=%s Port=%d",   crcl::crclIp.c_str(), crcl::crclPort);
 }

}
std::shared_ptr<CRosCrclClient> rosCrclClient;



double rcs_status_type::currentRobotJointSpeed()
{
    // FIXME: this is wrong
    return crcl::crclApi->rates.CurrentTransSpeed();
}
double rcs_status_type::currentGripperJointSpeed()
{
    // FIXME: this is wrong
    return crcl::crclApi->rates.CurrentTransSpeed();
}
double rcs_status_type::currentLinearSpeed()
{
    // FIXME: this is wrong
    return crcl::crclApi->rates.CurrentTransSpeed();
}
double rcs_status_type::currentAngularSpeed()
{
    // FIXME: this is wrong
    return crcl::crclApi->rates.CurrentRotSpeed();
}

////////////////////////////////////////////////////////////////////////////////

void rcs_world_type::configure(ros::NodeHandle *nh, std::string ns)
{
    // list of parts (i.e., gears, trays, kits) in this robot world model
    if(!nh->hasParam(Globals.ns + "/model/parts"))
        ROS_ERROR_STREAM("Missing ROS param" << Globals.ns  << "/model/parts");
    nh->getParam(Globals.ns + "/model/parts", part_list);

    // z upwards offset of vessel slot
    std::vector<double> offset;
    if(!nh->hasParam(Globals.ns + "/offset/vesselslot"))
        ROS_ERROR_STREAM("Missing ROS param" << Globals.ns  << "/offset/vesselslot");
    nh->getParam(Globals.ns  + "/offset/vesselslot", offset);
    slotoffset["vessel"]=RCS::Convert<std::vector<double>, tf::Pose> (offset);

    if(!nh->hasParam(Globals.ns + "/model/debug/inferences"))
        ROS_ERROR_STREAM("Missing ROS param" << Globals.ns  << "/model/debug/inferences");
    nh->getParam(Globals.ns + "/model/debug/inferences", Globals.dbgModelInferences);

    if(!nh->hasParam(Globals.ns + "/model/debug/model"))
        ROS_ERROR_STREAM("Missing ROS param" << Globals.ns  << "/model/debug/model");
    nh->getParam(Globals.ns + "/model/debug/model", Globals.dbgModelStatus);

    // This initializes the shapes in the robot kitting model: supply trays, kits, gears, slots
    WorldModel::CShapes::initDefinitions();

}
 void rcs_world_type::printConfiguration()
 {
     ROS_DEBUG( "Kitting parts= %d",  vectorDump<std::string>(part_list,","));
     ROS_DEBUG( "Kitting model/debug/inferences= %d",  Globals.dbgModelInferences);
     ROS_DEBUG( "Kitting model/debug/model= %d",  Globals.dbgModelStatus);
     // FIXME: add print of slotoffsets

 }

////////////////////////////////////////////////////////////////////////////////

void rcs_robot_type::configure(std::string robot, ros::NodeHandle * nh, std::string ns)
{
    //_prefix=robot;

    try {
        bool bParam=1;

        /// ROBOT configuration
        ////////////////////////////////////////////
        bParam&=nh->getParam(Globals.ns + "/nc/tiplink", robotTiplink);
        bParam&=nh->getParam(Globals.ns + "/nc/baselink", robotBaselink);
        // Read the robot urdf file
        bParam&=nh->getParam(Globals.ns + "/robot_description", robot_urdf);
        if(!bParam)
            ROS_ERROR( "rcs_robot_type::configure getParam failed");

        parseURDF();

        // FIXME:
        // add moveit assistant macros joint pose moves

        std::vector<double> dbase;
        std::vector<double> dbend;
        nh->getParam(Globals.ns  + "/xform/base", dbase);
        nh->getParam(Globals.ns  + "/xform/qbend", dbend);

        // Translate 4 doubles into quaternion
        if(dbase.size() < 6  || dbend.size() < 4)
        {
            throw std::runtime_error(std::string( "dbase or dbend missing values"));

        }
        basePose=RCS::Convert<std::vector<double>, tf::Pose> (dbase);
        basePose = tf::Pose( tf::Quaternion( 0.0, 0.0, 0.0, 1.0),tf::Vector3(-0.169, -1.140, 0.934191));
        basePoseInverse=basePose.inverse();

        QBend = tf::Quaternion(dbend[0], dbend[1], dbend[2], dbend[3]);
        QBend= tf::Quaternion( 0,0.707107,0,0.707107);

        std::vector<double> dretract;
        nh->getParam(Globals.ns  + "/xform/retract", dretract);
        Retract =  RCS::Convert<std::vector<double>, tf::Pose>(dretract);
        Retract = tf::Pose(QBend, tf::Vector3(0.0, 0.0, 0.04));
        RetractInv=Retract.inverse();

        std::vector<double> deeoffset;
        nh->getParam(Globals.ns  + "/xform/gripper", deeoffset);
        Gripper = RCS::Convert<std::vector<double>, tf::Pose>(deeoffset);
        Gripper = tf::Pose( tf::Quaternion(  0.0,0.0,0.0, 1.0),
                                tf::Vector3( 0.0, 0.0,-.182));
                                //tf::Vector3( .182,0.0, 0.0));

        GripperInv= Gripper.inverse();

        /// Determine if we are using multi-robot URDF or
        /// multi-robot SDF.
        /// URDF has robots offset from 0,0,0 world coordinates.
        /// SDF gazebo has robots with local cooredinate space
        /// If robotxyz param not found, assume robot in world origin (0,0,0)
        ////////////////////////////////////
        std::string _param(Globals.ns+"/xform/base");
        std::string _value;
        if (!ros::param::has(_param))
        {
            std::cerr << "Couldn't find param " << _param << "\n";
            Globals.bRobotInWorldCoordinates=false;
        }
        else
        {
            CRos::nh->getParam(_param, _value);
            double x,y,z;
            int n = sscanf(_value.c_str(), "-x %lf -y %lf -z %lf", &x,&y,&z);
            if(n==3 && x==0.0 && y ==0.0 && z==0.0)
                Globals.bRobotInWorldCoordinates=true;
            else
                Globals.bRobotInWorldCoordinates=false;
        }


        /// Part offsets
        ////////////////////////////////////////////
        std::vector<double> offset;
        nh->getParam(Globals.ns + "/offset/smallgear", offset);
        gripperoffset["sku_part_small_gear"]=RCS::Convert<std::vector<double>, tf::Pose> (offset);
        nh->getParam(Globals.ns + "/offset/mediumgear", offset);
        gripperoffset["sku_part_medium_gear"]=RCS::Convert<std::vector<double>, tf::Pose> (offset);
        nh->getParam(Globals.ns + "/offset/largegear", offset);
        gripperoffset["sku_part_large_gear"]=RCS::Convert<std::vector<double>, tf::Pose> (offset);
    }
    catch(...)
    {
        ROS_FATAL( "Fatal error in rcs_robot_type::configure");
    }

}

void rcs_robot_type::printConfiguration()
{
    ROS_DEBUG_STREAM( "URDF Robot " << this->robotName.c_str());
    ROS_DEBUG_STREAM( "base link= " << this->robotBaselink);
    ROS_DEBUG_STREAM( "ee link= " <<this->robotTiplink);
    ROS_DEBUG_STREAM( "num joints= " << numJoints());
    ROS_DEBUG_STREAM( "baseoffset= " << dumpPoseSimple(this->basePose).c_str());
    ROS_DEBUG_STREAM( "tooloffset= " << dumpPoseSimple(this->Gripper).c_str());
    ROS_DEBUG_STREAM( "qbend= " << dumpQuaterion(this->QBend).c_str());
    ROS_DEBUG_STREAM( "Joint names= " << vectorDump<std::string>(this->jointNames).c_str() );
    ROS_DEBUG_STREAM( "Robot World Coordinates= " << Globals.bRobotInWorldCoordinates );

}

bool rcs_robot_type::isBusy(int & cmdnum)
{
//    if(_status == NULL)
//        return true;
    bool bFlag;
    if(cmdnum<0)
        bFlag = false;
    else if(cmdnum > _status.crclcommandnum)
        bFlag = true;
    else if(cmdnum != _status.crclcommandnum)
        bFlag = true;
    else
        bFlag =   _status.crclcommandstatus!=crcl::CRCL_Done;
    return bFlag;
}


size_t rcs_robot_type::numJoints()
{
    size_t n = jointNames.size();
    return n;
}

std::vector<unsigned long> rcs_robot_type::allJointNumbers()
{
    std::vector<unsigned long> jointnum(numJoints());
    std::iota(jointnum.begin(), jointnum.end(), 0); // adjusted already to 0..n-1
    return jointnum;
}

bool rcs_robot_type::parseURDF()
{
    urdf::Model robot_model;
    if(robot_urdf.empty())
    {
        ROS_FATAL(" rcs_robot_type::parseURDF() empty robot description");
        return false;
    }

    if(!robot_model.initString(robot_urdf))
    {
        ROS_FATAL(" robot_model.initString(robot_urdf) FAILED");
    }

    robotName=robot_model.getName();

    // These vectdors are cleared in case parseURDF is called twice...
    linkNames.clear();
    jointNames.clear();
    axis.clear();
    xyzorigin.clear();
    rpyorigin.clear();
    jointHasLimits.clear();
    jointMin.clear();
    jointMax.clear();
    jointEffort.clear();
    jointVelmax.clear();

    urdf::LinkConstSharedPtr link =robot_model.getLink(robotTiplink);
    while (link->name != robotBaselink) { // && joint_names.size() <= num_joints_) {
        linkNames.push_back(link->name);
        urdf::JointSharedPtr joint   = link->parent_joint;
        if (joint) {
            if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {

                jointNames.push_back(joint->name);
                axis.push_back(Convert<urdf::Vector3, tf::Vector3>(joint->axis));
                xyzorigin.push_back(Convert<urdf::Vector3, tf::Vector3>(joint->parent_to_joint_origin_transform.position));
                double roll, pitch, yaw;
                joint->parent_to_joint_origin_transform.rotation.getRPY(roll, pitch, yaw);
                rpyorigin.push_back(tf::Vector3(roll, pitch, yaw));

                float lower, upper, maxvel = 0.0, maxeffort = 0.0;
                int hasLimits;
                if (joint->type != urdf::Joint::CONTINUOUS) {
                    maxvel = joint->limits->velocity;
                    maxeffort = joint->limits->effort;
                    if (joint->safety) {
                        lower = joint->safety->soft_lower_limit;
                        upper = joint->safety->soft_upper_limit;
                    } else {
                        lower = joint->limits->lower;
                        upper = joint->limits->upper;
                    }
                    hasLimits = 1;
                } else {
                    lower = -M_PI;
                    upper = M_PI;
                    hasLimits = 0;
                }
                if (hasLimits) {
                    jointHasLimits.push_back(true);
                    jointMin.push_back(lower);
                    jointMax.push_back(upper);
                } else {
                    jointHasLimits.push_back(false);
                    jointMin.push_back(-M_PI);
                    jointMax.push_back(M_PI);
                }
                jointEffort.push_back(maxeffort);
                jointVelmax.push_back(maxvel);
            }
        } else {
            ROS_WARN_NAMED("nc", "no joint corresponding to %s", link->name.c_str());
        }
        link = link->getParent();
    }

    std::reverse(linkNames.begin(), linkNames.end());
    std::reverse(jointNames.begin(), jointNames.end());
    std::reverse(jointMin.begin(), jointMin.end());
    std::reverse(jointMax.begin(), jointMax.end());
    std::reverse(jointHasLimits.begin(), jointHasLimits.end());
    std::reverse(axis.begin(), axis.end());
    std::reverse(xyzorigin.begin(), xyzorigin.end());
    std::reverse(rpyorigin.begin(), rpyorigin.end());
    std::reverse(jointEffort.begin(), jointEffort.end());
    std::reverse(jointVelmax.begin(), jointVelmax.end());

    return true;
}
