/*
DISCLAIMER:
This software was produced by the National Institute of Standards
and Technology (NIST), an agency of the U.S. government, and by statute is
not subject to copyright in the United States.  Recipients of this software
assume all responsibility associated with its operation, modification,
maintenance, and subsequent redistribution.

See NIST Administration Manual 4.09.07 b and Appendix I.
 */


#include "crclapp/Crcl2RosMsgApi.h"
#include "crclapp/Globals.h"
#include "crclapp/RosMsgHandler.h"
#include "crclapp/CrclWm.h"

#include "rcs/Conversions.h"
using namespace RCS;
using namespace crcl;

// Simplistic Testing code
IRate CCrcl2RosMsgApi::rates;

////////////////////////////////////////////////////////////////////////////////
void CCrcl2RosMsgApi::setDwell(double d)
{
    _mydwell=d;
}
////////////////////////////////////////////////////////////////////////////////
void CCrcl2RosMsgApi::slow()
{
    rates.CurrentTransSpeed() = rates.CurrentTransSpeed()/2.;
    rates.MaxJointVelocity() = rates.MaxJointVelocity() / 2.0;
}
////////////////////////////////////////////////////////////////////////////////
void CCrcl2RosMsgApi::medium()
{
    setVelocity(1.0);
}
////////////////////////////////////////////////////////////////////////////////
void CCrcl2RosMsgApi::fast()
{
    rates.CurrentTransSpeed()= rates.CurrentTransSpeed()*2.;
    rates.MaxJointVelocity() = rates.MaxJointVelocity() * 2.0;
}

////////////////////////////////////////////////////////////////////////////////
void CCrcl2RosMsgApi::setGraspingDwell(double d)
{
    _mygraspdwell=d;
}
////////////////////////////////////////////////////////////////////////////////
void CCrcl2RosMsgApi::setVelocity(double speed)
{
    rates.CurrentTransSpeed()=speed;
    rates.MaxJointVelocity() = speed;
}

////////////////////////////////////////////////////////////////////////////////
int CCrcl2RosMsgApi::setGripper(double ee)
{

    RCS::CCanonCmd cmd;
    cmd.crclcommand = CanonCmdType::CANON_SET_GRIPPER;
    cmd.crclcommandnum = ++rcs_robot._crclcommandnum;
    cmd.eepercent = ee;
    RCS::cmds.addMsgQueue(cmd);
    return cmd.crclcommand;

}

////////////////////////////////////////////////////////////////////////////////
void CCrcl2RosMsgApi::setVelGripper(double vel, double fmax)
{

    RCS::CCanonCmd cmd;
    cmd.crclcommand = CanonCmdType::CANON_SET_EE_PARAMETERS;
    cmd.crclcommandnum = ++rcs_robot._crclcommandnum;
    cmd.parameter_names = { "action", "vel", "fmax"};
    cmd.parameter_values = { "Vel/Fmax", Globals.strConvert(vel), Globals.strConvert(fmax)};;

    RCS::cmds.addMsgQueue(cmd);

}

////////////////////////////////////////////////////////////////////////////////
int CCrcl2RosMsgApi::closeGripper()
{
     return setGripper(0.0);
}
////////////////////////////////////////////////////////////////////////////////
int CCrcl2RosMsgApi::openGripper()
{
   return setGripper(1.0);
}

////////////////////////////////////////////////////////////////////////////////
int CCrcl2RosMsgApi::doDwell(double dwelltime) {

    RCS::CCanonCmd cmd;
    // Toggleable infiste loop where debugging will get stuck
//    {
//        int bBreak=1;
//        while(bBreak) Globals.sleep(1000);
//    }
    cmd.crclcommand = CanonCmdType::CANON_DWELL;
    cmd.crclcommandnum = ++rcs_robot._crclcommandnum;
    cmd.dwell_seconds = dwelltime;
    cmd.eepercent=-1.0; // keep as is
    RCS::cmds.addMsgQueue(cmd);
    return cmd.crclcommandnum;


}
////////////////////////////////////////////////////////////////////////////////
::crcl_rosmsgs::CrclMaxProfileMsg CCrcl2RosMsgApi::getSpeeds()
{
    ::crcl_rosmsgs::CrclMaxProfileMsg profile;
    profile.maxvel=rates.CurrentTransSpeed();
    profile.maxacc=10.*rates.CurrentTransSpeed();
    profile.maxjerk= 100.*rates.CurrentTransSpeed();
    return profile;
}
////////////////////////////////////////////////////////////////////////////////
int CCrcl2RosMsgApi::moveTo(tf::Pose pose)
{

    RCS::CCanonCmd cmd;
    cmd.crclcommandnum = ++rcs_robot._crclcommandnum;
    cmd.crclcommand = CanonCmdType::CANON_MOVE_TO;
    cmd.profile.push_back(getSpeeds());
    cmd.finalpose = RCS::Convert<tf::Pose, geometry_msgs::Pose> (pose);
    cmd.eepercent=-1.0; // keep as is
    RCS::cmds.addMsgQueue(cmd);
    return cmd.crclcommandnum;
}


////////////////////////////////////////////////////////////////////////////////
int CCrcl2RosMsgApi::moveJoints(std::vector<long unsigned int> jointnum,
                         std::vector<double> positions,
                         double vel)
{
//#ifdef DIRECT_ROS_MSG
    crcl_rosmsgs::CrclCommandMsg cmd;
    CCanonCmd::setRosMsgTimestamp(cmd.header);
    cmd.crclcommandnum = ++rcs_robot._crclcommandnum;
    cmd.crclcommand = CanonCmdType::CANON_MOVE_JOINT;
    cmd.joints = zeroJointState(jointnum.size());
    cmd.joints.position = positions;
    cmd.jointnum=jointnum;
    cmd.joints.name=rcs_robot.jointNames;
    cmd.eepercent=-1.0; // keep as is
    cmd.bCoordinated = true;

    // THe lower level robot control really needs
    // the velocity profile set.
    ::crcl_rosmsgs::CrclMaxProfileMsg profile;
    vel = rates.MaxJointVelocity();
    profile.maxvel=vel;
    profile.maxacc=vel*10.;
    profile.maxjerk=vel*100.;
    cmd.profile.push_back(profile)  ;
    crcl::CRosCmdStatusHandler::crclmsgq->addMsgQueue(cmd);
//#else
#if 0
    RCS::CCanonCmd cc;
    cc.crclcommandnum = _crclcommandnum++;
    cc.crclcommand = CanonCmdType::CANON_MOVE_JOINT;
    cc.joints = zeroJointState(jointnum.size());
    cc.joints.position = positions;
    cc.jointnum=jointnum;
    cc.joints.name=rcs_robot.jointNames;
    cc.eepercent=-1.0; // keep as is
    cc.bCoordinated = true;

    // higher bogusian
    vel = rates.MaxJointVelocity();
    cc.Rates().CurrentTransSpeed()=vel;

    RCS::cmds.addMsgQueue(cmd);
#endif
    return cmd.crclcommandnum;
}

////////////////////////////////////////////////////////////////////////////////
void  CCrcl2RosMsgApi::pick(tf::Pose pose )
{

    tf::Vector3 offset = pose.getOrigin();

    openGripper(); // make sure griper is open

    // Approach object
    moveTo(rcs_robot.Retract * tf::Pose(rcs_robot.QBend, offset));
    doDwell(motionDwell());
    moveTo(tf::Pose(rcs_robot.QBend, offset));
    doDwell(motionDwell());
    closeGripper();
    doDwell(graspDwell());
    moveTo(rcs_robot.Retract * tf::Pose(rcs_robot.QBend, offset));
}

////////////////////////////////////////////////////////////////////////////////
void CCrcl2RosMsgApi::place(tf::Pose pose)
{

    tf::Vector3 offset = pose.getOrigin();

    // Retract
    moveTo(rcs_robot.Retract * tf::Pose(rcs_robot.QBend, offset));
    doDwell(motionDwell());

    // Place the grasped object
    moveTo(tf::Pose(rcs_robot.QBend, offset));
    doDwell(motionDwell());

    // open gripper and wait
    openGripper();
    doDwell(graspDwell());

    // Retract from placed object
    moveTo(rcs_robot.Retract * tf::Pose(rcs_robot.QBend, offset));

}


