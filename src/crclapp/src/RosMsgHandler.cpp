
/*
 * DISCLAIMER:
 * This software was produced by the National Institute of Standards
 * and Technology (NIST), an agency of the U.S. government, and by statute is
 * not subject to copyright in the United States.  Recipients of this software
 * assume all responsibility associated with its operation, modification,
 * maintenance, and subsequent redistribution.
 *
 * See NIST Administration Manual 4.09.07 b and Appendix I.
 */

// ROS
#include <urdf/model.h>

// Code synthesis
#include <CRCLStatus.hxx>

// NIST RCS
#include "rcs/IRcs.h"
#include "rcs/Debug.h"

// replace with globals
#include <crclapp/Crcl2RosMsgApi.h>
#include <crclapp/RosMsgHandler.h>
#include "crclapp/CrclPublisherInterface.h"

using namespace urdf;
using namespace RCS;
using namespace crcl;

namespace RCS {
CMessageQueue<RCS::CCanonCmd> cmds;
CCanonWorldModel wm; // for motion control planning wm
}

size_t num_joints;
size_t num_links;
RCS::CrclMessageQueue * crcl::CRosCmdStatusHandler::crclmsgq;
long CRosCmdMsgHandler::last_cmdnum=0;
std::string CRosCmdStatusHandler::length_units;
std::string CRosCmdStatusHandler::angle_units;


////////////////////////////////////////////////////////////////////////////////
CRosCmdStatusHandler::CRosCmdStatusHandler() :
    RCS::Thread(.01, "CCrcl2Ros")
{
    this->crclmsgq=NULL;
}


////////////////////////////////////////////////////////////////////////////////
void CRosCmdStatusHandler::statusUpdate(crcl_rosmsgs::CrclStatusMsg& statusmsg)
{
    std::lock_guard<std::mutex> guard(crcl::mutex);
    try {
        rcs_robot._status=statusmsg;
        crclinterface->crclsettings.StatusID() = (unsigned long long) statusmsg.crclcommandnum;
        rcs_robot.crclcommandid= statusmsg.crclcommandnum;
        rcs_robot.crclcommandstatus= statusmsg.crclcommandstatus;
        static char * value[] =
        {
            "CRCL_Done",
            "CRCL_Error",
            "CRCL_Working",
            "CRCL_Ready"
        };
        //        crclinterface->crclwm.CommandStatus() = ::CommandStateEnumType(CanonStatusType::value(statusmsg->crclcommandstatus));
        if(statusmsg.crclcommandstatus < 4)
        {
            crclinterface->crclsettings.Update( ::CommandStateEnumType( value[statusmsg.crclcommandstatus]));
            rcs_robot.s_crclcommandstatus= value[statusmsg.crclcommandstatus];
        }

        // FIXME: this has to be in the agreed upon CRCL units
        tf::Pose pose = RCS::Convert< geometry_msgs::Pose, tf::Pose>(statusmsg.statuspose);

        // default tf length units are Meters - convert to crcl
        pose.setOrigin(pose.getOrigin() *  1.0/crclinterface->crclsettings._lengthConversion);
        crclinterface->crclsettings.Update(pose );

        sensor_msgs::JointState js = statusmsg.statusjoints;

        for(size_t i=0; i< js.position.size(); i++)
            js.position[i]=js.position[i]* 1.0/crclinterface->crclsettings._angleConversion;

        crclinterface->crclsettings.Update((sensor_msgs::JointState&) js);
        crclinterface->crclsettings.Gripper().Position() = statusmsg.eepercent;

        for(size_t i=0; i< WorldModel::instances.size(); i++)
        {
            tf::Pose pose = WorldModel::instances[i].centroid();
            crclinterface->crclsettings.Update(WorldModel::instances[i].name(), pose);
            crclinterface->crclsettings.Update(WorldModel::instances[i]);
        }
    }
    catch(...)
    {
        ROS_DEBUG("crclwm command status failed");
    }
}

////////////////////////////////////////////////////////////////////////////////
void CRosCmdStatusHandler::setup()
{

    // Controller instantiation of shared objects - NOT dependent on ROS
    crclinterface = boost::shared_ptr<crcl::CrclSubscriberDelegateInterface>(
                new crcl::CrclSubscriberDelegateInterface());

    // Couple code attempts at reading from robot_description parameter - see above
    crclinterface->crclsettings.jointnames.clear();

    //rcs_robot.parseURDF(xmlString, baseLink,  tipLink);

    crclinterface->crclsettings.jointnames=rcs_robot.jointNames;
}

////////////////////////////////////////////////////////////////////////////////
int CRosCmdStatusHandler::action()
{
    try {

        int n=0;
        /////////////////////////////////////////////////////////////////////////////////////////////
        // See if new CRCL commanded motion - if so, interpret as RCS command in session
        if(crcl::CSocketSession::InMessages().sizeMsgQueue() > n)
        {
            crcl::CrclMessage msg = crcl::CSocketSession::InMessages().popFrontMsgQueue();
            std::string crclcmd = boost::get<0>(msg);

            if(crcl::dbgCrclCommand)
            {
                ROS_DEBUG_STREAM( crclcmd);
            }


            crcl::CrclReturn ret = crclinterface->DelegateCRCLCmd(crclcmd);

            if (ret == crcl::CANON_STATUSREPLY )
            {
                crcl::CrclStatus crclstatus;
                {
                    // can't have reporting and robot controller so make copy
                    std::lock_guard<std::mutex> guard(crcl::mutex);
                    crclstatus = crclinterface->crclsettings;
                }
                std::string sStatus = crcl::CrclPublisherCmdInterface().GetStatusReply(&crclstatus);

                if(!sStatus.empty())
                {
                    // no mutex as there is only one session running handling all client CRCL messages
                    crcl::CSocketSession *_pSession;
                    _pSession = boost::get<1>(msg);
                    _pSession->SyncWrite(sStatus);
                    if(crcl::dbgCrclStatus)
                    {
                        std::cout << sStatus;
                    }
                }

                if(crcl::dbgCrclStatusMsg)
                {
                    ROS_DEBUG("===========================================================\n"
                              "Status:\n%s",
                              sStatus.c_str());
                    ROS_DEBUG("===========================================================");
                }

            }
            if(crcl::bFlywheel)
            {
                n=crcl::CSocketSession::InMessages().sizeMsgQueue();
            }
        }
        if (crcl::bAutoCrclStatus)
        {
            crcl::CrclStatus crclstatus;
            {
                // get copy of current crcl status settings
                std::lock_guard<std::mutex> guard(crcl::mutex);
                crclstatus = crclinterface->crclsettings;
            }
            std::string sStatus = crcl::CrclPublisherCmdInterface().GetStatusReply(&crclstatus);

            if(!sStatus.empty())
            {
                // no mutex as there is only one session running handling all client CRCL messages
                crcl::CSocketSession *_pSession=crclCommServer.get();
                _pSession->SyncWrite(sStatus);
                if(crcl::dbgCrclStatus)
                {
                    std::cout << sStatus;
                }
            }
        }
    }
    catch (std::exception & e)
    {
        std::cerr << "Exception in  CCrcl2Ros::action thread: " << e.what() << "\n";
    }
    catch (...)
    {
        std::cerr << "Exception in CCrcl2Ros::action thread\n";
    }
    return 1;
}


////////////////////////////////////////////////////////////////////////////////
CRosCmdMsgHandler::CRosCmdMsgHandler( crcl::CRosCmdStatusHandler * rosCmdStatMsgHandler) :
    RCS::Thread(.01, "CCrcl2RosMsg")
{
    _rosCmdStatMsgHandler=rosCmdStatMsgHandler;
}

////////////////////////////////////////////////////////////////////////////////
crcl_rosmsgs::CrclStatusMsg CRosCmdMsgHandler::createRosStatus(
        long crclCommandStatus,
        long crclcommandnum,
        tf::Pose currentpose,
        sensor_msgs::JointState robotjoints,
        double eepercent
        )
{
    crcl_rosmsgs::CrclStatusMsg statusmsg;
    statusmsg.crclcommandstatus = crclCommandStatus;
    statusmsg.crclcommandnum = crclcommandnum;
    statusmsg.crclstatusnum = crclcommandnum;

    statusmsg.statuspose = RCS::Convert<tf::Pose, geometry_msgs::Pose> (currentpose);
    statusmsg.statusjoints = robotjoints;
    statusmsg.eepercent = eepercent;
    return statusmsg;
}

////////////////////////////////////////////////////////////////////////////////
int CRosCmdMsgHandler::action()
{
    /////////////////////////////////////////////////////////////////////////////////////////////
    // interpret translated CRCL command.
    // Commands in canonical form: standard units (mm, radians)
    // Note many CRCL commands are NOT translated into corresponding canoncial ROS commands
    try
    {
        int n = 0; // flywheel implementation - whether to process all messages at once or per cycle.
        for (;RCS::cmds.sizeMsgQueue() > n;)
        {
            RCS::CCanonCmd cc = RCS::cmds.popFrontMsgQueue();

            if(cc.crclcommand == CanonCmdType::CANON_INIT_CANON)
            {
                // Ignore init canon
                if(crcl::bDebug)
                    std::cout << "crcl initcanon[" << cc.crclcommandnum <<"]" << std::endl;
            }
            else if(cc.crclcommand == CanonCmdType::CANON_END_CANON)
            {
                // Ignore end canon
                if(crcl::bDebug)
                    std::cout << "crcl endcanon[" << cc.crclcommandnum <<"]" << std::endl;
            }
            else if (cc.crclcommand == CanonCmdType::CANON_MOVE_JOINT)
            {
                if(crcl::bDebug)
                    std::cout << "crcl moveJoints[" << cc.crclcommandnum <<"]" << dumpStdVector(cc.joints.position)  << std::endl;
                // FIXME: could only move 1 joint as in a jog....
                crclRobotImpl->moveJoints(cc.joints.position);
            }
            else if (cc.crclcommand == CanonCmdType::CANON_MOVE_TO)
            {
                if(crcl::bDebug)
                    std::cout << "crcl moveTo[" << cc.crclcommandnum <<"]"
                              << dumpPoseSimple(RCS::Convert<geometry_msgs::Pose, tf::Pose>(cc.finalpose) )
                              << std::endl;

                // Fixme: rates are ignored by RCS crcl interpreter
//                if(cc.Rates().CurrentTransSpeed() >0.0)
//                {
//                    // clear rate vector
//                    rosmsg.profile.clear();

//#if 1
//                    ::crcl_rosmsgs::CrclMaxProfileMsg profile;
//                    double vel = CCrcl2RosMsgApi::rates.MaxJointVelocity();
//                    profile.maxvel=vel;
//                    profile.maxacc=vel*10.;
//                    profile.maxjerk=vel*100.;
//                    rosmsg.profile.push_back(profile)  ;
//#endif
//                }

                crcl_rosmsgs::CrclStatusMsg statusmsg = createRosStatus(
                            RCS::CanonStatusType::CANON_WORKING,
                            cc.crclcommandnum,
                            rcs_robot.current_pose,
                            rcs_robot.current_joints,
                            rcs_robot.current_eesetting
                            );
                _rosCmdStatMsgHandler->statusUpdate(statusmsg);
                tf::Pose tfpose =  RCS::Convert<geometry_msgs::Pose,tf::Pose> (cc.finalpose);
                crclRobotImpl->eeMoveTo(tfpose);
                statusmsg.crclcommandstatus=CanonStatusType::CANON_DONE;
                _rosCmdStatMsgHandler->statusUpdate(statusmsg);
            }
            else if (cc.crclcommand == CanonCmdType::CANON_STOP_MOTION)
            {
                // Fixme: there are other parameters specifying stop
                if(crcl::bCrclStopIgnore)
                {
                    // fixme: interrupt "moveto abort looping to goal...
                    // problem is queue handling is synchronous and wont see this.
                    // may have to preempt in cmd/stat queue handling  above thread
                }

            }
            else if (cc.crclcommand == CanonCmdType::CANON_MOVE_THRU)
            {
                ROS_FATAL("**********CanonCmdType::CANON_MOVE_THRU NOT IMPLEMENTED");
                // FIXME: just loop through waypoints as means of working.
                assert(0);
            }
            else if (cc.crclcommand == CanonCmdType::CANON_DWELL)
            {
                if(crcl::bDebug)
                    std::cout << "crcl [" << cc.crclcommandnum <<"] dwell=" <<  cc.dwell_seconds << std::endl;

                crcl_rosmsgs::CrclStatusMsg statusmsg = createRosStatus(
                            CanonStatusType::CANON_WORKING,
                            cc.crclcommandnum,
                            rcs_robot.current_pose,
                            rcs_robot.current_joints,
                            rcs_robot.current_eesetting
                            );
                _rosCmdStatMsgHandler->statusUpdate(statusmsg);
                crclRobotImpl->doDwell(cc.dwell_seconds);
                statusmsg.crclcommandstatus=CanonStatusType::CANON_DONE;
                _rosCmdStatMsgHandler->statusUpdate(statusmsg);
            }
            else if (cc.crclcommand == CanonCmdType::CANON_SET_GRIPPER)
            {
                if(crcl::bDebug)
                    std::cout << "crcl setGripper[" << cc.crclcommandnum << "]" <<  cc.eepercent << std::endl;
                crcl_rosmsgs::CrclStatusMsg statusmsg = createRosStatus(
                            CanonStatusType::CANON_WORKING,
                            cc.crclcommandnum,
                            rcs_robot.current_pose,
                            rcs_robot.current_joints,
                            rcs_robot.current_eesetting
                            );
                _rosCmdStatMsgHandler->statusUpdate(statusmsg);
                crclRobotImpl->setGripper(cc.eepercent);
                statusmsg.crclcommandstatus=CanonStatusType::CANON_DONE;
                _rosCmdStatMsgHandler->statusUpdate(statusmsg);
            }
            // added oct 30 2018 to see if "processing all messages important"
            // actually no flywheel but processAllCrclMessages
            else if(!crcl::bProcessAllCrclMessages)
            {
                cc.crclcommand=CanonCmdType::CANON_NOOP;
            }
            if(crcl::bFlywheel)
            {
                n=RCS::cmds.sizeMsgQueue();
            }

        }
    }
    catch (std::exception & e)
    {
        std::cerr << "Exception in  CCrcl2RosMsg::action() thread: " << e.what() << "\n";
    }
    catch (...)
    {
        std::cerr << "Exception in CCrcl2RosMsg::action() thread\n";
    }
    return 1;
}



