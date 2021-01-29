// CrclSubscriberInterface.cpp

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
// C++ headers
#include <iostream>
#include <fstream>
#include <sstream>
#define _USE_MATH_DEFINES
#include <math.h>       /* isnan, sqrt */

// xercesc headers
#include <xercesc/dom/DOM.hpp>
#include <xercesc/util/PlatformUtils.hpp>
#include <xercesc/framework/XMLGrammarPoolImpl.hpp>

// boost headers
#include <boost/regex.hpp>
#include <boost/exception/all.hpp>

// crcl headers
#include "crclapp/CrclSubscriberInterface.h"
#include "crclapp/CrclPublisherInterface.h"
#include "crclapp/RosMsgHandler.h"
#include "crclapp/CrclPrimitives.h"

#include "rcs/RCSMsgQueue.h"
#include "rcs/Conversions.h"

using namespace xsd;
using namespace xml_schema;
using namespace crcl;
using namespace RCS;

CrclStatus CrclSubscriberDelegateInterface::crclsettings;

//
// ------------------------------------------------------------------------------
// CrclSubscriberDelegateInterface

std::string CrclSubscriberDelegateInterface::FindLeadingElement(std::string xml) {
    boost::match_results<std::string::const_iterator> matchResult;
    bool found;
    boost::regex e("<[A-Za-z0-9_]+");
    found = boost::regex_search(xml, matchResult, e);

    if (found) {
        std::string elem(matchResult[0]);
        elem.insert(1, 1, '/');
        elem = Globals.trim(elem);
        elem.append(">"); // not space
        return elem;
    }
    return "";
}
void CrclSubscriberDelegateInterface::CrclRunProgram(::CRCLProgramType::MiddleCommand_sequence cmds)
{
    for(size_t i=0; i< RCS::cmds.sizeMsgQueue(); i++){
                   
        ::CRCLCommandType & crclCommand(cmds[i]);
        DelegateCRCLCmd(crclCommand);
        //::MiddleCommandType  crclCommand( crclInstanceCommand->CRCLCommand());
    }
    RCS::cmds.clearMsgQueue();
}
CrclReturn CrclSubscriberDelegateInterface::DelegateCRCLCmd(std::string str) {

    boost::trim(str);
    std::istringstream istr(str);

    if (FindLeadingElement(str) == "</CRCLProgram>") {
        try {
            boost::shared_ptr<::CRCLProgramType> crclProgram(
                    CRCLProgram(istr, xml_schema::flags::dont_initialize | xml_schema::flags::dont_validate | xml_schema::flags::keep_dom)
                    );
            ::CRCLProgramType::MiddleCommand_sequence& cmds = crclProgram->MiddleCommand();
            CrclRunProgram(cmds);

        } catch (const xml_schema::exception& e) {
            // Most likely here due to illegal XML in Crcl program. Note, is not validated against XSD.
            std::cout << "Parse Exception RobotProgram::ExecuteProgram:" << e << std::endl;
            return CANON_FAILURE;
        } catch (...) {
            std::cout << "Unhandled exception\n";
            return CANON_FAILURE;
        }
        
    }
    else if (FindLeadingElement(str) == "</CRCLCommandInstance>") {
        try {
            std::auto_ptr<CRCLCommandInstanceType> crclInstanceCommand(
                    CRCLCommandInstance(istr, xml_schema::flags::dont_initialize | xml_schema::flags::dont_validate | xml_schema::flags::keep_dom)
                    );
            ::CRCLCommandType & crclCommand(crclInstanceCommand->CRCLCommand());

            //::MiddleCommandType  crclCommand( crclInstanceCommand->CRCLCommand());

            // Assign command id using CodeSynthesis XML parsing.
            _nCommandNum = crclCommand.CommandID();
            crclsettings.CommandID()= crclCommand.CommandID();

            if (GetStatusType * stat = dynamic_cast<GetStatusType *> (&(crclCommand))) {
                ROS_DEBUG("GetStatus id=%d", stat->CommandID());
                return GetStatus();
            }
            if(crcl::dbgCrclCommandMsg)
            {
                ROS_DEBUG("===========================================================\n"
                          "[%s]",
                          str.c_str());
            }

            return DelegateCRCLCmd(crclCommand);
        } catch (const xml_schema::exception& e) {
            // Most likely here due to illegal XML in Crcl command. Note, is not validated against XSD?!?!
            ROS_DEBUG( "Parse Exception CrclServerDelegateInterface::DelegateCRCLCmd: %s\n%s\n", e.what(), str.c_str());
            return CANON_FAILURE;
        } 
    }
    else if (FindLeadingElement(str) == "</CRCLStatus>") {
        // should never get status as server
//        std::auto_ptr<CRCLStatusType> crclstat(
//                CRCLStatus(istr, xml_schema::flags::dont_initialize | xml_schema::flags::dont_validate | xml_schema::flags::keep_dom)
//                );
        return CANON_REJECT;
    }
    return CANON_SUCCESS;

}

CrclReturn CrclSubscriberDelegateInterface::DelegateCRCLCmd(::CRCLCommandType &crclCommand) {
    if (ActuateJointsType * actuateJoints = dynamic_cast<ActuateJointsType *> (&crclCommand)) // (&(*cmd)))
    {

        crclsettings.Update(_nCommandNum);
        crclsettings.Update(crcl::CommandStateEnum("CRCL_Working"));
        ActuateJointsType::ActuateJoint_sequence & joints(actuateJoints->ActuateJoint());
        ROS_DEBUG("ActuateJoints id=%d joint=%d pos = %6.5f", crclCommand.CommandID(), joints[0].JointNumber(), joints[0].JointPosition());
        return ActuateJoints(joints);

        // m_thread=boost::thread(&CrclServerDelegateInterface::CRCLActuateJoints, this, joints);
    }        // MoveToType
    else if (MoveToType * moveto = dynamic_cast<MoveToType *> (&(crclCommand)))
    {
        MoveToType::EndPosition_type & endpose(moveto->EndPosition());
        bool bStraight = moveto->MoveStraight();
        return  MoveTo(endpose, bStraight);
    }
    else if (EndCanonType * endcanon = dynamic_cast<EndCanonType *> (&(crclCommand)))
    {
        return  EndCanon();
    }
    else if (InitCanonType * init = dynamic_cast<InitCanonType *> (&(crclCommand)))
    {
        ROS_DEBUG("InitCanonType id=%d", init->CommandID());
        return  InitCanon();
    }
    else if (MessageType * msg = dynamic_cast<MessageType *> (&(crclCommand)))
    {
        return Message(msg->Message());
    }
    else if (MoveThroughToType * movethrough = dynamic_cast<MoveThroughToType *> (&(crclCommand)))
    {
        size_t n = movethrough->NumPositions();
        bool bStraight = movethrough->MoveStraight();
        MoveThroughToType::Waypoint_sequence & waypoints = movethrough->Waypoint();
        // poses are copied - otherwise fault is thrown
        std::vector<crcl::PoseType> poses;
        for (size_t i = 0; i < waypoints.size(); i++)
            poses.push_back(waypoints[i]);
        return MoveThroughTo(poses, bStraight);
    }
    else if (StopMotionType * stopmotion = dynamic_cast<StopMotionType *> (&(crclCommand)))
    {
        const char *sEnums[] = {"Immediate", "Fast", "Normal"};
        ::StopConditionEnumType & cond(stopmotion->StopCondition());
        int nCond = cond;
        std::string sUnit("Immediate");

        if ((nCond >= 0) && (nCond < 3)) {
            sUnit = sEnums[nCond];
        }
        return StopMotion(nCond);
    }
    else if (OpenToolChangerType * opentool = dynamic_cast<OpenToolChangerType *> (&(crclCommand)))
    {
        return OpenToolChanger();
    }
    else if (SetAngleUnitsType * angleUnits = dynamic_cast<SetAngleUnitsType *> (&(crclCommand)))
    {
        const char *sEnums[] = {"degree", "radian"};
        ::AngleUnitEnumType & units(angleUnits->UnitName());
        int nUnit = units;
        std::string sUnit("radian");

        if ((nUnit >= 0) && (nUnit < 2)) {
            sUnit = sEnums[nUnit];
        }
        return SetAngleUnits(sUnit);
    }
    else if (SetLengthUnitsType * lengthUnits = dynamic_cast<SetLengthUnitsType *> (&(crclCommand)))
    {
        const char *sEnums[] = {"meter", "millimeter", "inch"};
        ::LengthUnitEnumType & units(lengthUnits->UnitName());
        int nUnit = units;
        std::string sUnit("meter");

        if ((nUnit >= 0) && (nUnit < 3)) {
            sUnit = sEnums[nUnit];
        }
        return SetLengthUnits(sUnit);
    }
    else if (CloseToolChangerType * closetool = dynamic_cast<CloseToolChangerType *> (&(crclCommand)))
    {
        return CloseToolChanger();
    }
    else if (ConfigureJointReportsType * configurereports = dynamic_cast<ConfigureJointReportsType *> (&(crclCommand))) {
        bool bResetAll = configurereports->ResetAll();
        ConfigureJointReportsType::ConfigureJointReport_sequence & report = configurereports->ConfigureJointReport();
        std::vector<crcl::JointReport> jointReports;

        for (size_t i = 0; i < report.size(); i++) {
            crcl::JointReport jr;
            jr._nJointNumber = report[i].JointNumber();
            jr._bReportPosition = report[i].ReportPosition();
            jr._bReportTorqueOrForce = report[i].ReportTorqueOrForce();
            jr._bReportVelocity = report[i].ReportVelocity();
            jointReports.push_back(jr);
        }
        return ConfigureJointReports(jointReports);
    }
    else if (DwellType * dwelltype = dynamic_cast<DwellType *> (&(crclCommand)))
    {
        double time = dwelltype->DwellTime();
        return Dwell(time);    
    }
    else if (MoveScrewType * movescrew = dynamic_cast<MoveScrewType *> (&(crclCommand)))
    {
    }
//    else if (SetEndEffectorParametersType * endeeparamtype = dynamic_cast<SetEndEffectorParametersType *> (&(crclCommand))) {
//    }
    else if (SetEndEffectorType * endeetype = dynamic_cast<SetEndEffectorType *> (&(crclCommand)))
    {
        double setting = endeetype->Setting();
        return SetEndEffector(setting);
    }
    else if (SetEndPoseToleranceType * setEndTolerance = dynamic_cast<SetEndPoseToleranceType *> (&(crclCommand)))
    {
        SetEndPoseToleranceType::Tolerance_type & tolerancepose(setEndTolerance->Tolerance());
        return SetEndPoseTolerance(tolerancepose);
    }
    else if (SetForceUnitsType * forceunits = dynamic_cast<SetForceUnitsType *> (&(crclCommand)))
    {
        //    newton,    pound,    ounce
        const char *sEnums[] = {"newton", "pound", "ounce"};
        ::ForceUnitEnumType & units(forceunits->UnitName());
        int nUnit = units;
        std::string sUnit("newton");

        if ((nUnit >= 0) && (nUnit < 3)) {
            sUnit = sEnums[nUnit];
        }
        return SetTorqueUnits(sUnit);
    }


    else if (SetEndEffectorParametersType * param = dynamic_cast<SetEndEffectorParametersType *> (&(crclCommand)))
    {
        std::vector<std::string> names;
        std::vector<std::string> values;

        SetEndEffectorParametersType::ParameterSetting_sequence& settings = param->ParameterSetting();
        for(size_t i=0;  i< settings.size(); i++)
        {
            ParameterSettingType setting = settings[i];
            names.push_back(setting.ParameterName());
            values.push_back(setting.ParameterValue());
        }
        return SetEndEffectorParameters(names, values);
    }
    else if (SetIntermediatePoseToleranceType * posetol = dynamic_cast<SetIntermediatePoseToleranceType *> (&(crclCommand))) {
        return SetIntermediatePoseTolerance(posetol->Tolerance());
    }
    else if (SetMotionCoordinationType * coordtype = dynamic_cast<SetMotionCoordinationType *> (&(crclCommand)))
    {
        bool bCoordinatedMotion = coordtype->Coordinated();
        return SetMotionCoordination(bCoordinatedMotion);
    }
    else if (RotAccelAbsoluteType * absrotaccel = dynamic_cast<RotAccelAbsoluteType *> (&(crclCommand)))
    {
        double accel = absrotaccel->Setting();
        return SetRotAccel(accel);
    }
    else if (RotAccelRelativeType * relrotaccel = dynamic_cast<RotAccelRelativeType *> (&(crclCommand)))
    {
        double percent = relrotaccel->Fraction();
        double accel = RCS::wm._maxRotAccel * percent;
        return SetRotAccel(accel);
    }
    else if (RotSpeedAbsoluteType * rotspeed = dynamic_cast<RotSpeedAbsoluteType *> (&(crclCommand)))
    {
        double speed = rotspeed->Setting();
        return SetRotSpeed(speed);
    }
    else if (RotSpeedRelativeType * rotspeed = dynamic_cast<RotSpeedRelativeType *> (&(crclCommand)))
    {
        long percent = rotspeed->Fraction();
        double speed = RCS::wm._maxRotVel * percent;
        return SetRotSpeed(speed);
    }
    else if (SetTorqueUnitsType * torqueUnits = dynamic_cast<SetTorqueUnitsType *> (&(crclCommand)))
    {
        //   newtonMeter,    footPound
        const char *sEnums[] = {"newtonMeter", "footPound"};
        ::TorqueUnitEnumType & units(torqueUnits->UnitName());
        int nUnit = units;
        std::string sUnit("newtonMeter");

        if ((nUnit >= 0) && (nUnit < 2)) {
            sUnit = sEnums[nUnit];
        }
        return SetTorqueUnits(sUnit);
    }
    else if (SetTransAccelType * accltype = dynamic_cast<SetTransAccelType *> (&(crclCommand)))
    {
        double accel;

        if (TransAccelAbsoluteType * absAccltype = dynamic_cast<TransAccelAbsoluteType *> (&(accltype->TransAccel()))) {
            accel = absAccltype->Setting();
        } else if (TransAccelRelativeType * relAccltype = dynamic_cast<TransAccelRelativeType *> (&(accltype->TransAccel()))) {
            double percentage = relAccltype->Fraction();
            accel = RCS::wm._maxTransVel * percentage;
        }
        return SetTransAccel(accel);
    }
    else if (SetTransSpeedType * speedtype = dynamic_cast<SetTransSpeedType *> (&(crclCommand)))
    {
        double speed;

        if (TransSpeedAbsoluteType * absSpeedtype = dynamic_cast<TransSpeedAbsoluteType *> (&(speedtype->TransSpeed()))) {
            speed = absSpeedtype->Setting();
        } else if (TransSpeedRelativeType * relSpeedtype = dynamic_cast<TransSpeedRelativeType *> (&(speedtype->TransSpeed()))) {
            double percentage = relSpeedtype->Fraction();
            speed = RCS::wm._maxTransVel * percentage;
        }
        return SetTransSpeed(speed);
    }
    return CANON_NOT_IMPLEMENTED;
}


// ---------------------------------------------------------------------------------------
// CrclServerDelegateInterface - handles CRCL command


CrclReturn CrclSubscriberDelegateInterface::SetEndEffectorParameters(std::vector<std::string> names,
                                                                 std::vector<std::string> values)
{
    RCS::CCanonCmd cc;
    cc.crclcommand = CanonCmdType::CANON_SET_EE_PARAMETERS;
    cc.CrclCommandID() = crclsettings.CommandID();
    cc.parameterNames=names;
    cc.parameterValues=values;

    RCS::cmds.addMsgQueue(cc);
    return CANON_MOTION;
}

////////////////////////////////////////////////////////////////////////////////
CrclReturn CrclSubscriberDelegateInterface::ActuateJoints(crcl::ActuatorJointSequence joints)
{
    std::string strcmd="";
    RCS::CCanonCmd cc;
    cc.crclcommand = CanonCmdType::CANON_MOVE_JOINT;
    cc.CrclCommandID() = crclsettings.CommandID();
    cc.bCoordinated=crclsettings._bCoordinatedMotion;
    cc.endPoseTol=crcl::Convert(crclsettings._endPoseTolerance); // not sure useful?

    for (int i = 0; i < joints.size(); i++) {
        int jn = joints[i].JointNumber() - 1;
        double speed, accel;
        speed = crclsettings.Rates().JointVelLimit().at(jn);
        accel = crclsettings.Rates().JointAccLimit().at(jn);
        double setting, changerate;
        std::string type;
        if (JointSpeedAccelType * speedacc = dynamic_cast<JointSpeedAccelType *> (&(joints[i].JointDetails())))
        {
            type = "JointSpeedAccelType";
            if(speedacc->JointSpeed().present ())
                speed = *speedacc->JointSpeed() * crclsettings._lengthConversion;
            if(speedacc->JointAccel().present ())
                accel = *speedacc->JointAccel() * crclsettings._lengthConversion;

        }
        else if (JointForceTorqueType * forcetorque = dynamic_cast<JointForceTorqueType *> (&(joints[i].JointDetails())))
        {
            type = "JointForceTorqueType";
            if(forcetorque->Setting().present ())
                setting = *forcetorque->Setting();
            if(forcetorque->ChangeRate().present ())
                changerate = *forcetorque->ChangeRate();
        }


        //if ( ( j.type == RCS::RdfJoint::REVOLUTE ) || ( j.type == RCS::RdfJoint::CONTINUOUS ) )
        {
            cc.jointnum.push_back(joints[i].JointNumber() - 1); // adjust back to zero based math
            double pos = joints[i].JointPosition() * crclsettings._angleConversion;
            ROS_DEBUG("Joint names %s", RCS::vectorDump<std::string> (crclsettings.jointnames).c_str());
            cc.joints.name.push_back(crclsettings.jointnames[cc.jointnum.back()]);
            cc.joints.position.push_back(pos);
            cc.joints.velocity.push_back(speed); //  need conversion of velocity?
            cc.joints.effort.push_back(accel); //   need conversion of acc?Format()
            strcmd+=StrFormat(",%6.4f",pos);
        }
    }
    ROS_DEBUG("ActuateJoints In=%s", strcmd.c_str());
    ROS_DEBUG("ActuateJoints Rcs=%s", RCS::vectorDump<double>(cc.joints.position).c_str());

    RCS::cmds.addMsgQueue(cc);

    return CANON_MOTION;
}

////////////////////////////////////////////////////////////////////////////////
CrclReturn CrclSubscriberDelegateInterface::CloseToolChanger()
{
    RCS::CCanonCmd cc;
    cc.crclcommand = CanonCmdType::CANON_SET_GRIPPER;
    cc.CrclCommandID() =  crclsettings.CommandID();
    cc.eepercent = 0.0;
    RCS::cmds.addMsgQueue(cc);
    return CANON_MOTION;
}

////////////////////////////////////////////////////////////////////////////////
CrclReturn CrclSubscriberDelegateInterface::ConfigureJointReports(std::vector<crcl::JointReport> & jointReports)
{
    RCS::CCanonCmd cc;
    cc.crclcommand = CanonCmdType::CANON_NOOP;
    cc.CrclCommandID() =  crclsettings.CommandID();
    RCS::cmds.addMsgQueue(cc);

    crclsettings._vJointReport.clear();
    crclsettings._vJointReport.insert(crclsettings._vJointReport.begin(), jointReports.begin(), jointReports.end());
    return CANON_SUCCESS;
}

CrclReturn CrclSubscriberDelegateInterface::Couple(char *targetID)
{
    RCS::CCanonCmd cc;
    cc.crclcommand = CanonCmdType::CANON_NOOP;
    cc.CrclCommandID() =  crclsettings.CommandID();
    RCS::cmds.addMsgQueue(cc);

    return CANON_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////
CrclReturn CrclSubscriberDelegateInterface::Dwell(double seconds)
{
    RCS::CCanonCmd cc;
    cc.crclcommand = CanonCmdType::CANON_DWELL;
    cc.CrclCommandID() = crclsettings.CommandID();
    cc.dwell_seconds = seconds;
    RCS::cmds.addMsgQueue(cc);
    return CANON_MOTION;
}

////////////////////////////////////////////////////////////////////////////////
CrclReturn CrclSubscriberDelegateInterface::EndCanon()
{
    //signal(SIG_INT);
    RCS::CCanonCmd cc;
    cc.crclcommand = CanonCmdType::CANON_END_CANON;
    cc.CrclCommandID() =  crclsettings.CommandID();
    RCS::cmds.addMsgQueue(cc);

    return CANON_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////
CrclReturn CrclSubscriberDelegateInterface::GetStatus()
{
    crclsettings.Update(_nCommandNum);
    return CANON_STATUSREPLY;
}

////////////////////////////////////////////////////////////////////////////////
CrclReturn CrclSubscriberDelegateInterface::InitCanon()
{
    crclsettings.Update(_nCommandNum);
    crclsettings.Update(crcl::CommandStateEnum("CRCL_Done"));
    crcl::JointStatusSequence jstat(CrclStatus().JointsHome());
    crclsettings.Update(jstat);
    crcl::PoseType poseref(crcl::PoseHome());
    crclsettings.Update(poseref);
    std::string sStatus = crcl::CrclPublisherCmdInterface().GetStatusReply(&crclsettings);

    // _pAsioSession->SyncWrite(sStatus);
    return CANON_STATUSREPLY;
}

////////////////////////////////////////////////////////////////////////////////
CrclReturn CrclSubscriberDelegateInterface::Message(std::string message)
{
    RCS::CCanonCmd cc;
    cc.crclcommand = CanonCmdType::CANON_NOOP;
    cc.CrclCommandID() =  crclsettings.CommandID();
    RCS::cmds.addMsgQueue(cc);

    if(*message.rbegin() != '\n')
        message+="\n";
    // Fixme: add popup? Forward to CNC?
    std::cout << message;
    return CANON_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////
CrclReturn CrclSubscriberDelegateInterface::MoveTo(crcl::PoseType endpose, bool bStraight)
{
    RCS::CCanonCmd cc;
    cc.crclcommand = CanonCmdType::CANON_MOVE_TO;
    cc.CrclCommandID() = crclsettings.CommandID();
    cc.bStraight=bStraight;
    cc.endPoseTol=crcl::Convert(crclsettings._endPoseTolerance);


    // default length units are Meters
    tf::Pose pose = Convert(endpose, crclsettings._lengthConversion);

    // convert pose into message pose
    cc.finalpose =  RCS::Convert<tf::Pose, geometry_msgs::Pose>(pose);

    // Save rates, although unused for now
    cc.Rates() = crclsettings.Rates();
            
    // Save command in parsed message queue
    RCS::cmds.addMsgQueue(cc);

    // Signal that this was a motion to handle
    return CANON_MOTION;
}

////////////////////////////////////////////////////////////////////////////////
CrclReturn CrclSubscriberDelegateInterface::MoveThroughTo(std::vector<crcl::PoseType> & poses, bool bStraight)
{
    RCS::CCanonCmd cc;
    cc.crclcommand = CanonCmdType::CANON_MOVE_THRU;
    cc.CrclCommandID() =  crclsettings.CommandID();
    cc.bStraight=bStraight;
    cc.endPoseTol=crcl::Convert(crclsettings._endPoseTolerance);
    cc.intermediatePtTol=crcl::Convert(crclsettings._intermediatePoseTolerance);
    //cc.endposetolerance=crcl::Convert(crclwm._gripperPoseTolerance);

    // default length and angular units are Meters and Radians
    for (size_t i = 0; i < poses.size(); i++) {
        tf::Pose waypoint = Convert(poses[i], crclsettings._lengthConversion);
        cc.waypoints.push_back(RCS::Convert<tf::Pose, geometry_msgs::Pose>(waypoint));

        ROS_DEBUG("GotoCRCL Pose %s" , crcl::DumpPose(poses[i], ",").c_str() );
        ROS_DEBUG("Goto urdf Pose %s" , RCS::dumpPose(waypoint).c_str());
    }

    // FIXME: add tolerance to each waypoint - blending distance?
    RCS::cmds.addMsgQueue(cc);
    return CANON_MOTION;
}

////////////////////////////////////////////////////////////////////////////////
CrclReturn CrclSubscriberDelegateInterface::OpenToolChanger()
{
    RCS::CCanonCmd cc;
    cc.crclcommand = CanonCmdType::CANON_SET_GRIPPER;
    cc.CrclCommandID() = crclsettings.CommandID();
    cc.eepercent = 1.0;
    RCS::cmds.addMsgQueue(cc);
    return CANON_MOTION;
}

////////////////////////////////////////////////////////////////////////////////
CrclReturn CrclSubscriberDelegateInterface::RunProgram(std::string programText)
{
    RCS::CCanonCmd cc;
    cc.crclcommand = CanonCmdType::CANON_NOOP;
    cc.CrclCommandID() =  crclsettings.CommandID();
    RCS::cmds.addMsgQueue(cc);

    ROS_DEBUG("CrclServerDelegateInterface::RunProgram NOT IMPLEMENTED\n");
    return CANON_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////
CrclReturn CrclSubscriberDelegateInterface::SetAbsoluteAcceleration(double acceleration)
{
    RCS::CCanonCmd cc;
    cc.crclcommand = CanonCmdType::CANON_NOOP;
    cc.CrclCommandID() =  crclsettings.CommandID();
    RCS::cmds.addMsgQueue(cc);

    crclsettings.Rates().CurrentTransAcceleration()=acceleration;
    return CANON_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////
CrclReturn CrclSubscriberDelegateInterface::SetAbsoluteSpeed(double speed)
{
    RCS::CCanonCmd cc;
    cc.crclcommand = CanonCmdType::CANON_NOOP;
    cc.CrclCommandID() =  crclsettings.CommandID();
    RCS::cmds.addMsgQueue(cc);

    crclsettings.Rates().CurrentTransSpeed()=speed;
    return CANON_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////
CrclReturn CrclSubscriberDelegateInterface::SetAngleUnits(std::string unitName)
{
    RCS::CCanonCmd cc;
    cc.crclcommand = CanonCmdType::CANON_NOOP;
    cc.CrclCommandID() =  crclsettings.CommandID();
    RCS::cmds.addMsgQueue(cc);

    if (strncasecmp(unitName.c_str(), "RADIAN", unitName.size())==0) {
        crclsettings._angleUnit = CanonAngleUnit::RADIAN;
        crclsettings._angleConversion = 1.0;
    } else if (strncasecmp(unitName.c_str(), "DEGREE", unitName.size())==0) {
        crclsettings._angleUnit = CanonAngleUnit::DEGREE;
        crclsettings._angleConversion = M_PI / 180.0;
    } else {
        return CANON_FAILURE;
    }
    return CANON_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////
CrclReturn CrclSubscriberDelegateInterface::SetAxialSpeeds(std::vector<double> speeds)
{
    crclsettings._speeds.clear();
    copy(speeds.begin(), speeds.end(), std::back_inserter(crclsettings._speeds));

    RCS::CCanonCmd cc;
    cc.crclcommand = CanonCmdType::CANON_SET_MAX_JOINT_SPEED;
    cc.speed.clear();
    cc.CrclCommandID() = crclsettings.CommandID();
    copy(speeds.begin(), speeds.end(), std::back_inserter(cc.speed));
    RCS::cmds.addMsgQueue(cc);
    return CANON_MOTION;
}

////////////////////////////////////////////////////////////////////////////////
CrclReturn CrclSubscriberDelegateInterface::SetEndPoseTolerance(crcl::PoseToleranceType tolerance)
{
    crclsettings._endPoseTolerance = tolerance;

    RCS::CCanonCmd cc;
    cc.crclcommand = CanonCmdType::CANON_SET_TOLERANCE;
    cc.CrclCommandID() = crclsettings.CommandID();
    // FIXME
    //cc.endPoseTol=crclwm._endPoseTolerance;
    RCS::cmds.addMsgQueue(cc);

    return CANON_MOTION;
}

////////////////////////////////////////////////////////////////////////////////
CrclReturn CrclSubscriberDelegateInterface::SetForceUnits(std::string unitName)
{
    RCS::CCanonCmd cc;
    cc.crclcommand = CanonCmdType::CANON_NOOP;
    cc.CrclCommandID() =  crclsettings.CommandID();
    RCS::cmds.addMsgQueue(cc);


    if (strncasecmp(unitName.c_str(), "newton", unitName.size())==0) {
        crclsettings._forceUnit = CanonForceUnit::NEWTON;
        crclsettings._forceConversion = 1.0;
    } else if (strncasecmp(unitName.c_str(), "pound", unitName.size())==0) {
        // FN = Flbf × 4.4482216152605
        crclsettings._forceUnit = CanonForceUnit::POUND;
        crclsettings._forceConversion = 4.4482216152605;
    } else if (strncasecmp(unitName.c_str(), "ounce", unitName.size())==0) {
        crclsettings._forceUnit = CanonForceUnit::OUNCE;
        crclsettings._forceConversion = 0.2780138509534;
    } else {
        return CANON_FAILURE;
    }
    return CANON_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////
CrclReturn CrclSubscriberDelegateInterface::SetIntermediatePoseTolerance(crcl::PoseToleranceType tolerance)
{
    RCS::CCanonCmd cc;
    cc.crclcommand = CanonCmdType::CANON_NOOP;
    cc.CrclCommandID() =  crclsettings.CommandID();
    RCS::cmds.addMsgQueue(cc);


    crclsettings._intermediatePoseTolerance = tolerance;
    return CANON_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////
CrclReturn CrclSubscriberDelegateInterface::SetLengthUnits(std::string unitName)
{
    RCS::CCanonCmd cc;
    cc.crclcommand = CanonCmdType::CANON_NOOP;
    cc.CrclCommandID() =  crclsettings.CommandID();
    RCS::cmds.addMsgQueue(cc);


    if (strncasecmp(unitName.c_str(), "METER", unitName.size())==0) {
        crclsettings._lengthUnit = CanonLengthUnit::METER;
        crclsettings._lengthConversion = 1.0;
    } else if (strncasecmp(unitName.c_str(), "MM", unitName.size())==0) {
        crclsettings._lengthUnit = CanonLengthUnit::MM;
        crclsettings._lengthConversion = 0.001;
    } else if (strncasecmp(unitName.c_str(), "INCH", unitName.size())==0) {
        crclsettings._lengthUnit = CanonLengthUnit::INCH;
        crclsettings._lengthConversion = 0.0254;
    } else {
        return CANON_FAILURE;
    }
    return CANON_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////
CrclReturn CrclSubscriberDelegateInterface::SetMotionCoordination(bool bCoordinatedMotion)
{
    RCS::CCanonCmd cc;
    cc.crclcommand = CanonCmdType::CANON_NOOP;
    cc.CrclCommandID() =  crclsettings.CommandID();
    RCS::cmds.addMsgQueue(cc);


    crclsettings._bCoordinatedMotion = bCoordinatedMotion;
    return CANON_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////
CrclReturn CrclSubscriberDelegateInterface::SetParameter(char *paramName, void *paramVal)
{
    RCS::CCanonCmd cc;
    cc.crclcommand = CanonCmdType::CANON_NOOP;
    cc.CrclCommandID() =  crclsettings.CommandID();
    RCS::cmds.addMsgQueue(cc);

    ROS_DEBUG("CrclServerDelegateInterface::SetParameter NOT IMPLEMENTED");
    return CANON_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////
CrclReturn CrclSubscriberDelegateInterface::SetRelativeAcceleration(double percent)
{
    RCS::CCanonCmd cc;
    cc.crclcommand = CanonCmdType::CANON_NOOP;
    cc.CrclCommandID() =  crclsettings.CommandID();
    RCS::cmds.addMsgQueue(cc);

    crclsettings.Rates().CurrentRotAcceleration()=percent*crclsettings.Rates().CurrentRotAcceleration();
    crclsettings.Rates().CurrentTransAcceleration()=percent*crclsettings.Rates().CurrentTransAcceleration();
    return CANON_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////
CrclReturn CrclSubscriberDelegateInterface::SetEndEffector(double percent)
{
    RCS::CCanonCmd cc;
    cc.crclcommand = CanonCmdType::CANON_SET_GRIPPER;
    cc.CrclCommandID() =  crclsettings.CommandID();
    // FIXME: validate that percent is between 0..1
    cc.eepercent = percent;
    RCS::cmds.addMsgQueue(cc);
    return CANON_MOTION;
}

////////////////////////////////////////////////////////////////////////////////
CrclReturn CrclSubscriberDelegateInterface::SetEndEffectorTolerance(crcl::PoseToleranceType dTolerance)
{
    RCS::CCanonCmd cc;
    cc.crclcommand = CanonCmdType::CANON_NOOP;
    cc.CrclCommandID() =  crclsettings.CommandID();
    RCS::cmds.addMsgQueue(cc);

    crclsettings._gripperPoseTolerance = dTolerance;
    return CANON_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////
CrclReturn CrclSubscriberDelegateInterface::StopMotion(int nCondition)
{
    RCS::CCanonCmd cc;
    cc.crclcommand = CanonCmdType::CANON_STOP_MOTION;
    cc.CrclCommandID() =  crclsettings.CommandID();
    cc.stoptype =  nCondition;
    RCS::cmds.addMsgQueue(cc);
    return CANON_MOTION;
}

////////////////////////////////////////////////////////////////////////////////
CrclReturn CrclSubscriberDelegateInterface::SetRotAccel(double accel)
{
    RCS::CCanonCmd cc;
    cc.crclcommand = CanonCmdType::CANON_NOOP;
    cc.CrclCommandID() =  crclsettings.CommandID();
    RCS::cmds.addMsgQueue(cc);

    crclsettings.Rates().CurrentRotAcceleration()=accel*crclsettings._angleConversion;
    return CANON_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////
CrclReturn CrclSubscriberDelegateInterface::SetRotSpeed(double speed)
{
    RCS::CCanonCmd cc;
    cc.crclcommand = CanonCmdType::CANON_NOOP;
    cc.CrclCommandID() =  crclsettings.CommandID();
    RCS::cmds.addMsgQueue(cc);

    crclsettings.Rates().CurrentRotSpeed()=speed*crclsettings._angleConversion;
    return CANON_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////
CrclReturn CrclSubscriberDelegateInterface::SetTorqueUnits(std::string unitName)
{
    RCS::CCanonCmd cc;
    cc.crclcommand = CanonCmdType::CANON_NOOP;
    cc.CrclCommandID() =  crclsettings.CommandID();
    RCS::cmds.addMsgQueue(cc);

    //   newtonMeter,   footPound
    // 1.35581794833 × F * lbf-ft = F * N⋅m
    if (strncasecmp(unitName.c_str(), "newtonMeter", unitName.size())==0) {
        crclsettings._torqueUnit = CanonTorqueUnit::NEWTONMETER;
        crclsettings._torqueConversion = 1.0;
    } else if (strncasecmp(unitName.c_str(), "footPound", unitName.size())==0) {
        crclsettings._torqueUnit = CanonTorqueUnit::FOOTPOUND;
        crclsettings._torqueConversion = 1.35581794833;
    } else {
        return CANON_FAILURE;
    }
    return CANON_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////
CrclReturn CrclSubscriberDelegateInterface::SetTransAccel(double accel)
{
    RCS::CCanonCmd cc;
    cc.crclcommand = CanonCmdType::CANON_NOOP;
    cc.CrclCommandID() =  crclsettings.CommandID();
    RCS::cmds.addMsgQueue(cc);

    crclsettings.Rates().CurrentTransAcceleration()=accel*crclsettings._lengthConversion;
    return CANON_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////
CrclReturn CrclSubscriberDelegateInterface::SetTransSpeed(double speed)
{
    RCS::CCanonCmd cc;
    cc.crclcommand = CanonCmdType::CANON_NOOP;
    cc.CrclCommandID() =  crclsettings.CommandID();
    RCS::cmds.addMsgQueue(cc);

    crclsettings.Rates().CurrentTransSpeed()=speed*crclsettings._lengthConversion;
    return CANON_SUCCESS;
}

