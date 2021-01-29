// CrclInterface.cpp

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
#include <string>
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
#include "crcllib/CrclConfig.h"
#include "crcllib/CrclInterface.h"
#include "crcllib/Crcl2Rcs.h"

#include "nist/RCSMsgQueue.h"
#include "nist/Logger.h"
#include "nist/Conversions.h"


using namespace xsd;
using namespace xml_schema;
using namespace Crcl;
using namespace RCS;

unsigned long long CrclClientCmdInterface::_commandnum = 0;

static std::string replace_all(std::string str, std::string oldstr, std::string newstr)
{
    std::string::size_type n = 0;

    while ( ( n = str.find( oldstr, n ) ) != std::string::npos )
    {
        str.replace( n, oldstr.size(), newstr );
        n += newstr.size();
    }

    return str;
}

static std::string trim (std::string source, std::string delims = " \t\r\n")
{
    std::string result = source.erase(source.find_last_not_of(delims) + 1);
    return result.erase(0, result.find_first_not_of(delims));
}

static bool ReadFile (std::string filename, std::string & contents)
{
    std::ifstream     in(filename.c_str( ), std::ifstream::in );
    std::stringstream buffer;

    if(!in.is_open())
    {
        logFatal("CGlobals::ReadFile failed file does not exist %s\n" , filename.c_str());

    }
    buffer << in.rdbuf( );
    contents = buffer.str( );
    return true;
}

////////////////////////////////////////////////////////////////////////////////
std::string CrclClientCmdInterface::GetStatusReply(CrclStatus *wm)
{
    // Log status for now
    static std::string last_status;
    std::stringstream now_status;
    now_status << "GetStatusReply=" << wm->CommandID() << ":" << wm->CommandStatus() << std::endl;
    if(now_status.str() != last_status)
    {
        last_status=now_status.str();
        logStatus(last_status.c_str());
    }

    CommandStatusType status(wm->CommandID(), wm->CommandID(),wm->CommandStatus());
    CRCLStatusType cmd(status);

    cmd.CommandStatus(status);

    CRCLStatusType::JointStatuses_type jointStatuses;
    jointStatuses.JointStatus(wm->_CurrentJoints);
    cmd.JointStatuses(jointStatuses);

    cmd.PoseStatus(wm->_CurrentPose);

    xml_schema::namespace_infomap map;
    std::ostringstream strfs;
    CRCLStatus(strfs, // or std::cout,
            (CRCLStatusType &) cmd,
            map,
            "UTF-8",
            xml_schema::flags::dont_initialize);

    // fix: replace nan with somthing
    std::string sStatus= strfs.str();
    // bad hack to overcome bombing for now....
    sStatus=replace_all(sStatus, "nan", "0.0");
    return sStatus;


}

////////////////////////////////////////////////////////////////////////////////
std::string CrclClientCmdInterface::CloseToolChanger() {
    CloseToolChangerType cmd(++_commandnum);
    xml_schema::namespace_infomap map;
    std::ostringstream strfs;

    CRCLCommandInstance(strfs, // std::cout,
            (CRCLCommandType &) cmd,
            map,
            "UTF-8",
            xml_schema::flags::dont_initialize);
    return strfs.str();
}

////////////////////////////////////////////////////////////////////////////////
std::string CrclClientCmdInterface::Dwell(double time ){ // int *events, double *params, int numEvents) {
   std::ostringstream strfs;
 #if 0
    if (numEvents < 1) {
        return "";
    }
    DwellType cmd(++_commandnum, *params);
    xml_schema::namespace_infomap map;
    CRCLCommandInstance(strfs, // std::cout,
            (CRCLCommandType &) cmd,
            map,
            "UTF-8",
            xml_schema::flags::dont_initialize);
#endif
    return strfs.str();
}

////////////////////////////////////////////////////////////////////////////////
std::string CrclClientCmdInterface::EndCanon(int reason) {
    EndCanonType cmd(++_commandnum);
    xml_schema::namespace_infomap map;
    std::ostringstream strfs;

    CRCLCommandInstance(strfs, // std::cout,
            (CRCLCommandType &) cmd,
            map,
            "UTF-8",
            xml_schema::flags::dont_initialize);
    return strfs.str();
}

/**
 * xercesc::XMLPlatformUtils::Initialize();
 * CrclClientCmdInterface crcl;
 * std::string text = crcl.CRCLGetStatusCmd();
 * xercesc::XMLPlatformUtils::Terminate ();
 */
////////////////////////////////////////////////////////////////////////////////
std::string CrclClientCmdInterface::GetStatus() {
    GetStatusType cmd(++_commandnum);
    xml_schema::namespace_infomap map;

    // map[""].name = "http://www.w3.org/2001/XMLSchema";
    // map[""].schema = "http://www.w3.org/2001/XMLSchema ../CRCLStatus.xsd";
    std::ostringstream strfs;

    CRCLCommandInstance(strfs, // std::cout,
            (CRCLCommandType &) cmd,
            map,
            "UTF-8",
            xml_schema::flags::dont_initialize);
    return strfs.str();
}

////////////////////////////////////////////////////////////////////////////////
std::string CrclClientCmdInterface::InitCanon() {
    InitCanonType cmd(++_commandnum);
    xml_schema::namespace_infomap map;
    std::ostringstream strfs;

    CRCLCommandInstance(strfs, // std::cout,
            (CRCLCommandType &) cmd,
            map,
            "UTF-8",
            xml_schema::flags::dont_initialize);
    return strfs.str();
}

////////////////////////////////////////////////////////////////////////////////
std::string CrclClientCmdInterface::Message(std::string message) {
    MessageType cmd(++_commandnum, message);
    xml_schema::namespace_infomap map;
    std::ostringstream strfs;

    CRCLCommandInstance(strfs,
            (CRCLCommandType &) cmd,
            map,
            "UTF-8",
            xml_schema::flags::dont_initialize);
    return strfs.str();
}

////////////////////////////////////////////////////////////////////////////////
std::string CrclClientCmdInterface::MoveScrew(Crcl::PoseType startPose, Crcl::VectorType axisPoint, double dAxialDistanceFree, double dAxialDistanceScrew, double dTurn) {
    assert(0);
    return "";
}

////////////////////////////////////////////////////////////////////////////////
std::string CrclClientCmdInterface::MoveTo(Crcl::PoseType pose, bool bStraight) {
    // ::PoseType mypose (::PoseType(::PointType(pose.x, pose.y, pose.z),VectorType(1.0,0.0,0.0), VectorType(1.0,0.0,0.0) ));
    MoveToType cmd(++_commandnum, bStraight, pose);

    // cmd.EndPosition(pose);
    xml_schema::namespace_infomap map;
    std::ostringstream strfs;

    CRCLCommandInstance(strfs,
            (CRCLCommandType &) cmd,
            map,
            "UTF-8",
            xml_schema::flags::dont_initialize);
    return strfs.str();
}

////////////////////////////////////////////////////////////////////////////////
std::string CrclClientCmdInterface::MoveThroughTo(Crcl::PoseType *poses,
        int numPoses,
        double * accelerations,
        double * speeds,
        Crcl::PoseToleranceType * tolerances) {
    std::ostringstream strfs;

    // FIXME:
    assert(0);
#if 0
    MoveThroughToType::Waypoint_sequence wpt;

    for (int i = 0; i < numPoses; i++) {
        ::PoseType pose(::PoseType(::PointType(poses[i].x, poses[i].y, poses[i].z), VectorType(1.0, 0.0, 0.0), VectorType(1.0, 0.0, 0.0)));
        wpt.push_back(pose);
    }
    MoveThroughToType cmd(++_commandnum, true, numPoses);
    cmd.Waypoint(wpt);
    xml_schema::namespace_infomap map;
    CRCLCommandInstance(strfs,
            (CRCLCommandType &) cmd,
            map,
            "UTF-8",
            xml_schema::flags::dont_initialize);
#endif
    return strfs.str();
}

////////////////////////////////////////////////////////////////////////////////
std::string CrclClientCmdInterface::OpenToolChanger() {
    OpenToolChangerType cmd(++_commandnum);
    xml_schema::namespace_infomap map;
    std::ostringstream strfs;

    CRCLCommandInstance(strfs,
            (CRCLCommandType &) cmd,
            map,
            "UTF-8",
            xml_schema::flags::dont_initialize);
    return strfs.str();
}

////////////////////////////////////////////////////////////////////////////////
std::string CrclClientCmdInterface::RunProgram(std::string programText) {
    RunProgramType cmd(++_commandnum, programText);
    xml_schema::namespace_infomap map;
    std::ostringstream strfs;

    CRCLCommandInstance(strfs,
            (CRCLCommandType &) cmd,
            map,
            "UTF-8",
            xml_schema::flags::dont_initialize);
    return strfs.str();
}

////////////////////////////////////////////////////////////////////////////////
std::string CrclClientCmdInterface::SetEndEffector(double fractionalSetting) {
    SetEndEffectorType cmd(++_commandnum, fractionalSetting);
    xml_schema::namespace_infomap map;
    std::ostringstream strfs;

    CRCLCommandInstance(strfs,
            (CRCLCommandType &) cmd,
            map,
            "UTF-8",
            xml_schema::flags::dont_initialize);
    return strfs.str();
}

////////////////////////////////////////////////////////////////////////////////
std::string CrclClientCmdInterface::SetEndEffectorTolerance(Crcl::PoseToleranceType toleranceSetting) {
    std::ostringstream strfs;

    // FIXME:
    assert(0);
#if 0

    // SetEndPoseToleranceType::Tolerance_type tolerancepose;
    // tolerancepose.XPointTolerance(toleranceSetting.x);
    // tolerancepose.YPointTolerance(toleranceSetting.y);
    // tolerancepose.ZPointTolerance(toleranceSetting.z);
    SetEndPoseToleranceType cmd(++_commandnum, toleranceSetting);
    xml_schema::namespace_infomap map;
    CRCLCommandInstance(strfs,
            (CRCLCommandType &) cmd,
            map,
            "UTF-8",
            xml_schema::flags::dont_initialize);
#endif
    return strfs.str();
}

////////////////////////////////////////////////////////////////////////////////
std::string CrclClientCmdInterface::SetEndPoseTolerance(Crcl::PoseToleranceType toleranceSetting) {
    SetEndPoseToleranceType cmd(++_commandnum, toleranceSetting);
    xml_schema::namespace_infomap map;
    std::ostringstream strfs;

    CRCLCommandInstance(strfs,
            (CRCLCommandType &) cmd,
            map,
            "UTF-8",
            xml_schema::flags::dont_initialize);
    return strfs.str();
}

////////////////////////////////////////////////////////////////////////////////
std::string CrclClientCmdInterface::SetAngleUnits(std::string sUnitName) {
    ::AngleUnitEnumType eUnitName = ::AngleUnitEnumType::radian;

    if (strncasecmp(sUnitName.c_str(), "radian", strlen("radian"))==0) {
        eUnitName = ::AngleUnitEnumType::radian;
    } else if (strncasecmp(sUnitName.c_str(), "degree", strlen("degree"))==0) {
        eUnitName = ::AngleUnitEnumType::degree;
    }
    SetAngleUnitsType cmd(++_commandnum, eUnitName);
    xml_schema::namespace_infomap map;
    std::ostringstream strfs;
    CRCLCommandInstance(strfs,
            (CRCLCommandType &) cmd,
            map,
            "UTF-8",
            xml_schema::flags::dont_initialize);
    return strfs.str();
}

////////////////////////////////////////////////////////////////////////////////
std::string CrclClientCmdInterface::SetLengthUnits(std::string sUnitName) // UnitName is a string that can be only the literals 'meter',  'millimeter', or 'inch'.
{
    ::LengthUnitEnumType eUnitName = ::LengthUnitEnumType::meter;

    if (strncasecmp(sUnitName.c_str(), "meter", strlen("meter"))==0) {
        eUnitName = ::LengthUnitEnumType::meter;
    } else if (strncasecmp(sUnitName.c_str(), "millimeter", strlen("millimeter"))==0) {
        eUnitName = ::LengthUnitEnumType::millimeter;
    } else if (strncasecmp(sUnitName.c_str(), "inch", strlen("inch"))==0) {
        eUnitName = ::LengthUnitEnumType::inch;
    }
    SetLengthUnitsType cmd(++_commandnum, eUnitName);
    xml_schema::namespace_infomap map;
    std::ostringstream strfs;
    CRCLCommandInstance(strfs,
            (CRCLCommandType &) cmd,
            map,
            "UTF-8",
            xml_schema::flags::dont_initialize);
    return strfs.str();
}

////////////////////////////////////////////////////////////////////////////////
std::string CrclClientCmdInterface::SetMotionCoordination(bool bCoordinated) {
    SetMotionCoordinationType cmd(++_commandnum, bCoordinated);
    xml_schema::namespace_infomap map;
    std::ostringstream strfs;

    CRCLCommandInstance(strfs,
            (CRCLCommandType &) cmd,
            map,
            "UTF-8",
            xml_schema::flags::dont_initialize);
    return strfs.str();
}

std::string CrclClientCmdInterface::SetRotAccel(double dAccel) {
    SetRotAccelType::RotAccel_type accel;

    accel.Name("");
    SetRotAccelType cmd(++_commandnum, accel);
    xml_schema::namespace_infomap map;
    std::ostringstream strfs;
    CRCLCommandInstance(strfs,
            (CRCLCommandType &) cmd,
            map,
            "UTF-8",
            xml_schema::flags::dont_initialize);
    return strfs.str();
}

////////////////////////////////////////////////////////////////////////////////
std::string CrclClientCmdInterface::SetRotSpeed(double dSpeed) {
    SetRotSpeedType::RotSpeed_type speed;

    speed.Name("");
    SetRotSpeedType cmd(++_commandnum, speed);
    xml_schema::namespace_infomap map;
    std::ostringstream strfs;
    CRCLCommandInstance(strfs,
            (CRCLCommandType &) cmd,
            map,
            "UTF-8",
            xml_schema::flags::dont_initialize);
    return strfs.str();
}

////////////////////////////////////////////////////////////////////////////////
std::string CrclClientCmdInterface::StopMotion(int condition) // 0=Immediate, Fast, Normal
{
    ::StopConditionEnumType eStopCondition = ::StopConditionEnumType::Normal;

    if (condition == 0) {
        eStopCondition = ::StopConditionEnumType::Immediate;
    } else if (condition == 1) {
        eStopCondition = ::StopConditionEnumType::Fast;
    } else if (condition == 2) {
        eStopCondition = ::StopConditionEnumType::Normal;
    }

    StopMotionType cmd(++_commandnum, eStopCondition);
    xml_schema::namespace_infomap map;
    std::ostringstream strfs;
    CRCLCommandInstance(strfs,
            (CRCLCommandType &) cmd,
            map,
            "UTF-8",
            xml_schema::flags::dont_initialize);
    return strfs.str();
}

// -------------------------------------------------------------
//  CrclStatusMsgInterface

////////////////////////////////////////////////////////////////////////////////
CrclReturn CrclStatusMsgInterface::ParseCRCLStatus(std::string filename) {
    std::string contents;

    ReadFile(filename, contents);
    if(contents.empty())
        return CANON_FAILURE;
    return ParseCRCLStatusString(contents);
}

////////////////////////////////////////////////////////////////////////////////
CrclReturn CrclStatusMsgInterface::ParseCRCLStatusString(std::string str) {
    try {
        std::istringstream istr(str);
        std::auto_ptr<CRCLStatusType> crclstat(
                CRCLStatus(istr, xml_schema::flags::dont_initialize | xml_schema::flags::dont_validate | xml_schema::flags::keep_dom)
                );

        // xercesc::DOMElement* e = static_cast<xercesc::DOMElement*> ((*crclstat)._node ());
        CLEANSTORE(_status.CommandID(), crclstat->CommandStatus().CommandID(), 0);
        CLEANSTORE(_status.StatusID(), crclstat->CommandStatus().StatusID(), 0);
        CLEANSTORE(_status.CommandStatus(), crclstat->CommandStatus().CommandState(), "");
        VALIDSTORE(_status._CurrentPose, (*crclstat->PoseStatus()).Pose()); // no change if not valid
        // _status.Dump();
    }    catch (const xml_schema::exception & e) {
        logError(e.what());
        return CANON_FAILURE;
    }    catch (...) {
        logError("ParseCRCLStatusString error\n");
        return CANON_FAILURE;
    }
    return CANON_SUCCESS;
}
