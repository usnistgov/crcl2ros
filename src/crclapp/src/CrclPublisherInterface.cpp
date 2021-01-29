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
#include <map>
// xercesc headers
#include <xercesc/dom/DOM.hpp>
#include <xercesc/util/PlatformUtils.hpp>
#include <xercesc/framework/XMLGrammarPoolImpl.hpp>

// boost headers
#include <boost/regex.hpp>
#include <boost/exception/all.hpp>

// crcl headers
#include "crclapp/CrclPublisherInterface.h"
#include "crclapp/RosMsgHandler.h"

#include "rcs/RCSMsgQueue.h"
#include "rcs/Conversions.h"

using namespace xsd;
using namespace xml_schema;
using namespace crcl;
using namespace RCS;

unsigned long long CrclPublisherCmdInterface::_commandnum = 0;

////////////////////////////////////////////////////////////////////////////////
std::string CrclPublisherCmdInterface::GetStatusReply(CrclStatus *wm)
{
    CommandStatusType cmdstatus(wm->CommandID(), wm->CommandID(),wm->CommandStatus());
    CRCLStatusType status(cmdstatus);

    status.CommandStatus(cmdstatus);

    CRCLStatusType::JointStatuses_type jointStatuses;
    jointStatuses.JointStatus(wm->_CurrentJoints);
    status.JointStatuses(jointStatuses);

    status.PoseStatus(wm->_CurrentPose);

    if(!Globals.bReadAllInstances)
        return "";
    // Extension: Model information reported in CRCL
    CRCLStatusType::ModelStatus_sequence model_seq;

    std::map<std::string, tf::Pose>::iterator it= wm->_ModelStates.begin();
    for(;it!=wm->_ModelStates.end();++it)
    {
        std::string modelname=(*it).first;
        // Convert from tf pose to crcl pose
        PoseType crclpose = Convert((*it).second);

        // create crcl model (name/crcl pose)
        ModelsStatusType m(modelname,crclpose);


        std::vector<CrclStatus::inference_type>  &props = wm->_ModelInferences[(*it).first];
        map::item_sequence  seq;
        size_t n=fmin(props.size(), 1);
        if(n==0)
        {
            int m=0;
        }
        for(size_t j=0; j< props.size(); j++)
        {
 //           map::item_sequence & seq = crclprop.get().map().item(); // .key(std::string("hihi"));

            CrclStatus::inference_type &inference=props[j];            // key, value, name
            // key, value, name
            MapItemType mi("name", inference.name,inference.name);
            seq.push_back(mi);
            if(!inference.type.empty())
            {
                if(Globals.dbgModelStatus)
                {
                    if(inference.type.empty()||
                            inference.location.empty() ||
                            inference.state.empty())
                    {
                        std::cout << "Error: inference field empty - should never be here \n";
                        std::cout << "\tcheck that model is part of robot \n";

                    }
                }
                // slot - properties are type (sm,med,lg),
                // location - reoriented slot xyz location based on tray rotation
                // state - open or gear name in the slot
                MapItemType mi1("type", inference.type,inference.name);
                mi1.key("type");
                mi1.value(inference.type);
                mi1.name(inference.name);
                seq.push_back(mi1);

                MapItemType mi2("location", inference.location,inference.name);
                mi2.key("location");
                mi2.value(inference.location);
                mi2.name(inference.name);
                seq.push_back(mi2);

                MapItemType mi3("state", inference.state,inference.name);
                mi2.key("state");
                mi2.value(inference.state);
                mi2.name(inference.name);
                seq.push_back(mi3);

            }
            else
            {
                // gear - properties are parent tray and slot name
                MapItemType mi1("parent", inference.parent,inference.name);
                seq.push_back(mi1);
                MapItemType mi2("slot", inference.slot,inference.name);
                seq.push_back(mi2);
            }
        }

        map ma;  // xsdxml map
        ma.item(seq);  // map item is assigned seq
        MapType mt(ma);  // maptype assign xsdxml map
        // now assign maptype as properties for this status item
        ModelsStatusType::Properties_optional  crclprop (mt);

        // save vector of properties for this model
        m.Properties(crclprop);

        // save model status (name,pose,properties) as crcl status
        model_seq.push_back(m);
    }
    status.ModelStatus()=model_seq;

    xml_schema::namespace_infomap map;
    std::ostringstream strfs;


    CRCLStatus(strfs, // or std::cout,
            (CRCLStatusType &) status,
            map,
            "UTF-8",
            xml_schema::flags::dont_initialize);

    // fix: replace nan with somthing
    std::string sStatus= strfs.str();
    // bad hack to overcome bombing for now....
    sStatus= Globals.replaceAll(sStatus, "nan", "0.0");
    if(Globals.dbgModelStatus)
        std::cout << sStatus;
    return sStatus;


}

////////////////////////////////////////////////////////////////////////////////
std::string CrclPublisherCmdInterface::CloseToolChanger() {
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
std::string CrclPublisherCmdInterface::Dwell(double time ){ // int *events, double *params, int numEvents) {
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
std::string CrclPublisherCmdInterface::EndCanon(int reason) {
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
std::string CrclPublisherCmdInterface::GetStatus() {
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
std::string CrclPublisherCmdInterface::InitCanon() {
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
std::string CrclPublisherCmdInterface::Message(std::string message) {
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
std::string CrclPublisherCmdInterface::MoveScrew(crcl::PoseType startPose, crcl::VectorType axisPoint, double dAxialDistanceFree, double dAxialDistanceScrew, double dTurn) {
    assert(0);
    return "";
}

////////////////////////////////////////////////////////////////////////////////
std::string CrclPublisherCmdInterface::MoveTo(crcl::PoseType pose, bool bStraight) {
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
std::string CrclPublisherCmdInterface::MoveThroughTo(crcl::PoseType *poses,
        int numPoses,
        double * accelerations,
        double * speeds,
        crcl::PoseToleranceType * tolerances) {
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
std::string CrclPublisherCmdInterface::OpenToolChanger() {
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
std::string CrclPublisherCmdInterface::RunProgram(std::string programText) {
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
std::string CrclPublisherCmdInterface::SetEndEffector(double fractionalSetting) {
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
std::string CrclPublisherCmdInterface::SetEndEffectorTolerance(crcl::PoseToleranceType toleranceSetting) {
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
std::string CrclPublisherCmdInterface::SetEndPoseTolerance(crcl::PoseToleranceType toleranceSetting) {
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
std::string CrclPublisherCmdInterface::SetAngleUnits(std::string sUnitName) {
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
std::string CrclPublisherCmdInterface::SetLengthUnits(std::string sUnitName) // UnitName is a string that can be only the literals 'meter',  'millimeter', or 'inch'.
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
std::string CrclPublisherCmdInterface::SetMotionCoordination(bool bCoordinated) {
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

std::string CrclPublisherCmdInterface::SetRotAccel(double dAccel) {
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
std::string CrclPublisherCmdInterface::SetRotSpeed(double dSpeed) {
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
std::string CrclPublisherCmdInterface::StopMotion(int condition) // 0=Immediate, Fast, Normal
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

    Globals.readFile(filename, contents);
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
        ROS_DEBUG(e.what());
        return CANON_FAILURE;
    }    catch (...) {
        ROS_DEBUG("ParseCRCLStatusString error");
        return CANON_FAILURE;
    }
    return CANON_SUCCESS;
}
