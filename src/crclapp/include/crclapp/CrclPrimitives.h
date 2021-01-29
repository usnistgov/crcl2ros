// CrclPrimitives.h

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

#pragma once
#include <map>

#define _USE_MATH_DEFINES
#include <math.h>       /* isnan, sqrt */
#include "rcs/IRcs.h"

// CRCL CodeSynthesis XML interface
//#undef _GLIBCXX_DEPRECATED

#include "CrclXsd/DataPrimitives.hxx"
#include "CrclXsd/CRCLCommands.hxx"
#include "CrclXsd/CRCLStatus.hxx"
#include "CrclXsd/CRCLCommandInstance.hxx"
#include "CrclXsd/CRCLProgramInstance.hxx"

#include "crclapp/CrclWm.h"

/**
 * These class and routines are for representation, conversion to/from CRCL
 * to ROS tf.
 *
 * the CRCL data structures are based on the CodeSynthesis generation of C++ header/source
 * from the CRCL XSD. For example ::PoseType is mapped into crcl::PoseType.  THis in turn
 * has conversion to/from tf::Pose.
 *
 * Likewise ROS sensor_msgs::JointState is mapped into CodeSynthesis ::JointStatusesType::JointStatus_sequence
 * which is mapped into CRCL crcl::JointStatusSequence.
 */
namespace crcl {
    struct CrclStatus;
    struct GripperStatus;
    struct JointReport;

    enum CRCLCmdStatus {
        CRCL_Done = 0,
        CRCL_Error,
        CRCL_Working,
        CRCL_Ready
    };

    enum CrclReturn {
        CANON_REJECT = -2,
        CANON_FAILURE = -1,
        CANON_SUCCESS = 0,
        CANON_STATUSREPLY = 1,
        CANON_MOTION = 2,
        CANON_RUNNING,
        CANON_NOT_IMPLEMENTED
    };

    typedef ::ActuateJointsType::ActuateJoint_sequence ActuatorJointSequence;
    typedef ::PoseType PoseType;
    typedef ::JointStatusType JointStatus;
    typedef ::CommandStateEnumType CommandStateEnum;
    typedef ::PointType PointType;
    typedef ::VectorType VectorType;
    typedef ::CommandStateEnumType CommandStateEnum;
    typedef ::JointStatusesType::JointStatus_sequence JointStatusSequence;
    typedef ::PoseToleranceType PoseToleranceType;
    typedef tf::Vector3 Vector3D;

    /*!
     * \brief ConvertToPositionVector converts codesynthesis actuator sequence into std vector of position as doubles.
     * \param  codesynthesis sequence of joints.
     * \param  conversion factor for each joint (e.g., degree to radian).
     * \return std vector of doubles representing position.
     */
    std::vector<double> ConvertToPositionVector(ActuatorJointSequence &, double dConversion);

    /*!
     * \brief Convert converts codesynthesis commanded actuator joint sequence into status actuator joint sequence .
     * \param  codesynthesis sequence of commanded joints.
     * \return codesynthesis sequence of status joints.
     */
    JointStatusSequence Convert(ActuatorJointSequence jin);

    /*!
     * \brief Convert converts codesynthesis status actuator joint sequence into ROS JointState.(Fills position).
     * \param  codesynthesis sequence of status joints.
     * \param  conversion factor for each joint (e.g., degree to radian).
     * \return ROS JointState with Joint positions filled.
     */
    sensor_msgs::JointState Convert(crcl::JointStatusSequence jointStatusSeq, double angleConversion = 1.0);

    /*!
     * \brief Convert converts ROS JointState  (primarily position for now) and creates codesynthesis status actuator joint sequence.
     * \param ROS JointState with Joint positions filled.
     * \return  codesynthesis sequence of status joints.
     */
    crcl::JointStatusSequence Convert(sensor_msgs::JointState joints);

    /*!
     * \brief Convert converts ROS/RCS pose into codesynthesis Crcl Pose.
     * Note, no angle conversion - rotation/orientation always in radians
     * \param  ROS/RCS pose.
     * \return  codesynthesis CRCL pose type.
     */
    crcl::PoseType Convert(tf::Pose pose);

    /*!
     * \brief Convert converts codesynthesis Crcl Pose into ROS/RCS pose using length conversion .
     * Note, no angle conversion - rotation/orientation always in radians
     * \param  codesynthesis CRCL pose type.
     * \param lengthConversion to make length in meters. (e.g., if mm input, conversion = 0.001).
     * \return  ROS/RCS pose.
     */
    tf::Pose Convert(crcl::PoseType & pose, double lengthConversion = 1.0);

    /*!
     * \brief Convert converts codesynthesis Crcl rotation (as x,z rotation vectors) into ROS/RCS rotation.
     * \param  codesynthesis CRCL x,z rotation vectors as used in pose.
     * \return   ROS/RCS rotation.
     */
    tf::Quaternion Convert(tf::Vector3 Xrot, tf::Vector3 Zrot);

    /*!
     * \brief Get roll,pitch,yaw orientation from codesynthesis Crcl pose.
     * \param  codesynthesis CRCL pose.
     * \param  roll,pitch, yaw reference to doubles that will be filled with angles in radians.
     * \return   true if sucessful, false otherwise.
     */
    bool GetPoseToRPY(crcl::PoseType & pose, double & dRoll, double & dPitch, double & dYaw);


    RCS::PoseTolerance Convert(crcl::PoseToleranceType);

    /*!
     * \brief Dump contents of codesynthesis Crcl pose.
     * \param  codesynthesis CRCL pose.
     * \param  separator defines character to use a separater between values (e.g., "," for csv).
     * \return   string with Crcl pose contents.
     */
    //std::string DumpCrclPose(crcl::PoseType pose, std::string separator=",");
    std::string DumpPosition(crcl::PoseType pose, std::string separator = ",");
    std::string DumpRotationAsCrcl(tf::Pose rcspose, std::string separator= ",");
    std::string DumpRotationAsCrcl(crcl::PoseType crclpose, std::string separator = ",");
    std::string DumpPose(crcl::PoseType pose, std::string separator = ",");
    std::string DumpStatusReply(crcl::CrclStatus *wm);
    std::string DumpCrclCommand(::CRCLCommandType & crclCommand);
    std::string DumpCrclJoints(crcl::JointStatusSequence);

    /*!
     * \brief Create codesynthesis Crcl pose that is necessary for all codesynthesis Crcl pose constructors.
     * \return  empty codesynthesis Crcl pose.
     */
    PoseType PoseHome();
    crcl::PoseType NullPose();

    /**
     * \brief GripperStatus  dummy class for future gripper information. 
     */
    struct GripperStatus {
         VAR( std::string, Name );  // 0..1
         VAR(double, Position );  // 0..1
    };
    
    /**
     * \brief JointReport  dummy class for future customization of Crcl status reports. 
     */
    struct JointReport {
        size_t _nJointNumber;
        bool _bReportPosition;
        bool _bReportTorqueOrForce;
        bool _bReportVelocity;
    };

    /**
     * \brief CrclStatus  is a class that encapsulates all the CRCL information. 
     * All lot of the knowledge is converting ROS oriented RCS data into codesynthesis Crcl representation and vice versa.
     * CrclStatus maintains the unit a crcl session uses to transmit robot commands,
     * latest robot status, etc. 
     */
    struct CrclStatus {
        CrclStatus();
        crcl::JointStatusSequence JointsHome();
 
        GripperStatus gripper;
        //        void                        Update (unsigned long long CommandID,
        //            crcl::CommandStateEnum                             state,
        //            crcl::JointStatusSequence &                        joints,
        //            crcl::PoseType &                                   _pose);
        void Update(unsigned long long CommandID);
        void Update(crcl::CommandStateEnum state);
        void Update(crcl::JointStatusSequence & joints, int type = RCS::TrajPointType::WAYPOINT);
        void Update(crcl::PoseType & pose, int type = RCS::TrajPointType::WAYPOINT);
        void Update(sensor_msgs::JointState & joints);
        void Update(tf::Pose & pose);
        void Update(std::string name, tf::Pose & pose);

        void Update(WorldModel::CShape & shape);

        VAR(unsigned long long, CommandID);
        VAR(unsigned long long, StatusID);
        VAR(crcl::CommandStateEnum, CommandStatus);
        VAR(RCS::IRate, Rates);
        VAR( GripperStatus, Gripper);

        // //////////////////////////////////////////////////
        crcl::PoseType _CurrentPose, _GoalPose;
        crcl::JointStatusSequence _GoalJoints;
        crcl::JointStatusSequence _CurrentJoints;
        std::vector<double> _speeds; // for each axis
        double _translationSpeed;
        double _translationAccel;
        double _rotSpeed;
        double _rotAccel;
        bool _bCoordinatedMotion;
        struct inference_type
        {
            std::string name;
            std::string type;
            std::string state;
            std::string location;

            std::string parent;
            std::string slot;
        } ;

        std::map<std::string, tf::Pose  >  _ModelStates;
        std::map<std::string, std::vector<inference_type>  >  _ModelInferences;

        // Conversion variables
        int _lengthUnit;
        double _lengthConversion;

        int _angleUnit;
        double _angleConversion;

        int _forceUnit;
        double _forceConversion;

        int _torqueUnit;
        double _torqueConversion;

        // Tolerance data structures
        //std::vector<crcl::PoseToleranceType> _intermediatePoseTolerance; // one only
        crcl::PoseToleranceType _endPoseTolerance;
        crcl::PoseToleranceType _gripperPoseTolerance;
        crcl::PoseToleranceType _intermediatePoseTolerance;

        std::vector<JointReport> _vJointReport;
        std::string sCommandState;
        std::string Alarm;
        std::vector<std::string> jointnames; /**<  vector of joint names used by command */

    };
};
