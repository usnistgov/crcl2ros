// RCS.h

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

#ifndef _RCS_H
#define _RCS_H

#include <stdint.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>

// Protobuf generated messages
#ifdef INCROSMSG
#include <crcl_rosmsgs/CrclCommandMsg.h>
#include <crcl_rosmsgs/CrclStatusMsg.h>
#include <crcl_rosmsgs/CrclMaxProfileMsg.h>
#endif

#include "rcs/Core.h"
#include "rcs/Debug.h"
#include "rcs/Conversions.h"
#include <rcs/RCSPriorityQueue.h>

// ROS types
#include <tf/tf.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/ColorRGBA.h>

//typedef sensor_msgs::JointState JointState;
//typedef std::shared_ptr<JointState> JointStateSharedPtr;


// These are defined as macros somewhere
#undef max
#undef min

#define LENGTHUNITS 1000
#define EPSILON                    1E-04
#define DEFAULT_LOOP_CYCLE         0.10

#define DEFAULT_CART_MAX_ACCEL     20000.0/LENGTHUNITS
#define DEFAULT_CART_MAX_VEL        2000.0/LENGTHUNITS
#define DEFAULT_JOINT_MAX_ACCEL    20000.0/LENGTHUNITS
#define DEFAULT_JOINT_MAX_VEL       2000.0/LENGTHUNITS

#ifndef HAVE_SINCOS
#define HAVE_SINCOS

inline void sincos(double x, double *sx, double *cx) {
    *sx = sin(x);
    *cx = cos(x);
}
#endif

#ifndef SIGN
inline double SIGN(double x) {
    return ( x >= 0.0f) ? +1.0f : -1.0f;
}
#endif
inline double NORM(double a, double b, double c, double d) {
    return sqrt(a * a + b * b + c * c + d * d);
}

namespace crcl_rosmsgs
{
typedef float float64;
typedef uint uint8;
typedef unsigned long int uint64;


struct CrclMaxProfileMsg
{
    float64   maxvel; // maximum vel for each motion
    float64   maxacc; // maximum acc for each motion
    float64   maxjerk; // maximum jerk for each motion

};

struct CrclStatusMsg
{
    std_msgs::Header header;
    uint8 crclcommandnum;
    uint8 crclstatusnum;
    uint8 crclcommandstatus;

    static const uint8 done=0;
    static const uint8 error=1;
    static const uint8 working=2;

    geometry_msgs::Pose  statuspose;
    sensor_msgs::JointState statusjoints;
    float64 eepercent;

    CrclStatusMsg()
    {
        crclcommandnum=0;
        crclstatusnum=0;
        crclcommandstatus=CrclStatusMsg::done;
        eepercent=0;
        statusjoints.position={0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        //statuspose=tf::Pose(tf::Quaternion(0.0,0.0,0.0,1.0), tf::Vector3(0.0, 0.0, 0.0));
    }
};
struct CrclCommandMsg
{
    std_msgs::Header header;
    uint8 crclcommand;

    static const uint8 initCanon=1;
    static const uint8 endCanon=2;
    static const uint8 actuatejoints=3;
    static const uint8 moveto=4;
    static const uint8 dwell=5;
    static const uint8 message=6;
    static const uint8 moveThroughTo=7;
    static const uint8 setCoordinatedMotion=8;
    static const uint8 stopMotion=9;
    static const uint8 setEndEffector=10;
    static const uint8 openToolChange=11;
    static const uint8 closeToolChanger=12;
    static const uint8 setGripperPose = 15;

    uint64 crclcommandnum;
    geometry_msgs::Pose  finalpose;
    std::vector<geometry_msgs::Pose> waypoints;
    // Below joint info could be  trajectory_msgs/JointTrajectoryPoint
    sensor_msgs::JointState joints;
    std::vector<uint64> jointnum;
    std::vector<float64> hint;
    bool bStraight;
    float64   dwell_seconds;
    std::string opmessage;
    bool bCoordinated;
    float64 eepercent;
    std::vector<CrclMaxProfileMsg> profile; // maximum profile
    std::vector<std::string> parameter_names;
    std::vector<std::string> parameter_values;
};
}

namespace RCS
{


struct CanonControlType {
    static constexpr int NO_CONTROL = 0;
    static constexpr int POSITION_CONTROL = 1;
    static constexpr int VELOCITY_CONTROL = 2;
    static constexpr int FORCE_TORQUE_CONTROL = 3;
};

struct PoseTolerance : public std::vector<double> {

    enum Type {
        POSITION = 0, ORIENTATION = 1, JOINT = 2
    };

    PoseTolerance() {
        this->resize(3, 0.0);
    } // position, orientation
};

inline sensor_msgs::JointState emptyJointState(size_t n) {
   sensor_msgs::JointState js;
    js.position.resize(n, 0.0);
    js.velocity.resize(n, 0.0);
    js.effort.resize(n, 0.0);
    return js;
}

inline sensor_msgs::JointState subset(sensor_msgs::JointState js, size_t n) {
    js.position.resize(n);
    js.velocity.resize(n);
    js.effort.resize(n);
    return js;
}
inline std::vector<double> subset(std::vector<double> position, size_t n) {
    position.resize(n);
    return position;
}
inline sensor_msgs::JointState zeroJointState(size_t numjoints) {
    sensor_msgs::JointState joints;
    for (size_t i = 0; i < numjoints; i++) {
        joints.position.push_back(0.0);
        //instead use existing rate information for each controller
#if 0
        joints.velocity.push_back(DEFAULT_JOINT_MAX_VEL);
        joints.effort.push_back(DEFAULT_JOINT_MAX_ACCEL);
#endif
    }
    return joints;
}

inline bool hasMotion(sensor_msgs::JointState js)
{
    return js.position.size()>0 || js.velocity.size()>0 || js.effort.size()>0;
}

/*!
     * \brief enumeration of  length units. Conversion into ROS compatible meters.
     */
struct CanonLengthUnit {
    static const int METER = 0;
    static const int MM = 1;
    static const int INCH = 2;
};

/*!
     * \brief enumeration of trajectory pose points.
     */
struct TrajPointType {
    static const int WAYPOINT = 1;
    static const int GOAL = 2;
};

/*!
     * \brief enumeration of  angle units. Conversion into ROS compatible radians.
     */
struct CanonAngleUnit {
    static const int RADIAN = 0;
    static const int DEGREE = 1;
};

/*!
     * \brief enumeration of  force units.
     */
struct CanonForceUnit {
    static const int NEWTON = 0;
    static const int POUND = 1;
    static const int OUNCE = 2;
};

/*!
     * \brief enumerati_nextccon of  torque units.
     */
struct CanonTorqueUnit {
    static const int NEWTONMETER = 0;
    static const int FOOTPOUND = 2;
};

/*!
     * \brief enumeration of  return type from Crcl intepretation. If statusreply, requires status
     * sent to Crcl client.
     */
struct CanonReturn {
    static const int CANON_REJECT = -2;
    static const int CANON_FAILURE = -1;
    static const int CANON_SUCCESS = 0;
    static const int CANON_STATUSREPLY = 1;
    static const int CANON_MOTION = 2;
    static const int CANON_RUNNING = 3;
};

/*!
     * \brief enumeration of   Crcl commands. Many Crcl commands are wm parameter setting
     * and require no motion component.
     */
struct CanonCmdType {
    /**
         *  uint8 initCanon=1
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
         */
    static const int CANON_UNKNOWN = -1;
    static const int CANON_NOOP = 0;
    static const int CANON_INIT_CANON = 1;
    static const int CANON_END_CANON = 2;
    static const int CANON_MOVE_JOINT = 3;
    static const int CANON_MOVE_TO = 4;
    static const int CANON_DWELL = 5;
    static const int CANON_MESSAGE = 6;
    static const int CANON_MOVE_THRU = 7;
    static const int CANON_SET_COORDINATED_MOTION = 8;
    static const int CANON_STOP_MOTION = 9;
    static const int CANON_SET_GRIPPER = 10;
    static const int CANON_OPEN_GRIPPER = 11;
    static const int CANON_CLOSE_GRIPPER = 12;
    static const int CANON_SET_TOLERANCE = 13;
    static const int CANON_SET_MAX_JOINT_SPEED=16;
    static const int  CANON_SET_EE_PARAMETERS=25;
#if 0
    // Extensions:C
    static const int CANON_CONTACT_GRIPPER = 14;
    static const int CANON_PAVEL_GRIPPER = 15;

    static const int CANON_SET_GRIPPER_POSE = 17;
    static const int CANON_PICK = 18;
    static const int CANON_PLACE = 19;
    static const int CANON_SET_BASE_POSE = 20;
    static const int CANON_FEEDHOLD = 21;
    static const int  CANON_GRASP_OBJECT=22;
    static const int  CANON_RELEASE_OBJECT=23;
    static const int  CANON_MOVE_ROBOT=24;
    static const int  CANON_GET_STATUS=26;

    static const int CANON_SET_MAX_CART_ACC = 0;
    static const int CANON_SET_MAX_CART_SPEED = 0;
    static const int CANON_SET_MAX_JOINT_ACC = 0;
    static const int CANON_SET_MAX_JOINT_SPEED = 0;

#endif
};
////////////////////////////////////////////////////////////////////////////////
/**
 * @brief sCmd string corresponding to CRCL command enum
 * @param i the CRCL command id
 * @return string of enumeration
 */
inline const char * sCmd(int i)
{
    switch(i)
    {
    case CanonCmdType::CANON_NOOP : return  "CANON_NOOP";
    case CanonCmdType::CANON_INIT_CANON : return  "CANON_INIT_CANON";
    case CanonCmdType::CANON_END_CANON : return  "CANON_END_CANON";
    case CanonCmdType::CANON_MOVE_JOINT : return  "CANON_MOVE_JOINT";
    case CanonCmdType::CANON_MOVE_TO : return  "CANON_MOVE_TO";
    case CanonCmdType::CANON_DWELL: return  "CANON_DWELL";
    case CanonCmdType::CANON_MESSAGE : return  "CANON_MESSAGE";
    case CanonCmdType::CANON_MOVE_THRU : return  "CANON_MOVE_THRU";
    case CanonCmdType::CANON_SET_COORDINATED_MOTION : return  "CANON_SET_COORDINATED_MOTION";
    case CanonCmdType::CANON_STOP_MOTION : return  "CANON_STOP_MOTION";
    case CanonCmdType::CANON_SET_GRIPPER : return  "CANON_SET_GRIPPER";
    case CanonCmdType::CANON_OPEN_GRIPPER : return  "CANON_OPEN_GRIPPER";
    case CanonCmdType::CANON_CLOSE_GRIPPER : return  "CANON_CONTACT_GRIPPER";
    case CanonCmdType::CANON_SET_TOLERANCE : return  "CANON_CONTACT_GRIPPER";
    default:
       return  "CANON UNKNOWN";
    }

    return "CANON UNKNOWN";
}

/*!
  * \brief enumeration of  stopping motion, e.g., estop equivalent to immediate.
  */
 struct CanonStopMotionType {
     static const int UNSET = -1;
     static const int IMMEDIATE = 0;
     static const int FAST = 1;
     static const int NORMAL = 2;
 };

/*!
     * \brief enumeration of  trajectory acceleration profile.
     */
struct CanonAccProfile {
    static const int MS_IS_UNSET = 0;
    static const int MS_IS_DONE = 1;
    static const int MS_IS_ACCEL = 2;
    static const int MS_IS_CONST = 3;
    static const int MS_IS_DECEL = 4;
    static const int MS_IS_ESTOPPING = 5;
    static const int MS_IS_PAUSED = 6;
};

/*!
     * \brief enumeration of  trajectory motion type, joint or cartesian.
     */
struct MovementType {
    static const int MOVE_DEFAULT = 0;
    static const int MOVE_CARTESIAN = 1;
    static const int MOVE_JOINT = 2;
};

/*!
     * \brief enumeration of controller status types for individual commands.
     * Note, even though command types are listed, not all used or supported.
     */
struct CanonStatusType {
    /**
        uint8 done=0
        uint8 error=1
        uint8 working=2
         */
    static const int CANON_DONE = 0;
    static const int CANON_ERROR=1;
    static const int CANON_WORKING=2;
    static const int CANON_READY=3;
    static const int CANON_STOP=4;
    static std::string value(int n)
    {
        switch(n)
        {
        case CANON_DONE:
            return "CANON_DONE";
        case CANON_ERROR:
            return "CANON_ERROR";
        case CANON_WORKING:
            return "CANON_WORKING";
        case CANON_READY:
            return "CANON_READY";
        case CANON_STOP:
            return "CANON_STOP";

        }
        return "CANON_ERROR";
    }
};

/**
     * \brief IRate is an interface class for defining the allowed motion rates.
     */
class IRate {
public:

    IRate() {
        clear();
    }

    // Average velocity
    NVAR(FinalVelocity, double, _final_velocity);
    NVAR(CurrentVelocity, double, _current_velocity);

    // Feedrate equivalent
    VAR(double, CurrentTransSpeed);
    VAR(double, CurrentTransAcceleration);
    VAR(double, CurrentRotSpeed);
    VAR(double, CurrentRotAcceleration);

    // HARD LIMITS
    NVAR(MaximumVelocity, double, _maximum_velocity);
    NVAR(MaximumAccel, double, _maximum_accel);

    VAR(double, MaxJointVelocity);
    VAR(double, MaxJointAccel);
    VAR(std::vector<double>, JointMaxLimit );
    VAR(std::vector<double>, JointMinLimit);
    VAR(std::vector<double>, JointVelLimit);
    VAR(std::vector<double>, JointAccLimit);

    VAR(double, CurrentAccel);
private:

    void clear() {
        FinalVelocity() = 0.0;
        CurrentVelocity() =  DEFAULT_CART_MAX_VEL;
        CurrentAccel() = CurrentVelocity() * 10.;
        MaximumVelocity() =  DEFAULT_CART_MAX_VEL;
        MaximumAccel() = DEFAULT_CART_MAX_ACCEL;
        MaxJointVelocity() = DEFAULT_JOINT_MAX_VEL;
    }
};

/*!
     * \brief CanonCmd is the controller command structure.
     *
     * int64 crclcommand
           crclcommandnum
        # https://github.com/ros/common_msgs
        geometry_msgs/Pose  finalpose
        geometry_msgs/Pose[] waypoints
        # Below joint info could be  trajectory_msgs/JointTrajectoryPoint
        sensor_msgs/JointState joints
        bool bStraight
        float64   dwell_seconds
        string opmessage
        bool bCoordinated
        float64 eepercent
        CrclMaxProfileMsg[] profile # maximum profile
     */
struct CCanonCmd : public crcl_rosmsgs::CrclCommandMsg {

    /*!
         * \brief CanonCmd constructor.
         */
    CCanonCmd() {
        //           Init();
        CommandID() = cmdid()++;
        this->eepercent=-1.0;
    }

    CCanonCmd& Set(crcl_rosmsgs::CrclCommandMsg &msg) {
        this->header = msg.header;
        this->crclcommand = msg.crclcommand;
        this->crclcommandnum = msg.crclcommandnum;
        this->finalpose = msg.finalpose;
        this->waypoints = msg.waypoints;

        this->joints = msg.joints;
        this->jointnum = msg.jointnum;
        this->bStraight = msg.bStraight;

        this->dwell_seconds = msg.dwell_seconds;
        this->opmessage = msg.opmessage;
        this->bCoordinated = msg.bCoordinated;

        this->eepercent = msg.eepercent;
        this->parameterNames=msg.parameter_names;
        this->parameterValues=msg.parameter_values;
        this->profile=msg.profile;

        // Doesn't seem to work copying
        //    static_cast<nistcrcl::CrclCommandMsg>(*this)  = msg;
        return *this;

    }

    uint64_t & commandNum() { return this->crclcommandnum;}
    static unsigned long long & cmdid()
    {
        static unsigned long long _cmdid;
        return _cmdid;
    }

    static void setRosMsgTimestamp(::std_msgs::Header &header )
    {
        timespec start;
        clock_gettime(CLOCK_REALTIME, &start);
        header.stamp.sec  = start.tv_sec;
        header.stamp.nsec = start.tv_nsec;
    }
    static std::string strTimeval(struct timeval & tv)
    {
        time_t nowtime;
        struct tm *nowtm;
        char tmbuf[64], buf[64];
        nowtime = tv.tv_sec;
        nowtm = localtime(&nowtime);
        strftime(tmbuf, sizeof tmbuf, "%Y-%m-%d %H:%M:%S", nowtm);
        snprintf(buf, sizeof buf, "%s.%06ld", tmbuf, tv.tv_usec);
        return buf;
    }
    bool isMotionCmd()
    {
        static int motions[] = {CanonCmdType::CANON_MOVE_JOINT,
                                CanonCmdType::CANON_MOVE_TO,
                                CanonCmdType::CANON_MOVE_THRU,
                                CanonCmdType::CANON_SET_GRIPPER,
                                CanonCmdType::CANON_STOP_MOTION};
        int * it = std::find(motions, motions +sizeof(motions)/sizeof(int), crclcommand);
        if (it != motions + 5)
            return true;
        return false;
    }

    ////////////////////////////////////////////////////////////////////////////
    VAR(unsigned long long, CommandID );
    VAR(unsigned long long, CrclCommandID);
    VAR(unsigned long long, StatusID);
    VAR(IRate, Rates);
    VAR(std::vector<double>, ConfigMin);
    VAR(std::vector<double>, ConfigMax);

    int robotControlAlgorithm;  // each joint has different control algorithm?
    int gripperControlAlgorithm;  // each joint has different control algorithm?

    int status; /**<  status type */
    int type; /**<  trajectory points  type */
    int stoptype; /**<  stop trajectory choice */
    int accprofile; /**<  current trajectory acceleration profile */


    double absTransAcc; /**<  cartesian translational acceleration */
    double absTransSpeed; /**<  cartesian translational velocity */
    double absRotAcc; /**<  cartesian rotation acceleration */
    double absRotSpeed; /**<  cartesian rotation velocity */
    double absJointAcc; /**<  joint max acceleration */
    double absJointSpeed; /**<  joint max velocity */

    std::vector<std::string> parameterNames;
    std::vector<std::string> parameterValues;

    std::vector<double> speed; /**<  vector of joint velocities */
    RCS::PoseTolerance endPoseTol; /**<  commanded tolerance */
    // std::vector<RCS::Pose> waypointtolerances; /**< commanded cartesian waypoints in trajectory */
    RCS::PoseTolerance intermediatePtTol; /**< commanded cartesian waypoints in trajectory */

    RCS::PoseTolerance gripperTol; /**< gripper trajectory */
    std::vector<sensor_msgs::JointState> gripperMoves;
    sensor_msgs::JointState nextGripperGoalJoints;
    std::vector<int> gripperControlType;

    inline     std::string toString()
    {
        std::stringstream ss;
        ss << "======================================================\n";
        boost::posix_time::ptime pt = header.stamp.toBoost();
//        boost::gregorian::date d = pt.date();
//        boost::posix_time::time_duration td = pt.time_of_day();
        ss << "Timestamp =       " << pt.date() << " " << pt.time_of_day() << "\n";
        ss << "Command =         " << RCS::sCmd(this->crclcommand) << "\n";
        ss << "CommandNum =      " << crclcommandnum << "\n";
        ss << "Final Pose =      " << RCS::dumpPoseSimple(Convert<geometry_msgs::Pose, tf::Pose>(this->finalpose)) << "\n";
        ss << "Joints =          " << RCS::vectorDump<double>(this->joints.position) << "\n";
        ss << "JointNum =        " << RCS::vectorDump<uint64_t>(this->jointnum) << "\n";
        ss << "bStraight =       " << ToStr<bool>(bStraight) << "\n";
        ss << "dwell_seconds =   " << dwell_seconds << "\n";
        ss << "bCoordinated =    " << ToStr<bool>(bCoordinated) << "\n";
        ss << "eepercent =       " << eepercent << "\n";
        ss << "parameter_names = " << RCS::vectorDump<std::string>(this->parameterNames, ",") << "\n";
        ss << "parameter_values =" << RCS::vectorDump<std::string>(this->parameterValues, ",") << "\n";
        return ss.str();
    }
};

struct CController;
/*!
     * \brief CanonWorldModel describes the controller state. Includes reference to robot model.
     */
struct CCanonWorldModel {

    /*!
         * \brief CanonWorldModel constructor that initializes parameterization.
         */
    CCanonWorldModel() {
    }
    //@todo add some specific parameters to initialization
    void init()
    {
        _cycleTime = DEFAULT_LOOP_CYCLE;

        _maxTransAccel = DEFAULT_CART_MAX_ACCEL;
        _maxTransVel = DEFAULT_CART_MAX_VEL;
        _maxRotAccel = DEFAULT_CART_MAX_ACCEL;
        _maxRotVel = DEFAULT_CART_MAX_VEL;
        _maxJointAccel = DEFAULT_JOINT_MAX_ACCEL;
        _maxJointVel = DEFAULT_JOINT_MAX_VEL;
        _bStraight=true;
        _bCoordinated=true;
    }

    CCanonCmd echocmd; /**<  copy of current command */
    int echoCmdId; /**<  copy of current command type */
    int crclCommandStatus;
    int echoStatus; /**<  copy of current status type */

    /*!
         * \brief Cycletime of the world model.
         * /fixme what is this
         */
    double getCycleTime() {
        return _cycleTime; // milliseconds
    }

    // //////////////////////
    sensor_msgs::JointState robotjoints; /**<  current joint state */
    sensor_msgs::JointState gripperjoints; /**<  current joint state */
    tf::Pose currentpose; /**<  current robot pose */
    int robotControlAlgorithm;  /**< control algorithm for all robot joints */
    int gripperControlAlgorithm;  /**< control algorithm for all gripper jointsm?*/

    double _maxTransAccel; /**<  max translation acceleration */
    double _maxTransVel; /**<  max translation velocity */
    double _maxRotAccel; /**<  max rotational acceleration */
    double _maxRotVel; /**<  max rotational velocity */
    double _maxJointAccel; /**<  max joint acceleration */
    double _maxJointVel; /**<  max joint velocity */
    double _cycleTime; /**<  cycle time */
    bool _bStraight;
    bool _bCoordinated;
    double _eepercent;


};

/*!
     * \brief IInterpreter parses a RCS command and generates robot motion commands.
     */
class IRCSInterpreter {
public:

     IRCSInterpreter() :
         s2eFingerContactAlgorithm{{"none", none}, {"velocity", velocity}, {"force", force},{"squeeze", squeeze}}
     {

     }
    virtual int parseCommand(crcl_rosmsgs::CrclCommandMsg &incmd,
                             RCS::CCanonCmd &outcmd,
                             RCS::CCanonWorldModel instatus)
    {
        return -1;
    }

    virtual void init(std::vector<double> jnts)
    {
    }
    std::string _name;
    typedef  enum {none=0, velocity=1, force=2,  squeeze=3} eFingerContactAlgorithm;
    std::map<std::string, eFingerContactAlgorithm> s2eFingerContactAlgorithm;
    eFingerContactAlgorithm fingerContactAlgorithm; /// squeeze=3, velocity=1, force=2
};


struct compareCrclId {
    bool operator()(crcl_rosmsgs::CrclCommandMsg const & p1, crcl_rosmsgs::CrclCommandMsg const & p2) {
        // return "false" if "p1" is ordered before "p2"
        return p1.crclcommandnum < p2.crclcommandnum;
    }
};

//typedef RCS::CMessageQueue<crcl_rosmsgs::CrclCommandMsg > CrclMessageQueue;
typedef RCS::CPriorityMessageQueue<crcl_rosmsgs::CrclCommandMsg, RCS::compareCrclId>  CrclMessageQueue;

}



#endif
