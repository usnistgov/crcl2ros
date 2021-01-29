#ifndef CRCLWM_H
#define CRCLWM_H
///  CrclWm.h
#include <map>
#include <vector>

#include "rcs/RCSThreadTemplate.h"
#include "rcs/RCSMsgQueueThread.h"
#include "rcs/IRcs.h"

#include "crclapp/Shape.h"


class CRosCrclClient;  /// ROS topic communication handler
extern std::shared_ptr<CRosCrclClient> rosCrclClient;  /// ROS topic handling


namespace crcl{
//Forward declaration
class CSocketSession;  /// handle socket crcl messaging
class CCrcl2RosMsgApi;
class CRosCmdStatusHandler;    /// accept queued messages from crcl->ros
class CRosCmdMsgHandler; /// accept queued messages ros commands
class CRobotImpl;

extern std::mutex mutex;
extern std::string crclIp;
extern int crclPort;
extern int dbgCrclCommand;
extern int dbgCrclStatus;
extern int dbgCrclStatusMsg;
extern int dbgCrclCommandMsg;
extern int bDebug;
extern int bCrclStopIgnore;
extern int bFlywheel;  /// processing one queued crcl message until done
extern int bProcessAllCrclMessages; ///  process all crcl message until done
extern int bAutoCrclStatus;
extern int bSynchronousCmds;

extern void configure(ros::NodeHandle *nh, std::string ns);
extern void printConfiguration();

extern std::shared_ptr<crcl::CSocketSession> crclCommServer; /// crcl socket server
extern std::shared_ptr<crcl::CRosCmdStatusHandler> rosCmdStatMsgHandler;
extern std::shared_ptr<crcl::CRosCmdMsgHandler> rosCmdMsgHandler;
extern std::shared_ptr<crcl::CRobotImpl> crclRobotImpl;
extern std::shared_ptr<crcl::CCrcl2RosMsgApi>  crclApi;       /// outer crcl api to generate ROS command issuer


}
namespace RCS
{
/**
 * @brief cmds is the queue for CRCL XML commands that have been translated
 * from CRCL into ROS equivalent custom command message.
 */

extern  CMessageQueue<RCS::CCanonCmd> cmds;

/**
 * @brief wm contains the ROS representation of the curfrent robot world
 * model.
 */
extern  CCanonWorldModel wm;  // motion related parameters max vel,acc

};



struct rcs_state
{
    static const int NORMAL = 0;
    static const int EXITING = 1;
    static const int PAUSED = 2;
    static const int ERROR = 3;
    static const int ONCE = 4;
    static const int NOOP = 5;
    static const int AUTO = 6;
    static const int REVERSE=7;
    static const int REPEAT=7;
    static const int STEP=8;
    static const int TEST=9;
};

struct rcs_status_type
{
    // FIXME: this needs to be merged into other status
    sensor_msgs::JointState r_curjnts;
    tf::Pose r_curpose;
    double currentRobotJointSpeed();
    double currentGripperJointSpeed();
    double currentLinearSpeed();
    double currentAngularSpeed();

};
struct rcs_robot_type
{
    tf::Pose current_pose;
    double current_eesetting;
    sensor_msgs::JointState current_joints;
    static int _crclcommandnum; /**<  crcl command number for the robot -i.e., this one */

    /**
     * @brief numJoints robot's number of joints (assume serial)
     * @return number of joints
     */
    size_t numJoints();

    /**
     * @brief isBusy read the current status to determine if
     * the robot is still busy executing a command
     * @return true if busy
     */
    bool isBusy(int & cmdnum);

    /**
     * @brief allJointNumbers for CRCL generate vector of numbers
     * for all the joints
     * @return vector of integers, 0..n where n is the number of joints
     * in the robot
     */
    std::vector<unsigned long> allJointNumbers();
    /**
     * @brief configure read ini config file for parameters
     * @param robot prefix name of robot for ini file
     * @param inifile full path of inifile
     */
    void configure(std::string robot, ros::NodeHandle *nh, std::string ns);

    void printConfiguration();

    /**
     * @brief ParseURDF accepts an xml urdf string and parses out the joint information.
     * Really just for joint names since CRCL uses numbers.
     * @return true if successful
     */
    bool parseURDF();

    std::string robotTiplink, robotBaselink,robot_urdf;

    // URDF Derived knowledge
    std::vector<std::string> jointNames;
    std::vector<std::string> linkNames;
    std::vector< double> jointvalues;
    std::vector< double> jointMin;
    std::vector< double> jointMax;
    std::vector< bool> jointHasLimits;
    std::vector< double> jointEffort;
    std::vector< double> jointVelmax;
    std::vector<tf::Vector3> axis;
    std::vector<tf::Vector3> xyzorigin;
    std::vector<tf::Vector3> rpyorigin;
    std::string  robotName;

    // Saved named robot goal states
    std::map<std::string, std::vector<std::string>> namedCommand;
    std::map<std::string, std::vector<double>> namedJointMove;
    std::map<std::string, tf::Pose> namedPoseMove;

    tf::Pose removeGripper(tf::Pose pose )
    {
        if(Globals.dbgTransforms)
            std::cout<< "With gripper" << RCS::dumpPoseSimple(pose) << std::endl;
        pose=  GripperInv * pose;
        if(Globals.dbgTransforms)
            std::cout<< "Wout gripper" << RCS::dumpPoseSimple(pose) << std::endl;
        return pose;
    }
    tf::Pose addGripper(tf::Pose pose )
    {
        if(Globals.dbgTransforms)
            std::cout<< "With gripper" << RCS::dumpPoseSimple(pose) << std::endl;
        pose= pose * Gripper;
        if(Globals.dbgTransforms)
            std::cout<< "Wout gripper" << RCS::dumpPoseSimple(pose) << std::endl;
        return pose;
    }

    tf::Pose addOffset(tf::Pose pose, tf::Vector3 offset )
    {
        if(Globals.dbgTransforms)
            std::cout<< "Before addOffset" << RCS::dumpPoseSimple(pose) << ":" << RCS::dumpVector(offset)<< std::endl;
        pose= tf::Pose(pose.getRotation(), pose.getOrigin() + offset);
        if(Globals.dbgTransforms)
            std::cout<< "After addOffset" << RCS::dumpPoseSimple(pose) << ":" << RCS::dumpVector(offset)<< std::endl;
        return pose;
    }
    tf::Pose removeOffset(tf::Pose pose, tf::Vector3 offset  )
    {
        return tf::Pose(pose.getRotation(), pose.getOrigin() - offset);
    }
    tf::Pose removeRetract(tf::Pose pose )
    {
        if(Globals.dbgTransforms)
            std::cout<< "With retract" << RCS::dumpPoseSimple(pose) << std::endl;
        pose= tf::Pose(pose.getRotation(), pose.getOrigin() - Retract.getOrigin());
        if(Globals.dbgTransforms)
            std::cout<< "Wout retract" << RCS::dumpPoseSimple(pose) << std::endl;
        return pose;
    }
    tf::Pose addRetract(tf::Pose pose )
    {
        if(Globals.dbgTransforms)
            std::cout<< "Before Retract" << RCS::dumpPoseSimple(pose) << std::endl;
        pose= tf::Pose(pose.getRotation(), pose.getOrigin() + Retract.getOrigin());
        if(Globals.dbgTransforms)
            std::cout<< "After Retract" << RCS::dumpPoseSimple(pose) << std::endl;
        return pose;
    }
    tf::Pose removeBase(tf::Pose pose )
    {
        if(Globals.dbgTransforms)
            std::cout<< "With base" << RCS::dumpPoseSimple(pose) << std::endl;
        pose= basePoseInverse * pose ;
        if(Globals.dbgTransforms)
            std::cout<< "Wout base" << RCS::dumpPoseSimple(pose) << std::endl;
        return pose;
    }
    tf::Pose addBase(tf::Pose pose )
    {
        if(Globals.dbgTransforms)
            std::cout<< "Wout base" << RCS::dumpPoseSimple(pose) << std::endl;
        pose= basePose * pose ;
        if(Globals.dbgTransforms)
            std::cout<< "With base" << RCS::dumpPoseSimple(pose) << std::endl;
        return pose;
    }

    // FIXME: needs to be updated...
    crcl_rosmsgs::CrclStatusMsg  _status;

    // Canned poses
    tf::Pose Retract;         /**< pose offset for retract */
    tf::Pose RetractInv; /**< inverse pose offset for retract */
    tf::Quaternion QBend; /**< rotation to achieve pose rotation for grasping */
    tf::Pose currentPose;
    tf::Pose basePose;
    tf::Pose basePoseInverse;
    tf::Pose Gripper;
    tf::Pose GripperInv;

    std::map<std::string, tf::Pose> gripperoffset;       /// gripper offset for each part

    //std::string _prefix;  /**< robot name prefix */
    unsigned int crclcommandstatus;
    std::string s_crclcommandstatus;
    unsigned int crclcommandid;

};

struct rcs_world_type
{
    std::string robot;

    std::map<std::string, tf::Pose> _gearStartingPoses;
    std::map<std::string, tf::Pose>::iterator gearit;
    std::vector<std::string> part_list; /// robot parts from set of all parts
    std::map<std::string, tf::Pose> slotoffset;          /// gripper offset for container slot
    void configure(ros::NodeHandle *nh, std::string ns);
    void printConfiguration();

};

extern rcs_robot_type rcs_robot;
extern rcs_world_type rcs_world;
extern rcs_status_type rcs_status;
namespace WorldModel
{
extern CInstances instances;
}

/**
 * @brief instances list of all instances in world.
 */
#endif // CRCLWM_H
