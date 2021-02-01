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

#ifndef CRCLIMPL_H
#define CRCLIMPL_H
// CrclRobotImpl.h

#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <boost/shared_ptr.hpp>

#include "crclapp/Globals.h"
#include "crclapp/gripper.h"

typedef  boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> MoveGroupInterfacePtr;

namespace crcl
{
class CRobotImpl
{
public:
    CRobotImpl();
    ~CRobotImpl();

    CRobotImpl & init();

    CRobotImpl & nodeHandle(ros::NodeHandle *nh)
    {
        this->nh=nh;
        return *this;
    }
    CRobotImpl & urdfRobotDescriptionRosParam(std::string urdf_robot_description_param)
    {
        this->urdf_robot_description_param=urdf_robot_description_param;
        return *this;
    }
    CRobotImpl & moveGroupName(std::string move_group_name)
    {
        this->PLANNING_GROUP=move_group_name;
        return *this;
    }
    CRobotImpl & gzGripperTopic(std::string gripper_topic)
    {
        this->gz_gripper_topic=gripper_topic;
        return *this;
    }

    CRobotImpl & gzRobotModelName(std::string sRobotName)
    {
        this->_gzrobot_name=sRobotName;
        return *this;
    }
    CRobotImpl & gzEnabled(bool bFlag)
    {
        this->gz_enabled=bFlag;
        return *this;
    }
    CRobotImpl & jointStatePublisherTopic(std::string joint_state_publisher_topic)
    {
        this->joint_state_publisher_topic=joint_state_publisher_topic;
        return *this;
    }
    bool isConfigurationValid();
    CRobotImpl & assertConfigurationValid();


    //////////////////////////////////////////////////////////
    // Supported CRCL commands (not parameter modifications)
    int doDwell(double seconds ) ;
    int setGripper(double p);
    int initCrcl(){ return 0; }
    int endCrcl(){ return 0; }
    moveit_msgs::RobotTrajectory moveTo(tf::Pose final,
                                      double velscale=0.1);
    moveit_msgs::RobotTrajectory moveJoints(std::vector<double> positions,
                                            std::vector<std::string> jointnames=std::vector<std::string>(),
                                            double vel=0.1);

    // Trajectory support methods
    geometry_msgs::Pose currentPose();
    std::vector<double> currentJoints();
    std::string currentState();
    void publishFakeJoints(trajectory_msgs::JointTrajectoryPoint);
    tf::Pose FK(std::vector<double> positions);
    std::vector<double>  IK(tf::Pose pose );
    void move(moveit_msgs::RobotTrajectory);
    moveit_msgs::RobotTrajectory eeMoveTo(tf::Pose eeFinal,
                                                  double velscale=0.1);
    void publishJoints(std::vector<double> positions);
private:
    robot_model_loader::RobotModelLoader _robotModelLoader;
    robot_model::RobotModelPtr _robot_model;
    robot_state::RobotStatePtr _robot_state;
    MoveGroupInterfacePtr _move_group;//(PLANNING_GROUP);
    const moveit::core::JointModelGroup* _joint_model_group;
    ros::NodeHandle *nh; /**< ros pointer to app node handle */
    ros::ServiceClient _gzSetJointTraj; /**< gazebo ros api plugin service */
    ros::ServiceClient _gzGetJointTraj; /**< gazebo ros api plugin service */
    ros::Publisher _robotJointState; /**< ros subscriber information used for fake joint states */

    gzGripper gripper; /**< gripper wrapper */

    std::string PLANNING_GROUP; /**< name of move group of joints (defined in sdrf) */
    std::string urdf_robot_description_param;
    std::string _robot_name; /**< name of robot defined in urdf */
    std::string _gzrobot_name; /**< gazebo name of robot model defined in sdf */
    std::string gz_gripper_topic; /**< topic name for gazebo gripper service */
    std::string joint_state_publisher_topic; /**< renamed topic name for joint state publisher INPUT */
    bool bConfigException;

    std::vector<std::string> joint_names;
    bool gz_enabled;

};
}
#endif // CRCLIMPL_H
