#ifndef CRCLIMPL_H
#define CRCLIMPL_H

#include <ros/ros.h>
#include <tf/tf.h>

#if 0
// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <boost/shared_ptr.hpp>

#include "move_it/globals.h"
#endif
//#include "crclapp/gripper.h"
#include <moveit_msgs/RobotTrajectory.h>

//typedef  boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> MoveGroupInterfacePtr;
class crclimpl
{
public:
    crclimpl();
    ~crclimpl();

    crclimpl & init();

    crclimpl & nodeHandle(ros::NodeHandle *nh)
    {
        this->nh=nh;
        return *this;
    }
//    crclimpl & robotName(std::string robot_name)
//    {
//        this->robot_name=robot_name;
//        return *this;
//    }
    crclimpl & urdfRobotDescriptionRosParam(std::string urdf_robot_description_param)
    {
        this->urdf_robot_description_param=urdf_robot_description_param;
        return *this;
    }
    crclimpl & gzGripperTopic(std::string gripper_topic)
    {
        this->gz_gripper_topic=gripper_topic;
        return *this;
    }

    crclimpl & gzRobotModelName(std::string sRobotName)
    {
        this->_gzrobot_name=sRobotName;
        return *this;
    }
    crclimpl & gzEnabled(bool bFlag)
    {
        this->gz_enabled=bFlag;
        return *this;
    }

    bool isConfigurationValid();
    crclimpl & assertConfigurationValid();
    //////////////////////////////////////////////////////////
    // Supported CRCL commands (not parameter modifications)
    int doDwell(double seconds ) ;
    int setGripper(double p);
    int initCrcl(){ return 0; }
    int endCrcl(){ return 0; }

    moveit_msgs::RobotTrajectory eeMoveTo(geometry_msgs::Pose eeFinal,
                                                  double velscale=0.1);\

    moveit_msgs::RobotTrajectory eeMoveTo(tf::Pose eeFinal,
                                                  double velscale=0.1);\
    moveit_msgs::RobotTrajectory moveJoints(std::vector<double> positions,
                                            std::vector<std::string> jointnames=std::vector<std::string>(),
                                            double vel=0.1);

    tf::Pose FK(std::vector<double> positions);
    // Trajectory support methods
    geometry_msgs::Pose currentPose();
    std::vector<double> currentJoints();
     std::string currentState();
    void publishFakeJoints(trajectory_msgs::JointTrajectoryPoint);
    void publishJoints(std::vector<double> positions);


    ros::NodeHandle *nh; /**< ros pointer to app node handle */
    ros::ServiceClient _gzSetJointTraj; /**< gazebo ros api plugin service */
    ros::ServiceClient _gzGetJointTraj; /**< gazebo ros api plugin service */
    ros::Publisher _robotJointState; /**< ros subscriber information used for fake joint states */

    //gzGripper gripper; /**< gripper wrapper */

    std::string PLANNING_GROUP; /**< name of move group of joints (defined in sdrf) */
    std::string urdf_robot_description_param;
    std::string _robot_name; /**< name of robot defined in urdf */
    std::string _gzrobot_name; /**< gazebo name of robot model defined in sdf */
    std::string gz_gripper_topic; /**< topic name for gazebo gripper service */
    std::string joint_state_publisher_topic; /**< renamed topic name for joint state publisher INPUT */
    bool bConfigException;
    bool gz_enabled;

    std::vector<std::string> joint_names;
};
#endif // CRCLIMPL_H
