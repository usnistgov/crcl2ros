#include <gazebo_msgs/SetJointTrajectory.h>
#include <gazebo_msgs/SetModelConfiguration.h>
#include <gazebo_msgs/GetModelState.h>
#include <sensor_msgs/JointState.h>
#include <chrono>
#ifdef MOVEIT
#include "move_it/crclimpl.h"
#include "move_it/globals.h"
#include "move_it/gripper.h"
#include "move_it/wm.h"
#endif
#include "crclapp/crclimpl.h"
#include "rcs/Conversions.h"
#include "rcs/Debug.h"
#include "crclapp/gripper.h"
#include "crclapp/CrclWm.h"


// http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/move_group_interface/move_group_interface_tutorial.html#cartesian-paths
crclimpl::crclimpl()
{
    this->nh=nullptr;
    this->PLANNING_GROUP.clear();
    this->urdf_robot_description_param.clear();
    this->_robot_name.clear();
    this->gz_gripper_topic.clear();
    bConfigException=false; // nullify configuration
    this->gz_enabled=true;

}
crclimpl::~crclimpl()
{
    _gzSetJointTraj.shutdown();
    _gzGetJointTraj.shutdown();
    _robotJointState.shutdown();
}
bool crclimpl::isConfigurationValid()
{
    if(this->nh== nullptr ||
            this->PLANNING_GROUP.empty() ||
            this->urdf_robot_description_param.empty() ||
            this->_robot_name.empty()||
            this->joint_state_publisher_topic.empty() ||
            this->_gzrobot_name.empty() )
//            _joint_model_group==nullptr||
//            _move_group.get() == nullptr ||
//            _robot_model.get() == nullptr)
        return false;
    return true;
}
crclimpl & crclimpl::assertConfigurationValid()
{
    assert(isConfigurationValid());
    return *this;
}

/**
 * @brief config configuration of the move_it crcl trajectoy planner. The calling
 * arguments are the ros parameters that are needed to determine move_group "arm"
 * that is being controlled.
 *
 * NOTE the documentation appears to alow multiple URDF ros param definitionw to be
 * handled by moveit. THIS APPEARS TO BE TRICKY. So instead all the robot and gripper
 * combinations are merged as xacor defintions into one URDF file, where the base0joint
 * of each robot is offset fromt he worl coordinates.
 *
 * @param nh - ros node handle pointer
 * @param urdf_robot_description_param the ros param that contains the URDF description
 * of the robots in the Gazebo simulation (spawned by gazebo_ros_api plugin).
 * @param robot_name  - name of the robot defined in URDF.
 * @param PLANNING_GROUP
 */
crclimpl & crclimpl::init()
{
    try{

        // SKip all this as we are only echoing motion commands to console
    }
    catch(ros::InvalidNameException e)
    {
        std::cerr << "Invalid node handle topic name" << e.what();
        bConfigException=true; // nullify configuration
    }
    catch(...)
    {
        std::cerr << "Invalid crclimpl initialization\n";
        bConfigException=true; // nullify configuration
    }
    return *this;
}

int crclimpl::setGripper(double d)
{
    rcs_robot.current_eesetting=d;
    std::cout << "crclimpl::setGripper = " <<  d <<"\n";
    doDwell(1.0);
    return 0;
}

int crclimpl::doDwell(double seconds)
{
    //FIXME: if seconds < 0.0001 is zero sleep should be sleep or yield
    std::cout << "crclimpl::doDwell = " <<  seconds <<" seconds\n";
    std::this_thread::sleep_for(std::chrono::milliseconds((int64_t) (seconds*1000.)));
    return 0;
}

void crclimpl::publishFakeJoints(trajectory_msgs::JointTrajectoryPoint pt)
{
    std::vector<double> jts = pt.positions;

    // If no one listening don't publish
    if( _robotJointState.getNumSubscribers()==0)
    {
        std::cerr << "crclimpl::publishFakeJoints NO ONE LISTENING?!?!\n";
        std::cerr << "Check that joint_state_publisher and _robotJointState topic names match EXACTLY\n";
    }

    sensor_msgs::JointState jntCmdMsg;
    const std::vector<std::string>  joint_names;// FIXME = _joint_model_group->getVariableNames();
    jntCmdMsg.name.insert(jntCmdMsg.name.begin(), joint_names.begin(), joint_names.end());
    assert(joint_names.size() == jts.size());
    jntCmdMsg.position=jts;
    jntCmdMsg.header.stamp = ros::Time::now();
    _robotJointState.publish(jntCmdMsg);
}
/**
 * @brief FK forward kinematic solution return as tf Pose
 * There should be no problem with singularities, as expectation
 * is for industrial robot , not stweart platform
 * @param positions joint positions of the move_group
 * @return tf:pose of last link in robot coordinate system, w
 * which can be either robot or world coordiante system
 * depending on the URDF definition of the robot.
 */
tf::Pose crclimpl::FK(std::vector<double> positions)
{
    // FIXME: determine what joints names are actually moved
    sensor_msgs::JointState jnts;
    geometry_msgs::Pose p ;
#ifdef MOVEIT

    try
    {
        const moveit::core::RobotModelPtr& kinematic_model = _robotModelLoader.getModel();
        //std::cout << Globals.format("Model frame: %s", kinematic_model->getModelFrame().c_str());
        moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));

        const std::vector<std::string> & joint_names = _joint_model_group->getVariableNames();
        jnts.position=positions;
        jnts.name.insert(jnts.name.begin(), joint_names.begin(), joint_names.end());
        kinematic_state->setVariableValues(jnts);

        const std::vector< std::string > &links = _move_group->getLinkNames ();
        const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform(links.back());
        ::poseEigenToMsgImpl(end_effector_state,p);
    }
    catch ( std::exception & e)
    {
        std::cerr << "crclimpl::moveJoints exception " << e.what();
    }
#endif
    return RCS::Convert<geometry_msgs::Pose, tf::Pose>(p);
}

#ifdef MOVEIT
/**
 * @brief moveJoints moves the robot according to the joints
 * of  a kinemtaic model of the move_group robot to compute
 * the forward kinematic solution of the last link to get the pose at this
 * location.  Coordinate frame of result depends
 *  on the URDF definition (either command line xyz options
 * defining robot base coordiante space or the URDF base joint defining the
 * offset of the robot in the world coordinates.
 * @param positions joint positions
 * @param jointnames names of the joints if empty use move_group names
 * @param vel
 * @return
 */
moveit_msgs::RobotTrajectory crclimpl::moveJoints(std::vector<double> positions,
                                                  std::vector<std::string> jointnames,
                                                  double velscale)
{
    // FIXME: determine what joints names are actually moved assume all
    sensor_msgs::JointState jnts;
    geometry_msgs::Pose p ;

    try
    {
        const moveit::core::RobotModelPtr& kinematic_model = _robotModelLoader.getModel();
        std::cout << Globals.format("Model frame: %s", kinematic_model->getModelFrame().c_str());
        moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));

        const std::vector<std::string> & joint_names = _joint_model_group->getVariableNames();
        jnts.position=positions;
        jnts.name.insert(jnts.name.begin(), joint_names.begin(), joint_names.end());
        kinematic_state->setVariableValues(jnts);

        const std::vector< std::string > &links = _move_group->getLinkNames ();
        const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform(links.back());
        ::poseEigenToMsgImpl(end_effector_state,p);
    }
    catch ( std::exception & e)
    {
        std::cerr << "crclimpl::moveJoints exception " << e.what();
    }
    return moveTo(Convert<geometry_msgs::Pose, tf::Pose>(p));
}
#endif
std::string crclimpl::currentState()
{
    std::stringstream ss;

    tf::Pose now_pose = RCS::Convert<geometry_msgs::Pose, tf::Pose>(currentPose());
    ss << "Pose = " <<  RCS::dumpPoseSimple(now_pose) <<"\n";
    std::vector<double> jnts= currentJoints();
    ss << "Jnts = " <<  RCS::vectorDump(jnts,",")<<"\n";
    return ss.str();
}
geometry_msgs::Pose crclimpl::currentPose()
{
    geometry_msgs::Pose pose=RCS::Convert<tf::Pose, geometry_msgs::Pose>(rcs_robot.current_pose);
    return pose;
}
std::vector<double> crclimpl::currentJoints()
{
    // Need IK, don't have....
    return  std::vector<double> ();
}
moveit_msgs::RobotTrajectory crclimpl::eeMoveTo(geometry_msgs::Pose eeFinal,
                                              double velscale)
{
    tf::Pose pose =  RCS::Convert<geometry_msgs::Pose,tf::Pose> (eeFinal);
    return eeMoveTo(pose,velscale);
}
moveit_msgs::RobotTrajectory crclimpl::eeMoveTo(tf::Pose eeFinal,
                                                double velscale)
{
     try {
        tf::Pose rbtOrigin;

        if(Globals.bRobotInWorldCoordinates)
            //Convert robot origin coordinate space to world, remove  with gripper
            rbtOrigin = rcs_robot.addBase(eeFinal );
        else
            //This is robot origin final  coordinate space, remove  with gripper
            rbtOrigin = eeFinal;

        tf::Pose rbtFinal = rcs_robot.removeGripper(eeFinal);
        // Estimate distance and time taken to get there
        tf::Pose now_pose= rcs_robot.current_pose;

        std::cout << "crclimpl::eeMoveTo = " <<  RCS::dumpPoseSimple(now_pose) <<"\n";

        double d = sqrt(now_pose.getOrigin().distance2(rbtFinal.getOrigin()));
        doDwell(d*1.0);

        rcs_robot.current_pose=rbtFinal;

    }
    catch ( std::exception & e)
    {
        std::cerr << "crclimpl::moveTo exception " << e.what();
        assert(0);
    }
    return moveit_msgs::RobotTrajectory();
}

moveit_msgs::RobotTrajectory crclimpl::moveJoints(std::vector<double> positions,
                                                  std::vector<std::string> jointnames,
                                                  double velscale)
{
    // FIXME: determine what joints names are actually moved assume all
    rcs_robot.current_joints.position=positions;
    rcs_robot.current_joints.name=jointnames;
    return moveit_msgs::RobotTrajectory();

}

