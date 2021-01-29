#include <gazebo_msgs/SetJointTrajectory.h>
#include <gazebo_msgs/SetModelConfiguration.h>
#include <gazebo_msgs/GetModelState.h>
#include <sensor_msgs/JointState.h>
#include <chrono>

// application specific includes
#include "crclapp/CrclRobotImpl.h"
#include "crclapp/Globals.h"
#include "crclapp/gripper.h"
#include "crclapp/CrclWm.h"

#include <rcs/Conversions.h>
#include <rcs/Debug.h>
#include <rcs/math.h>

std::string moveit_error_str(int errcode)
{
    switch(errcode)
    {
    case moveit_msgs::MoveItErrorCodes::SUCCESS:  return "SUCCESS";
    case moveit_msgs::MoveItErrorCodes::FAILURE : return "FAILURE";
    case moveit_msgs::MoveItErrorCodes::PLANNING_FAILED: return "PLANNING_FAILED";
    case moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN: return "INVALID_MOTION_PLAN";
    case moveit_msgs::MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE: return "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE";
    case moveit_msgs::MoveItErrorCodes::CONTROL_FAILED: return "CONTROL_FAILED";
    case moveit_msgs::MoveItErrorCodes::UNABLE_TO_AQUIRE_SENSOR_DATA: return "UNABLE_TO_AQUIRE_SENSOR_DATA";
    case moveit_msgs::MoveItErrorCodes::TIMED_OUT: return "TIMED_OUT";
    case moveit_msgs::MoveItErrorCodes::PREEMPTED: return "PREEMPTED";
    case moveit_msgs::MoveItErrorCodes::START_STATE_IN_COLLISION: return "START_STATE_IN_COLLISION";
    case moveit_msgs::MoveItErrorCodes::START_STATE_VIOLATES_PATH_CONSTRAINTS: return "START_STATE_VIOLATES_PATH_CONSTRAINTS";
    case moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION: return "GOAL_IN_COLLISION";
    case moveit_msgs::MoveItErrorCodes::GOAL_VIOLATES_PATH_CONSTRAINTS: return "GOAL_VIOLATES_PATH_CONSTRAINTS";
    case moveit_msgs::MoveItErrorCodes::GOAL_CONSTRAINTS_VIOLATED: return "GOAL_CONSTRAINTS_VIOLATED";
    case moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME: return "INVALID_GROUP_NAME";
    case moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS: return "INVALID_GOAL_CONSTRAINTS";
    case moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE: return "INVALID_ROBOT_STATE";
    case moveit_msgs::MoveItErrorCodes::INVALID_LINK_NAME: return "INVALID_LINK_NAME";
    case moveit_msgs::MoveItErrorCodes::INVALID_OBJECT_NAME: return "INVALID_OBJECT_NAME";
    case moveit_msgs::MoveItErrorCodes::FRAME_TRANSFORM_FAILURE: return "FRAME_TRANSFORM_FAILURE";
    case moveit_msgs::MoveItErrorCodes::COLLISION_CHECKING_UNAVAILABLE: return "COLLISION_CHECKING_UNAVAILABLE";
    case moveit_msgs::MoveItErrorCodes::ROBOT_STATE_STALE: return "ROBOT_STATE_STALE";
    case moveit_msgs::MoveItErrorCodes::SENSOR_INFO_STALE: return "SENSOR_INFO_STALE";
    case moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION: return "NO_IK_SOLUTION";
    default:
        return "Not valid error code";
    }

    return "Not valid error code";
}
namespace RCS
{
std::string dumpPoseSimple(geometry_msgs::Pose pose)
{
    return dumpPoseSimple(Convert<geometry_msgs::Pose, tf::Pose>( pose));
}
}

using namespace RCS;
using namespace crcl;

template<typename T>
void poseEigenToMsgImpl(const T &e, geometry_msgs::Pose &m)
{
    m.position.x = e.translation()[0];
    m.position.y = e.translation()[1];
    m.position.z = e.translation()[2];
    Eigen::Quaterniond q = (Eigen::Quaterniond)e.linear();
    m.orientation.x = q.x();
    m.orientation.y = q.y();
    m.orientation.z = q.z();
    m.orientation.w = q.w();
    if (m.orientation.w < 0) {
        m.orientation.x *= -1;
        m.orientation.y *= -1;
        m.orientation.z *= -1;
        m.orientation.w *= -1;
    }
}

// http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/move_group_interface/move_group_interface_tutorial.html#cartesian-paths
CRobotImpl::CRobotImpl()
{
    this->nh=nullptr;
    this->PLANNING_GROUP.clear();
    this->urdf_robot_description_param.clear();
    this->_robot_name.clear();
    this->gz_gripper_topic.clear();
    bConfigException=false; // nullify configuration
    this->gz_enabled=true;

}
CRobotImpl::~CRobotImpl()
{
    _gzSetJointTraj.shutdown();
    _gzGetJointTraj.shutdown();
    _robotJointState.shutdown();
}
bool CRobotImpl::isConfigurationValid()
{
    if(this->nh== nullptr ||
            this->PLANNING_GROUP.empty() ||
            this->urdf_robot_description_param.empty() ||
            this->_robot_name.empty()||
            this->joint_state_publisher_topic.empty() ||
            _joint_model_group==nullptr||
            _move_group.get() == nullptr ||
            this->_gzrobot_name.empty() ||
            _robot_model.get() == nullptr)
        return false;
    return true;
}
CRobotImpl & CRobotImpl::assertConfigurationValid()
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
CRobotImpl & CRobotImpl::init()
{
    try{

        _robotModelLoader= robot_model_loader::RobotModelLoader(urdf_robot_description_param);

        _robot_model = robot_model::RobotModelPtr(_robotModelLoader.getModel());
        _robot_name=_robot_model->getName();

        _move_group=MoveGroupInterfacePtr(new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP));

        /* Create a RobotState and JointModelGroup to keep track of the current robot pose and planning group*/

        //RobotState is the object that contains all the current position/velocity/acceleration data.
        _robot_state=robot_state::RobotStatePtr (new robot_state::RobotState(_robot_model));

        //ROSCPP_DECL bool exists(const std::string& service_name, bool print_failure_reason);
        moveit::core::RobotModelPtr kinematic_model = _robotModelLoader.getModel();
        _joint_model_group = kinematic_model->getJointModelGroup(PLANNING_GROUP);

        // Save joint names for this move_group
        const std::vector<std::string> & joint_names = _joint_model_group->getVariableNames();
        this->joint_names.insert(this->joint_names.begin(), joint_names.begin(), joint_names.end());

        if(gz_enabled)
        {
            // gazebo ros topic to set joint positions
            _gzSetJointTraj = nh->serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");

            // gazebo ros topic to get joint positions
            _gzGetJointTraj= nh->serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
        }
        // ros topic to set joint positions for joint_state_publisher
        _robotJointState = nh->advertise<sensor_msgs::JointState>(this->joint_state_publisher_topic, 1);

        // gazebo plugin ros service for setting gripper.
        //gripper.init(nh, "/fanuc_lrmate200id/control");
        if(gz_enabled)
            gripper.init(nh, gz_gripper_topic);
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

int CRobotImpl::setGripper(double d)
{
    if(!gz_enabled)
        return 0;
    if(d==0.0)
        gripper.close();
    else
        gripper.open();
    return d;
}

int CRobotImpl::doDwell(double seconds)
{
    //FIXME: if seconds < 0.0001 is zero sleep should be sleep or yield
    //std::this_thread::sleep_for(std::chrono::milliseconds((int64_t) (seconds*1000.)));
    Globals.sleep(seconds*1000.0);
    std::cerr << "Done dwell " << seconds <<"\n";
    return 0;
}

void CRobotImpl::publishFakeJoints(trajectory_msgs::JointTrajectoryPoint pt)
{
    std::vector<double> jts = pt.positions;

    // If no one listening don't publish
    if( _robotJointState.getNumSubscribers()==0)
    {
        std::cerr << "crclimpl::publishFakeJoints NO ONE LISTENING?!?!\n";
        std::cerr << "Check that joint_state_publisher and _robotJointState topic names match EXACTLY\n";
    }

    sensor_msgs::JointState jntCmdMsg;
    const std::vector<std::string> & joint_names = _joint_model_group->getVariableNames();
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

std::vector<double>  CRobotImpl::IK(tf::Pose pose )
{
    std::vector<double> joint_values;
    const moveit::core::RobotModelPtr& kinematic_model = _robotModelLoader.getModel();
    //std::cout << Globals.format("Model frame: %s", kinematic_model->getModelFrame().c_str());
    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));


    geometry_msgs::Pose p = RCS::Convert<tf::Pose, geometry_msgs::Pose> (pose);
    bool found_ik = kinematic_state->setFromIK(_joint_model_group, p, 10, 0.1);

    if(found_ik)
    {
        kinematic_state->copyJointGroupPositions(_joint_model_group, joint_values);
    }
    else
    {
        ROS_INFO_STREAM("Did not find IK solution"<< RCS::dumpPoseSimple(pose));
    }
    return joint_values;
}

tf::Pose CRobotImpl::FK(std::vector<double> positions)
{
    // FIXME: determine what joints names are actually moved
    sensor_msgs::JointState jnts;
    geometry_msgs::Pose p ;

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
    return Convert<geometry_msgs::Pose, tf::Pose>(p);
}


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
moveit_msgs::RobotTrajectory CRobotImpl::moveJoints(std::vector<double> positions,
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
std::string CRobotImpl::currentState()
{
    std::stringstream ss;

    tf::Pose now_pose = Convert<geometry_msgs::Pose, tf::Pose>(currentPose());
    ss << "Pose = " <<  RCS::dumpPoseSimple(now_pose) <<"\n";
    std::vector<double> jnts= currentJoints();
    ss << "Jnts = " <<  RCS::vectorDump(jnts,",")<<"\n";
    return ss.str();
}
geometry_msgs::Pose CRobotImpl::currentPose()
{
    geometry_msgs::Pose now_pose = _move_group->getCurrentPose().pose;
    return now_pose;
}
std::vector<double> CRobotImpl::currentJoints()
{
    //    std::vector<double> group_variable_values;
    //   move_group->getCurrentState()->copyJointGroupPositions(move_group->getCurrentState()->getRobotModel()->getJointModelGroup(move_group->getName()),    group_variable_values);
    //   return group_variable_values;
    return _move_group->getCurrentJointValues();

}

moveit_msgs::RobotTrajectory CRobotImpl::eeMoveTo(tf::Pose eeFinal,
                                                  double velscale)
{
    moveit_msgs::RobotTrajectory trajectory;
    std::vector<geometry_msgs::Pose> waypoints;
    try {
        tf::Pose rbtOrigin;

        if(Globals.bRobotInWorldCoordinates)
            //Convert robot origin coordinate space to world
            rbtOrigin = rcs_robot.addBase(eeFinal );
        else
            //This is robot origin final  coordinate space
            rbtOrigin = eeFinal;

        // remove  with gripper
        tf::Pose rbtFinal=rbtOrigin;
        rbtFinal = rcs_robot.removeGripper(rbtOrigin);
        trajectory=moveTo(rbtFinal,  .1);

        //trajectory=moveTo(eeFinal,  .1);

        move(trajectory);
    }
    catch ( std::exception & e)
    {
        std::cerr << "crclimpl::moveTo exception " << e.what();
        assert(0);
    }
    return trajectory;
}
/**
 * @brief moveTo robot coordinate system trajectory to calculate
 * a trajectory. Uses current move_gropu position as the starting trajectory
 * point.
 * @param final  final point of the
 * @param velscale scale of the velcotiy maximum between points. Final velcotiy
 * IS ALWAYS ZERO THERE IS **NO** BLENDING BETWEEEN WAYPOINTS so avoid short trajectories as
 * this could war your robot staritng and stopping. At least in  theory, but
 * very few trajectoy planners blend anyway. So we have skipped waypoints as
 * moveit stops (zero velocity) at each waypoint when planning the trajectory.
 * TBH, specifying the blend can be complicated (undershoot, overshoot, cloverleaf, etc.)
 * @return
 */
moveit_msgs::RobotTrajectory CRobotImpl::moveTo(tf::Pose rbtFinal,
                                                double velscale)
{
    moveit_msgs::RobotTrajectory trajectory;
    std::vector<geometry_msgs::Pose> waypoints;
    try {
        // THis is robot coordinate space without gripper
        geometry_msgs::Pose now_pose = _move_group->getCurrentPose().pose;
        tf::Pose curpose=Convert<geometry_msgs::Pose,tf::Pose>(now_pose);
        //double d = curpose.getOrigin().distance(rbtFinal.getOrigin());


        geometry_msgs::Pose final_pose=Convert<tf::Pose, geometry_msgs::Pose>( rbtFinal);
        //std::vector<double> jts = move_group->getCurrentJointValues()


        if(crcl::bDebug)
        {
            ROS_INFO_STREAM( "Current pose" << RCS::dumpPoseSimple(now_pose) << std::endl);
            ROS_INFO_STREAM( "Goal pose" << RCS::dumpPoseSimple(final_pose) << std::endl);
        }

        // handle waypoints

        if(!Globals.bMoveit)
        {
            trajectory.joint_trajectory.header.frame_id="/world";
            trajectory.joint_trajectory.header.seq=0;
            trajectory.joint_trajectory.header.stamp= ros::Time::now();
            trajectory.joint_trajectory.joint_names =this->joint_names;


            // Did not matter
            for(double dIncrement=0.0; dIncrement<1.0; )
            {
                // Change from straight line to s curve
                double t=S_Curve::scurve(dIncrement);
                tf::Pose waypoint;
                waypoint.setOrigin(curpose.getOrigin().lerp(rbtFinal.getOrigin(), t));
                waypoint.setRotation(curpose.getRotation().slerp(rbtFinal.getRotation(), t));
                geometry_msgs::Pose final_waypoint=Convert<tf::Pose, geometry_msgs::Pose>( waypoint);
                //waypoints.push_back(final_waypoint);
                ::trajectory_msgs::JointTrajectoryPoint pt;

                // fixme: differentiate vel, acc values t-1, t-2
                pt.accelerations={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
                pt.velocities={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
                pt.positions=this->IK(waypoint);

                // simple test to see if IK worked. Should get all joint positions
                if(pt.positions.size() == trajectory.joint_trajectory.joint_names.size())
                    trajectory.joint_trajectory.points.push_back(pt);

                dIncrement=dIncrement+0.1;
            }
        }
        else {
            // should have final pose if to from 0..1
            waypoints.push_back(now_pose);
            waypoints.push_back(final_pose);

            // Scaling factor of the maximum speed of each joint
            _move_group->setMaxVelocityScalingFactor(0.1);
            _move_group->setMaxAccelerationScalingFactor(0.1);
            _move_group->setGoalOrientationTolerance(0.1);
            _move_group->setGoalJointTolerance(0.1);
            _move_group->setGoalPositionTolerance(0.1);
            _move_group->setGoalTolerance(0.1);

            // THese have all been moved to ROS params
            // plan the trajectory Cartesian motion
//            double jump_threshold = 0.00; // zero means ignore since KDL
//            //double jump_threshold = 0.01; // zero means ignore since KDL
//            double eef_step = 0.01;  // don't know if this per joint or cumulative
//            bool avoid_collisions=true;
            moveit_msgs::MoveItErrorCodes error_code;

            // Compute a Cartesian path that follows specified waypoints.

            // eef_step a step size of at most eef_step meters between
            // end effector configurations of consecutive points in the
            // result trajectory.

            // The reference frame for the waypoints
            // is that specified by setPoseReferenceFrame().

            // jump_threshold No more than jump_threshold is allowed as change in
            // distance in the configuration space of the robot
            // (this is to prevent 'jumps' in IK solutions). 0.0 disables

            // avoid_collisions Collisions are avoided if avoid_collisions is set to true.
            // If collisions cannot be avoided, the function fails.
            // Return a value that is between 0.0 and 1.0 indicating the
            // fraction of the path achieved as described by the waypoints.
            // Return -1.0 in case of error.
            double dDone=0;

            for(size_t j=0; j< 5 && dDone < 1.0 ; j++)
            {
                // FAILED until https://stackoverflow.com/questions/50609663/positionconstraint-goal-for-robot-arm-unable-to-construct-goal-representation
                _move_group->setStartStateToCurrentState();

                dDone = _move_group->computeCartesianPath(waypoints,
                                                          Globals.eef_step,
                                                          Globals.jump_threshold,
                                                          trajectory,
                                                          Globals.avoid_collisions,
                                                          &error_code
                                                          );
                if(error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
                {
                    ROS_FATAL_STREAM("computeCartesianPath error="<< moveit_error_str(error_code.val));
                }
                else if(dDone < 0.0 )
                {
                    ROS_FATAL_STREAM( "move_group->computeCartesianPath failed % done = " << dDone << " vel="<<velscale << " eef_step="<<Globals.eef_step);
                }
                else if(dDone <1.0 )
                {
                    ROS_DEBUG_STREAM( "move_group->computeCartesianPath only completed % done = " << dDone << " vel="<<velscale  << " eef_step="<<Globals.eef_step);
                    velscale=velscale/5.0;
                    Globals.eef_step=Globals.eef_step*2.;
                    _move_group->setMaxVelocityScalingFactor(velscale);
                }

            }
            if(dDone <1.0 )
            {
                ROS_ERROR_STREAM( "********************move_group->computeCartesianPath() only completed : = " << dDone << "%" );
                //assert(0);
            }
        }

    }
    catch ( std::exception & e)
    {
        ROS_FATAL_STREAM("climpl::moveTo exception " << e.what());
    }
    catch(...)
    {
        ROS_FATAL_STREAM("climpl::moveTo exception ");

    }

    return trajectory;
}

/**
 * @brief crclimpl::publishJoints bang bang move to joint positions
 * at next cycle. Assume simulation only. FIXME: check.
 * @param positions all joint positions
 */
void CRobotImpl::publishJoints(std::vector<double> positions)
{
    moveit_msgs::RobotTrajectory trajectory;
    trajectory.joint_trajectory.header.frame_id="/world";
    trajectory.joint_trajectory.header.seq=0;
    trajectory.joint_trajectory.header.stamp= ros::Time::now();
    trajectory.joint_trajectory.joint_names =this->joint_names;

    ::trajectory_msgs::JointTrajectoryPoint pt;
    pt.accelerations={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    pt.velocities={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    pt.positions=positions; // {0.1,0.1,0.1,0.1,0.1,0.1,0.1};
    trajectory.joint_trajectory.points.push_back(pt);
    move(trajectory);
}

/**
 * @brief move "pysically" move the robot given the trajectory.
 * Of not, only simulation USING GAZEBO is done - using the
 * move_grpou joints theses are snet to the gazel model containing
 * these joints as defined initially by the move_group, and these
 * joints are also updated by sending a command to joint_state_publisher
 * which is spawned with tthe ROS URDF parameter set - so if multiple
 * robots are defined, there can be multiple publishers to the  joint_state_publisher
 * ros node, WHICH HAS NOT BEEN TESTED.  The hope is that there can be TWO OR MORE
 * move_it trajectory SHIMDS to command different robots in gazebo (using one URDF
 * to define the multiple robots) that each has its own app to commmand and control
 * these gazebo robots based on CRCL API (which is synchronous to the crcl robot
 * app - which may be a problem).
 */
void CRobotImpl::move(moveit_msgs::RobotTrajectory trajectory)
{
    try {
        trajectory_msgs::JointTrajectoryPoint pt;

        for(size_t i=0; i< trajectory.joint_trajectory.points.size();++i)
        {
            pt = trajectory.joint_trajectory.points[i];


            if(gz_enabled)
            {
                gazebo_msgs::SetModelConfiguration traj;
                traj.request.model_name=this->_gzrobot_name;  //fanuc_lrmate200id
                traj.request.urdf_param_name="";
                traj.request.joint_names=this->joint_names;
                for(size_t k=0; k< joint_names.size(); k++)
                    traj.request.joint_names[k]="fanuc_lrmate200id::"+traj.request.joint_names[k];
                for(size_t k=0; k<pt.positions.size(); ++k)
                    traj.request.joint_positions.push_back(trajectory.joint_trajectory.points[i].positions[k]);

                Globals.sleep(50);
                if (!_gzSetJointTraj.call(traj))
                {
                    ROS_ERROR("Failed to call service /gazebo/set_model_configuration\n");
                }
            }
            // publish joints to joint state publisher using renamed topic
            publishFakeJoints(pt);
            ROS_INFO("%s", currentState().c_str());
        }

#if 0
        // Now moveit update (to rviz)
        moveit::planning_interface::MoveGroupInterface::Plan goal_plan;
        goal_plan.trajectory_ = trajectory;
        if(!move_group->execute(goal_plan))
        {
            std::cerr << "move_group->(goal_plan)failed\n";

        }
#endif

        //if(Globals.bDebug)
        ROS_INFO("%s", currentState().c_str());

    }
    catch ( std::exception & e)
    {
        ROS_ERROR_STREAM( "crclimpl::move exception " << e.what());
    }
}


