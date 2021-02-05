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

// ROS/Gazebo
#include <gazebo_msgs/SetModelState.h>
#include <ros/console.h>

// boost
#include <boost/algorithm/string.hpp>

// NIST RCS
#include "rcs/Core.h"
#include "rcs/IRcs.h"
#include "rcs/Debug.h"

// This app
#include "crclapp/Ros.h"
#include "crclapp/Globals.h"
#include "crclapp/CrclWm.h"
#include "crclapp/RosMsgHandler.h"
#include "crclapp/CrclRobotImpl.h"

ros::NodeHandlePtr CRos::nh;
CRos Ros;
ros::AsyncSpinner * CRos::_spinner;
////////////////////////////////////////////////////////////////////

CRosCrclClient::CRosCrclClient():
    RCS::Thread(.01, "CRosCrclClient")
{
    //this->crcl2ros=crcl2ros;
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
       ros::console::notifyLoggerLevelsChanged();
    }
}

////////////////////////////////////////////////////////////////////////////////
void CRosCrclClient::setup(std::string prefix)
{
    _prefix=prefix;
}

int CRosCrclClient::action()
{
    if(this->crcltopiccmds.size() > 0)
    {
#if 0
        crcl_rosmsgs::CrclCommandMsg cmdmsg = crclcmds.pop();
        publishCrclCommand(cmdmsg);
#endif
    }
    return 1;
}
////////////////////////////////////////////////////////////////////////////////
void CRosCrclClient::init()
{

    // this is a hard coded topic name  BUT SHOULD NOT BE
    _guiJointStatePublisher = Ros.nh->subscribe("/lrmate/gui/joint_states", 1, &CRosCrclClient::guiJointStatesCallback,this);

#ifdef GAZEBO
    // this is a hard coded topic name from gazebo_ros_api plugin
    _gzWorldModel = Ros.nh->subscribe("/gazebo/model_states", 1, &CRosCrclClient::gzModelStatesCallback,this);
#endif
    // This is never used...
    //Thread::start();
}

////////////////////////////////////////////////////////////////////////////////
void CRosCrclClient::stop()
{
    _guiJointStatePublisher.shutdown();
#ifdef GAZEBO
    _gzWorldModel.shutdown();
#endif
    Thread::stop(true);
}

void CRosCrclClient::guiJointStatesCallback(const sensor_msgs::JointStateConstPtr jntupdate)
{
    for(size_t i=0; i< jntupdate->name.size(); i++ )
    {
        std::cout << jntupdate->name[i] << ":";
        moveit_msgs::RobotTrajectory traj;
        std::vector<double> jnts=jntupdate->position;
        traj=crcl::crclRobotImpl->moveJoints(jnts);
        crcl::crclRobotImpl->move(traj);

    }
    std::cout << "\n";
}

#ifdef GAZEBO
void CRosCrclClient::gzModelStatesCallback(const gazebo_msgs::ModelStates &gzstate_current)
{
    static int nTest=0;
    for(size_t i=0; i< gzstate_current.name.size(); i++ )
    {
        std::string name = gzstate_current.name[i];
        tf::Pose pose = RCS::Convert<geometry_msgs::Pose, tf::Pose >(gzstate_current.pose[i]);
        std::string meshfile;
        tf::Vector3 scale(1.,1.,1.);

        // only save links for gear related models.
        if(name.find("sku")==std::string::npos)
            continue;

        // Lock updates of instance and model link to id
        {
            std::lock_guard<std::mutex> guard(_mymutex);
            WorldModel::instances.storeInstance(name, pose, meshfile, scale);
        }
        if(Globals.bReadAllInstances!=true)
        {
            if(name.find("part")!=std::string::npos)
                rcs_world._gearStartingPoses[name]=pose;
        }
        else
        {
            WorldModel::instances.firstOrderLogic();
            for(size_t i=0; i< WorldModel::instances.size(); i++)
            {
                // skip objects not part of this robot workspace
                if(!WorldModel::instances[i].inMyWorld())
                        continue;

                tf::Pose pose = WorldModel::instances[i].centroid();
                crcl::rosCmdStatMsgHandler->crclinterface->crclsettings.Update(WorldModel::instances[i].name(), pose);
                crcl::rosCmdStatMsgHandler->crclinterface->crclsettings.Update(WorldModel::instances[i]);
            }
        }
        if(Globals.dbgModelInferences )
        {
            ROS_DEBUG_THROTTLE(10, WorldModel::instances.dumpInferences().c_str());
        }
    }


}

void CRosCrclClient::reset()
{
    std::lock_guard<std::mutex> guard(_mymutex);
    geometry_msgs::Twist start_twist;
    start_twist.linear.x = 0.0;
    start_twist.linear.y = 0.0;
    start_twist.linear.z = 0.0;
    start_twist.angular.x = 0.0;
    start_twist.angular.y = 0.0;
    start_twist.angular.z = 0.0;

    std::map<std::string, tf::Pose>::iterator gearit;

    for(gearit= rcs_world._gearStartingPoses.begin();
        gearit!=rcs_world._gearStartingPoses.end();
        ++gearit
        )
    {
        ros::ServiceClient client = Ros.nh->serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
        gazebo_msgs::SetModelState setmodelstate;
        gazebo_msgs::ModelState modelstate;
        modelstate.model_name = (*gearit).first;;
        modelstate.reference_frame = "world";

        modelstate.pose = RCS::Convert<tf::Pose,geometry_msgs::Pose>((*gearit).second);
        modelstate.twist = start_twist;

        setmodelstate.request.model_state = modelstate;

        if (!client.call(setmodelstate))
        {
            ROS_ERROR("Failed to call gazebo_ros_api setmodelstate.request service");
        }
    }

}


#endif
///////////////////////////////////////////////////////////////
///  CRos
//////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////
CRos::CRos()
{
}
////////////////////////////////////////////////////////////////////////////////

void CRos::setupLogger(std::string logger_level)
{
    if(boost::iequals(logger_level, "Debug"))
        ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                       ros::console::levels::Debug) ;
    if(boost::iequals(logger_level, "Info"))
        ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                       ros::console::levels::Info) ;
    if(boost::iequals(logger_level, "Warn"))
        ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                       ros::console::levels::Warn) ;
    if(boost::iequals(logger_level, "Error"))
        ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                       ros::console::levels::Error) ;
    if(boost::iequals(logger_level, "Fatal"))
        ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                       ros::console::levels::Fatal) ;
     ros::console::notifyLoggerLevelsChanged();
}

////////////////////////////////////////////////////////////////////////////////
void CRos::init()
{
    // Wait for ROS master to be running -  use rosout, rosmaster is python program
    std::cout << "Connecting to ROS master ";
    while( Globals.procFind("rosout") < 0)
    {
        Globals.sleep(1000);
        std::cout << "." << std::flush;
    }
    std::cout << "\n" << std::flush;
    Globals.sleep(1000);


    ros::M_string remappings;
    remappings["__master"]=  Globals.sRosMasterUrl.c_str(); //
    remappings["__name"]= Globals.sRosPackageName; //Globals.AppProperties["Package"];

    // Initialize ROS environment ROS_MASTER_URI MUST BE SET or seg fault
    ros::init(remappings,Globals.sRosPackageName) ; // Globals.AppProperties["Package"]);

    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    //  Required for multithreaded ROS communication  NOT TRUE: if not ros::spinOnce
    _spinner = new ros::AsyncSpinner(2);
    _spinner->start();

}

////////////////////////////////////////////////////////////////////////////////
void CRos::close()
{
#ifdef ROS
    if(Globals.bRos)
        ros::shutdown();
#endif
}

