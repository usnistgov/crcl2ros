#include <string>
#include <sys/stat.h>
#include <fcntl.h>
#include <iostream>
#include <sstream>

#include <readline/readline.h>

// xerces header includes
#include <xercesc/dom/DOM.hpp>
#include <xercesc/util/PlatformUtils.hpp>
#include <xercesc/framework/XMLGrammarPoolImpl.hpp>

#include "crclapp/rcs/Debug.h"
#include "crclapp/rcs/IRcs.h"
#include "crclapp/rcs/RCSThreadTemplate.h"

#include "crclapp/Globals.h"
#include "crclapp/Ros.h"
#include "crclapp/CommandLineInterface.h"
#include "crclapp/CrclWm.h"
#include "crclapp/Demo.h"
#include "crclapp/CrclSocketServer.h"
#include "crclapp/RosMsgHandler.h"
#include "crclapp/CrclRobotImpl.h"

#include "rcs/math.h"

using namespace crcl;

/// Build:
///    catkin build -DCMAKE_BUILD_TYPE=Debug
/// Run:
/// You need to use roslaunch: ( based on folder name of robot info)
///   roslaunch fanuc_lrmate200id_support  top.launch


int bBreak=1;// command line option for breaking into attached exe


/// Globals
std::shared_ptr<CGearDemo> geardemo;

/// Locals
static std::string PLANNING_GROUP="fanucarm"; /**< name of move group of joints (defined in sdrf) */
static int gzEnable=1;




/**
 * @brief startup first configures components:
 * 1) world model (how the demo objects are modeled)
 * 2) robot model (how the robot is modeled, including gripper).
 * 3) crcl - how crcl parameters are configured.
 * Then, creates the instances and assigns to global pointers.
 */
static void setup()
{

    /// CRCl Communication handler - bundles socket xml into messages then into ROS
    /// THESE ARE DEFAULTS OVERRIDEN BY ROSPARAM
    ////////////////////////////////////
    crcl::CBufferHandler::_bTrace = false;
    crcl::dbgCrclStatusMsg=false;
    crcl::dbgCrclCommandMsg=false;
    crcl::bAutoCrclStatus=1;
    crcl::bSynchronousCmds=1;  // no effect - all synchronous for now

    /// WM configuration - needs ROS nh and ns
    rcs_world.configure(CRos::nh.get(), rcs_world.robot);
    rcs_robot.printConfiguration();

    /// Robot configuration - needs robot prefix, ROS nh and ns
    rcs_robot.configure(rcs_world.robot, CRos::nh.get(), rcs_world.robot);
    rcs_robot.printConfiguration();

    /// Configure crcl streaming and message handling
    crcl::configure(CRos::nh.get(), rcs_world.robot);
    crcl::printConfiguration();


    // high level crcl API which generates a ros cmd message
    crclApi=std::shared_ptr<crcl::CCrcl2RosMsgApi>(new crcl::CCrcl2RosMsgApi());

    /// Set up CRCL 2 ROS XML message reader
    rosCmdStatMsgHandler=std::shared_ptr<crcl::CRosCmdStatusHandler> (new crcl::CRosCmdStatusHandler());
    rosCmdStatMsgHandler->setCmdQueue(NULL);
    rosCmdStatMsgHandler->setup(); // Setup subelements of crcl2ros - need joint names

    rosCmdMsgHandler= std::shared_ptr<crcl::CRosCmdMsgHandler>(new crcl::CRosCmdMsgHandler(rosCmdStatMsgHandler.get()));


    /// Set up CRCL XML socket reader/writer
    crclCommServer= std::shared_ptr<crcl::CSocketSession> ( new crcl::CSocketSession(0.01, "crclCommServer", crcl::crclPort));
    crcl::CSocketSession::AssignMessageQueue(&rosCmdStatMsgHandler->msgq);

    /// This sets up ROS to connect to gazebo and if enabled ros crcl topics
    rosCrclClient=  std::shared_ptr<CRosCrclClient>(new CRosCrclClient());

    rosCmdStatMsgHandler->setCmdQueue(&(rosCrclClient->crcltopiccmds));

    /// Initializes ROS communication to topic using robot prefix...
    rosCrclClient->setup(rcs_world.robot);

    /// Initialize xercesc used by code synthesis to parse CRCL XML
    xercesc::XMLPlatformUtils::Initialize();

    if(rosCmdStatMsgHandler->crclinterface->SetLengthUnits(crcl::CRosCmdStatusHandler::length_units) == crcl::CANON_FAILURE)
    {
        std::cout << "crcl2ros SetLengthUnits from param failed\n";
    }

    if(rosCmdStatMsgHandler->crclinterface->SetAngleUnits(crcl::CRosCmdStatusHandler::angle_units)== crcl::CANON_FAILURE)
    {
        std::cout << "crcl2ros SetAngleUnits from param failed\n";
    }

    crclRobotImpl=std::shared_ptr<crcl::CRobotImpl>(new crcl::CRobotImpl());

    // FIXME: this is hard coded...
    crclRobotImpl->nodeHandle(CRos::nh.get())
            .urdfRobotDescriptionRosParam(Globals.ns+"/robot_description")
            .moveGroupName(PLANNING_GROUP)
            .gzGripperTopic("/fanuc_lrmate200id/control")
            .jointStatePublisherTopic("/crcl/joint_states")
            .gzEnabled(gzEnable)
            .gzRobotModelName("lrmate")
            .init()
            .assertConfigurationValid();

}
static void startup()
{

    if(!Globals.bSingleThread)
    {
        rosCrclClient->start();  // reads model info
        rosCmdStatMsgHandler->start();       // convert crcl xml to ros msg
        rosCmdMsgHandler->start();    //  interpret ros cmd/status msg
    }
    // THis allows for no threading  instead sequential call sequence
    // reqruies each pseudo thread to be inited
    if(Globals.bSingleThread)
        RCS::Thread::initAll();

    // crcrl xml socket server and XML interpreter
    if(Globals.bCrclStreaming)
        crclCommServer->start(); // streams a CRCL socket
}

static void cleanup()
{

    // This order shuts things down.


    // Stopping application
    Globals.bRunning=false;

    if(!Globals.bSingleThread)
    {
        // subscribers must be shutdown...
        if(rosCrclClient.get() != nullptr)
            rosCrclClient->stop();
    }

// Shutdown ROS
    Ros.close();

    if(!Globals.bSingleThread)
    {
        // stop processing crcl->ros message
        if(rosCmdStatMsgHandler.get()!=nullptr)
            rosCmdStatMsgHandler->stop();

        // stop processing crcl->ros command messages commands
        if(rosCmdMsgHandler.get()!=nullptr)
            rosCmdMsgHandler->stop();
    }

    // stop code synthesis xercesc use
    xercesc::XMLPlatformUtils::Terminate();

    if(!Globals.bSingleThread)
    {
        RCS::Thread::stopAll(); // includes thread for Controller, robotstatus
        //stop communication thread that accepts queued XML messages and translates into CRCL
        if(crclCommServer.get()!=nullptr)
            crclCommServer->stop();
    }
}


int main(int argc, char** argv)
{
    try
    {
        std::vector<std::string> args;
        for(size_t i=0; i< argc; ++i)
        {
            args.push_back(argv[i]);
        }

        gzEnable = Globals.convert<int>(Globals.getCmdOption(args, "gzEnable:=", "1"));

#if 1
        bBreak = Globals.convert<int>(Globals.getCmdOption(args, "qtbreak:=", "0"));
        std::cout << "******** break = " << bBreak <<"\n";

        // if no break then delay for Gazebo visual to finish loading.
        if(!bBreak)
        {
            for(size_t j=0; j< 30; j++)
            {
                // sleep 1 second
                Globals.sleep(1000);
            }
        }
        while(bBreak)
        {
            // sleep 1 second
            Globals.sleep(1000);
        }
#endif


//        Globals.catchControlC();  // this must be here or ^C seems to be ignored

        /// Find roslaunch namespace
        ////////////////////////////////////
        Globals.ns = Globals.getCmdOption(args, "ns:=", "");
        if(Globals.ns.empty())
            Globals.ns = "/lrmate";
        if(Globals.ns[0]!='/')
            Globals.ns = "/"+Globals.ns;

        std::cout << "********SimMot namespace = " << Globals.ns <<"\n";

        /// RCS World robot - FULL NAME
        ////////////////////////////////////
        rcs_world.robot =Globals.getCmdOption(args, "robot:=", "");
        if(rcs_world.robot.empty())
           rcs_world.robot= "fanuc_lrmate200id";
        std::cout << "********Robot name  = " << rcs_world.robot <<"\n";

        /// Find planning group name
        ////////////////////////////////////
        std::string armgroup = Globals.getCmdOption(args, "armgroup:=","");
        if(!armgroup.empty())
            PLANNING_GROUP=armgroup;

        Globals.sRosPackageName ="_crclapp";
        Globals.sRosPackageName=rcs_world.robot+Globals.sRosPackageName;

        /// Setup up ROS
        ////////////////////////////////////
        Globals.sRosMasterUrl = "http://localhost:11311";
        Ros.init();

        // revisit armgroup name to see if ROS param
        armgroup.clear();
        if(CRos::nh->getParam(Globals.ns+"armgroup", armgroup))
            if(!armgroup.empty())
                PLANNING_GROUP=armgroup;
        ROS_DEBUG("********PLANNING_GROUP=%s",PLANNING_GROUP.c_str());

        // Wait till gazebo working?
        ros::service::waitForService("/gazebo/apply_joint_effort", -1);

        /// Set up ROS logging level
        ////////////////////////////////////
        std::string ros_loglevel=CRos::nh->param<std::string>(Globals.ns+"/debug/ros_loglevel", std::string{"Debug"});
        Ros.setupLogger(ros_loglevel);



        // read ROS params for configuration of demo and tests
        Globals.appConfig(CRos::nh);


        // Save ROS param server state
        Ros.parseRosLogPath(argv[0]);
        Ros.saveRosParam(*CRos::nh.get());


        // This configures, assigns instance pointers
        setup();


        // Kitting Demo setup
        geardemo=std::shared_ptr<CGearDemo>(new CGearDemo(crclApi));
        geardemo->init(CRos::nh.get(),rcs_world.robot);

        /// Sequence of tests.
        /// 1) test ROS and Gazebo working with rawTest
        /// 2) test ROS model topic reporting and internal inferencing
        /// working with kittingModelTest()
        /// 3) test high level Crcl API round trip through system with
        /// hard coded motion locations.
        /// 4)
        if(Globals.bRawTest)
            return geardemo->rawTest();

        if(Globals.bModelTest)
            return geardemo->kittingModelTest();

        if(Globals.bHighLevelCrclApiTest)
        {
            Globals.bSingleThread=0; // needs threads running
            startup();
            return geardemo->kittingMotionTest();
        }

        // either start thread, crcl comm server or if single thread init
        startup();

        // Read current state - joints, fk(joints), gripper
        // or we can assume homed first.
#define HOME
#ifdef HOME
        moveit_msgs::RobotTrajectory traj;
        // Reset robot to "home"
        std::vector<double> zero(6,0.0);
        traj=crclRobotImpl->moveJoints(zero);
        crclRobotImpl->move(traj);
        // status should be updated and joints all zero
#else
        crcl_rosmsgs::CrclStatusMsg statusmsg = crcl2rosmsg->createRosStatus(
                    RCS::CanonStatusType::CANON_DONE,
                    0, /// command num
                    rcs_robot.current_pose,
                    rcs_robot.current_joints,
                    rcs_robot.current_eesetting
                    );
        crcl2rosmsg->statusUpdate(statusmsg);
#endif

        /// This is the actual auto demo where you input using the CLI in the ros console
        /// Auto should use model information from gazebo
        if(Globals.bDemoTest)
        {
            geardemo->demoKittingTest(true, false); // single thread , cli
        }

        ros::waitForShutdown();
        std::cerr << "Cntrl C pressed  or CLI quit\n" << std::flush;

        cleanup();

    }
    catch (std::string & e)
    {
        std::cerr << Globals.strFormat("%s\nAt: %s\nReason: %s\n", "Abnormal exception end to crclapp ",  Globals.getTimeStamp().c_str(), e.c_str() );
        cleanup();
    }
    catch (std::exception & e)
    {
        std::cerr << Globals.strFormat("%s\nAt: %s\nReason: %s\n", "Abnormal exception end to crclapp ", Globals.getTimeStamp().c_str(), e.what());
        cleanup();
    }
    catch (...)
    {
        std::cerr << "Abnormal exception end to  crclapp at " <<  Globals.getTimeStamp().c_str();
        cleanup();
    }
    std::cout << "crclapp: Stopped " << Globals.getTimeStamp().c_str() << "\n" ;
}
