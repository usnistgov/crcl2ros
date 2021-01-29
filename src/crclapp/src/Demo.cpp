

/*
DISCLAIMER:
This software was produced by the National Institute of Standards
and Technology (NIST), an agency of the U.S. government, and by statute is
not subject to copyright in the United States.  Recipients of this software
assume all responsibility associated with its operation, modification,
maintenance, and subsequent redistribution.

See NIST Administration Manual 4.09.07 b and Appendix I.
 */
//#pragma message "Compiling " __FILE__ 
#include <memory>
#include <mutex>

#include "crclapp/Demo.h"
#include "crclapp/Globals.h"
#include "crclapp/Shape.h"
#include "crclapp/Crcl2RosMsgApi.h"
#include "crclapp/RosMsgHandler.h"
#include "crclapp/CrclWm.h"
#include "crclapp/Ros.h"
#include "crclapp/CommandLineInterface.h"


using namespace RCS;
using namespace WorldModel;

// polled wait on function with timeout

#include <chrono>
#include <functional>
typedef std::function<bool ()> TPollFcn;

using namespace crcl;

// static definitions
int CGearDemo::bDebug;

// methods
int CGearDemo::init(ros::NodeHandle *nh, std::string ns)
{

    Globals.bClosestFree=nh->param<int>(Globals.ns+"/app/closest/free",int{0});
    Globals.bClosestOpenSlot=nh->param<int>(Globals.ns+"/app/closet/openslot",int{0});
    double dDwellTime = nh->param<double>(Globals.ns+"/app/dwell/time", double{1000.0});
    double dGraspingDwellTime = nh->param<double>(Globals.ns+ "/app/dwell/grasping", double{1000.0});
    if(crcl::crclApi.get() == NULL)
    {
        std::cout << "Demo assigning parameter to NULL crclApi\n";
        return -1;
    }
    crcl::crclApi->setDwell(dDwellTime);
    crcl::crclApi->setGraspingDwell(dGraspingDwellTime);
    crcl::crclApi->medium();
    return 0;
}


////////////////////////////////////////////////////////////////////////////////
CGearDemo::CGearDemo(std::shared_ptr<crcl::CCrcl2RosMsgApi>  crclApi) : _crclapiPtr(crclApi)
{
    //    WorldModel::CShapes::initDefinitions();
}

////////////////////////////////////////////////////////////////////////////////
/// \brief CGearDemo::issueRobotCommands
/// \param state
/// \return
#if 0
int CGearDemo::issueRobotCommands(int & state)
{

    // Finish queuing commands before handling them....
    //std::unique_lock<std::mutex> lock(cncmutex);
    int cmdnum;
    // Must declare all variables beforehand
    RCS::CCanonCmd cmd;
    tf::Pose pickpose;
    std::string gearname;
    tf::Pose affpose;
    tf::Pose gripperoffset;
    tf::Quaternion bend;
    tf::Vector3 offset;
    tf::Pose slotpose;
    tf::Pose slotoffset;
    tf::Pose placepose;

    if( WorldModel::instances.size()==0)
    {
        std::cerr << "Error: No gear instances can be read from Gazebo model- restart Gazebo!\n";
        return -1;
    }

    switch(state)
    {
    case 0:
    {
        // Find a free gear
        if ((_gear = WorldModel::instances.findFreeGear(rcs_world.part_list, rcs_robot.currentPose)) == NULL)
        {
            ROS_FATAL_ONCE ( "Error: No Free Gear in tray to move");
            return -1;
        }
        return state++;
    }
    case 1:
    {

        gearname = _gear->_name;
        // Ok we will move this gear - mark as no longer free standing
        _gear->_attributes["state"] = "stored";

        affpose =  _gear->_location ;
        affpose = rcs_robot.basePose.inverse() * affpose ;

        // The object gripper offset is where on the object it is to be gripped
        // e.g., offset.gripper->largegear = 0.0,0.0, -0.030, 0.0, 0.0.,0.0
        gripperoffset = rcs_robot.gripperoffset[_gear->_type];
        // gripperoffset = tf::Pose(tf::QIdentity(),tf::Vector3(0.0,0.0, - _instance->_height * .8));

        bend=rcs_robot.QBend;

        // The gripperoffset is the robot gripper offset back to the 0T6 equivalent
        pickpose =  tf::Pose(bend, affpose.getOrigin()) * gripperoffset ;

        offset = pickpose.getOrigin();

        // Retract
        int cmdnum = _crclapiPtr->moveTo(rcs_robot.Retract * tf::Pose(bend, offset));
        while(this->isWorking(cmdnum))
            Globals.sleep(100);
        cmdnum = _crclapiPtr->doDwell(_crclapiPtr->_mydwell);
        while(this->isWorking(cmdnum))
            Globals.sleep(100);
        return state++;
    }
    case 2:
    {
        _crclapiPtr->moveTo(tf::Pose(bend, offset) );
        _crclapiPtr->doDwell(_crclapiPtr->_mydwell);
        return state++;
    }
    case 3:
    {
        _crclapiPtr->closeGripper();
        _crclapiPtr->doDwell(_crclapiPtr->_mygraspdwell);
        return state++;
    }
    case 4:
    {
        _crclapiPtr->moveTo(rcs_robot.Retract * tf::Pose(bend, offset));
        _crclapiPtr->doDwell(_crclapiPtr->_mydwell);
        return state++;
    }

    case 5:
    {
        // Find a kit slot and then its offset from the centroid of the kit
        // Find a gear slot in a kit
        WorldModel::CShape * kit=NULL;
        WorldModel::CShape * slot=NULL;
        if(!WorldModel::instances.findFreeGearKitSlot(_gear,
                                                      slotpose,
                                                      rcs_world.part_list))
        {
            static int bThrottle=1;
            if(bThrottle>0)
                std::cout << "Error: No Free Kit Slot\n";
            bThrottle--;
            return -1;
        }

        slotpose = rcs_robot.basePose.inverse() * slotpose;
        slotoffset = rcs_world.slotoffset["vessel"];
        placepose = tf::Pose(bend, slotpose.getOrigin())* slotoffset; // fixme: what if gear rotated
        offset = placepose.getOrigin();

        // Approach
        _crclapiPtr->moveTo(rcs_robot.Retract* tf::Pose(bend, offset));
        _crclapiPtr->doDwell(_crclapiPtr->_mydwell);
        return state++;
    }
    case 6:
    {
        // Place the grasped object
        _crclapiPtr->moveTo(tf::Pose(bend, offset));
        _crclapiPtr->doDwell(_crclapiPtr->_mydwell);
        return state++;
    }
    case 7:
    {
        // open gripper and wait
        _crclapiPtr->openGripper();
        _crclapiPtr->doDwell(_crclapiPtr->_mygraspdwell);
        return state++;
    }
    case 8:
    {
        // Retract from placed object
        //r->Retract(0.04);
        _crclapiPtr->moveTo(rcs_robot.Retract * tf::Pose(bend, offset));
        _crclapiPtr->doDwell(_crclapiPtr->_mydwell);
        return state++;
    }
    }
    state=0;
    return state;

}
#endif
CShape * CGearDemo::findFreeGear(WorldModel::CInstances &now_instances, std::string geartype)
{
    for(size_t i=0; i< now_instances.size(); i++)
    {
        CShape & instance(now_instances[i]);

        // search if one of my robot kits/vessel, if not continue
        if(std::find(rcs_world.part_list.begin(), rcs_world.part_list.end(), instance._name)==rcs_world.part_list.end())
            continue;

        if(!instance.isVessel())
        {
            continue;
        }

        // kit - now do slots
        CShape * slotz = now_instances.getDefinition(instance.type());

        if(slotz==NULL)
            continue;

        for(size_t j=0; j < slotz->_contains.size(); j++)
        {
            CShape & slot(slotz->_contains[j]);
            std::map<std::string, std::string> &p =  instance._properties[slot.name()];
            if(p["type"]==geartype && p["state"]!="open")
            {
                // look up gear by name
                return now_instances.findInstance(p["state"]);
            }
        }

    }
    return nullptr;
}
bool CGearDemo::findOpenKittingGearSlot(WorldModel::CInstances &now_instances,
                                        CShape & kit,
                                        std::map<std::string, std::string> &slotprop)
{
    for(size_t i=0; i< now_instances.size(); i++)
    {
        CShape & instance(now_instances[i]);

        // search if one of my robot kits/vessel, if not continue
        if(std::find(rcs_world.part_list.begin(), rcs_world.part_list.end(), instance._name)==rcs_world.part_list.end())
            continue;

        if(!instance.isKit())
        {
            continue;
        }

        // kit - now do slots
        CShape * slotz = now_instances.getDefinition(instance.type());

        if(slotz==NULL)
            continue;

        // at this point instance is a kitting tray
        for(size_t j=0; j < slotz->_contains.size(); j++)
        {
            CShape & slot(slotz->_contains[j]);
            slotprop =  instance._properties[slot.name()];
            if(slotprop["state"]=="open")
            {
                kit=instances[i];
                return true;
            }
        }

    }
    return false;
}
////////////////////////////////////////////////////////////////////////////////
int CGearDemo::issueCommands(int & state, int & cmdnum)
{

    // Finish queuing commands before handling them....
    //std::unique_lock<std::mutex> lock(cncmutex);

    // Must declare all variables beforehand
    RCS::CCanonCmd cmd;

    if( WorldModel::instances.size()==0)
    {
        std::cerr << "Error: No gear instances can be read from gazebo_ro_api topic - restart Gazebo!\n";
        return -1;
    }


    WorldModel::CInstances now_instances;
    {
        std::unique_lock<std::mutex> lock(WorldModel::shapemutex);
        now_instances=instances;
    }

    switch(state)
    {
    case 0:
    {
        std::cout << "**************** STATE 0 \n";
        // Step: Find an open kitting slot and a free gear

        // Search kits for empty slot. Identify type of gear
        // find corredponding gear size in supply gtray.
        ;
        if(!findOpenKittingGearSlot(now_instances, _kit, _openslotprop))
        {
            ROS_FATAL_ONCE ( "Error: No open slots in any kits fopenslotpropound!");
            return -1;
        }

        std::string geartype = _openslotprop["type"];
        // this gear is in this slot should be checked to exist every iteration
        // of state table
        if ((_gear = findFreeGear(now_instances , geartype)) == nullptr)
        {
            ROS_FATAL_ONCE ( "Error: No matching free gear in supply vessel to move");
            return -1;
        }
        if(bDebug)
            std::cout << "Free slot kit="<< _kit.name() << " slot="<< _openslotprop["name"]<<
                         " Gear=" << _gear->name()  << " at" << RCS::dumpPoseSimple(_gear->centroid()) << "\n";
        return state++;
    }
    case 1:
    {
        std::cout << "**************** STATE 1 \n";
        // step: approach gear
        gearname = _gear->_name;

        if(bDebug)
            std::cout <<  "Gear World Coord=" << RCS::dumpPoseSimple(_gear->centroid()) << "\n";

        affpose =  _gear->_location ;
        affpose = rcs_robot.removeBase( affpose );
        if(bDebug)
        {
            std::cout <<  "Robot base    Coord=" << RCS::dumpPoseSimple(rcs_robot.basePose) << "\n";
            std::cout <<  "Robot baseinv Coord=" << RCS::dumpPoseSimple(rcs_robot.basePose.inverse()) << "\n";
            std::cout <<  "Gear Robot Coord=" << RCS::dumpPoseSimple(affpose) << "\n";
        }

        // The object gripper offset is where on the object it is to be gripped
        // e.g., offset.gripper->largegear = 0.0,0.0, -0.030, 0.0, 0.0.,0.0
        gripperoffset = rcs_robot.gripperoffset[_gear->_type];

        bend=rcs_robot.QBend;

        // The gripperoffset is the robot gripper offset back to the 0T6 equivalent
        //pickpose =  tf::Pose(bend, affpose.getOrigin()) * gripperoffset.inverse() ;
        pickpose = rcs_robot.addOffset( tf::Pose(bend, affpose.getOrigin()) , gripperoffset.getOrigin()) ;

        offset = pickpose.getOrigin();
        retractPose = rcs_robot.addRetract( tf::Pose(bend, offset));
        if(bDebug)
        {
            std::cout <<  "gripperoffset pose =" << RCS::dumpPoseSimple(gripperoffset) << "\n";
            std::cout <<  "Robot Pick pose =" << RCS::dumpPoseSimple(affpose) << "\n";
            std::cout <<  "Robot Pick offset pose =" << RCS::dumpPoseSimple(pickpose) << "\n";
            std::cout <<  "Robot Bend Moveto=" << RCS::dumpPoseSimple(tf::Pose(bend, offset)) << "\n";
            std::cout <<  "Robot Retract Moveto=" << RCS::dumpPoseSimple(retractPose) << "\n";

        }
        // Retract
        cmdnum = _crclapiPtr->moveTo(retractPose);
        cmdnum =_crclapiPtr->doDwell(_crclapiPtr->motionDwell());
        return state++;
    }
    case 2:
    {
        std::cout << "**************** STATE 2 \n";
        // move to grasping posiiton of gear
        cmdnum=_crclapiPtr->moveTo(tf::Pose(bend, offset) );
        cmdnum=_crclapiPtr->doDwell(_crclapiPtr->motionDwell());
        return state++;
    }
    case 3:
    {
        std::cout << "**************** STATE 3 \n";
        // step: grasp gear
        cmdnum=_crclapiPtr->closeGripper();
        cmdnum=_crclapiPtr->doDwell(_crclapiPtr->graspDwell());
        return state++;
    }
    case 4:
    {
        std::cout << "**************** STATE 4 \n";
        // step: retract robot after grasping gear

        cmdnum=_crclapiPtr->moveTo(retractPose);
        cmdnum=_crclapiPtr->doDwell(_crclapiPtr->motionDwell());
        return state++;
    }

    case 5:
    {
        std::cout << "**************** STATE 5 \n";
        // step: move to approach kitting open slot
        // Use existing found open kit slot
        // Offset from the centroid of the kit and reorientation
        // should be part of inferences
        //now_instances.findSlot(this->_kit, this->_openslot->name());

        // This xyz already has been reoriented by tray rotation.
        CShape * openslot=instances.findSlot(&_kit, _openslotprop["name"]);

        // compute reorient based on kit rotation
        tf::Pose slotloc = openslot->_location; // offset of locatino in tray
#if 1
        tf::Matrix3x3 m(_kit._location.getRotation());
        tf::Vector3 vec_slot = _kit._location.getOrigin() +  (m*slotloc.getOrigin());
        slotpose=tf::Pose(tf::QIdentity(), vec_slot);
#endif
        // adjust model z from world coordinate based to robot coordinate frame
        slotpose = rcs_robot.removeBase( slotpose);

        // up in z onloy for now - hard coded fudge factor
        slotoffset = rcs_world.slotoffset["vessel"];
        //placepose = tf::Pose(bend, slotpose.getOrigin())* slotoffset; // fixme: what if gear rotated
        placepose=rcs_robot.addOffset(tf::Pose(bend, slotpose.getOrigin()),slotoffset.getOrigin());

        // offset is the xyz position of slot
        offset = placepose.getOrigin(); // xyz position

        retractPose = rcs_robot.addRetract( tf::Pose(bend, offset));
        cmdnum=_crclapiPtr->moveTo(retractPose);
        cmdnum=_crclapiPtr->doDwell(_crclapiPtr->motionDwell());
        return state++;
    }
    case 6:
    {
        std::cout << "**************** STATE 6 \n";
        // Place the grasped object
        cmdnum=_crclapiPtr->moveTo(tf::Pose(bend, offset));
        cmdnum=_crclapiPtr->doDwell(_crclapiPtr->motionDwell());
        return state++;
    }
    case 7:
    {
        std::cout << "**************** STATE 7 \n";
        // open gripper and wait
        cmdnum=_crclapiPtr->openGripper();
        cmdnum=_crclapiPtr->doDwell(_crclapiPtr->graspDwell());
        return state++;
    }
    case 8:
    {
        // Retract from placed object
        //r->Retract(0.04);
        cmdnum=_crclapiPtr->moveTo(retractPose);
        cmdnum=_crclapiPtr->doDwell(_crclapiPtr->motionDwell());
        //        while(this->isWorking(cmdnum))
        //            Globals.sleep(100);
        return state++;
    }
    }
    state=0;
    return state;

}
int CGearDemo::isDone(int & state)
{
    return state >=9;
}

int CGearDemo::isWorking(int cmdnum)
{
    std::lock_guard<std::mutex> guard(crcl::mutex);
    if(rcs_robot._status.crclcommandnum>cmdnum)
        return false;
    bool bFlag = ((rcs_robot._status.crclcommandnum==cmdnum )
                  && boost::iequals(rcs_robot.s_crclcommandstatus , "CRCL_Done"));
    return !bFlag;
}

int CGearDemo::kittingMotionTest()
{
    // FIXME: test to see if threaded so crclapi calls
    // are serviced by background thread

    if(this->_crclapiPtr.get() == nullptr)
    {
        ROS_ERROR_STREAM("****null kittingtest crclapiPtr");
        return -1;
    }
    // Pass in the high level CRCL API test demo
    // Tests:
    // CRCL-> Canon Cmd-> ROS Q -> ROS cmd -> crclimpl
    return kittingtest(this->_crclapiPtr);
}

int CGearDemo::kittingModelTest()
{

    /// // Works: rosparam configuration of world, robot, crcl2ros works
    /// crcl2ros reading of gazebo model info works.
    /// crcl2ros performing model inferencing
    /// output should show all the gears/trays and open/full slots + gear location
    /// Note, gears are assume to be in a tray either kit or supply vessel.

    /// WM configuration - needs ROS nh and ns
    rcs_world.configure(CRos::nh.get(), rcs_world.robot);
    rcs_robot.printConfiguration();

    /// This sets up ROS to connect to gazebo and if enabled ros crcl topics
    rosCrclClient=  std::shared_ptr<CRosCrclClient>(new CRosCrclClient());

    /// Initializes ROS communication to topic using robot prefix...
    rosCrclClient->setup(rcs_world.robot);

    rosCmdStatMsgHandler=std::shared_ptr<crcl::CRosCmdStatusHandler> (new crcl::CRosCmdStatusHandler());
    rosCmdStatMsgHandler->setCmdQueue(NULL);
    rosCmdStatMsgHandler->setup(); // Setup subelements of crcl2ros - need joint names

    rosCmdMsgHandler= std::shared_ptr<crcl::CRosCmdMsgHandler>(new crcl::CRosCmdMsgHandler(rosCmdStatMsgHandler.get()));

    /// Start
    rosCrclClient->start();

    // Verify that Gazebo model and inferencing works.
    while(1)  {
        Globals.sleep(1000);
    }

    /** OUTPUT SHOULD LOOK LIKE:
     * Assuming kitting.yaml has model  debug set to 1:
     * model:
          parts: [sku_kit_m2l1_vessel14,sku_kit_m2l1_vessel15,sku_medium_gear_vessel16,sku_part_medium_gear17,sku_part_medium_gear18,sku_part_medium_gear19,sku_part_medium_gear20,sku_large_gear_vessel21,sku_part_large_gear22,sku_part_large_gear23]
          debug: 1

    [DEBUG] [1611067723.400670921, 69.342000000]: sku_kit_m2l1_vessel14 at    0.40,  -1.05,   0.92,0.000,0.000,-0.720,0.694
        slot1 sku_part_medium_gear open ( 0.45559 -1.00712  0.91599)
        slot2 sku_part_medium_gear open ( 0.45263 -1.08706  0.91599)
        slot3 sku_part_large_gear open ( 0.35917 -1.04358  0.91599)
    sku_kit_m2l1_vessel15 at    0.18,  -1.05,   0.92,0.000,0.000,-0.720,0.694
        slot1 sku_part_medium_gear open ( 0.23969 -1.01508  0.91599)
        slot2 sku_part_medium_gear open ( 0.23674 -1.09502  0.91599)
        slot3 sku_part_large_gear open ( 0.14328 -1.05154  0.91599)
    sku_medium_gear_vessel16 at    0.19,  -1.24,   0.92,0.000,0.000,0.017,1.000
        slot1 sku_part_medium_gear sku_part_medium_gear17 ( 0.22842 -1.19713  0.92317)
        slot2 sku_part_medium_gear sku_part_medium_gear18 ( 0.14927 -1.19981  0.92317)
        slot3 sku_part_medium_gear sku_part_medium_gear20 ( 0.23110 -1.27629  0.92317)
        slot4 sku_part_medium_gear sku_part_medium_gear19 ( 0.15195 -1.27897  0.92317)
    sku_part_medium_gear17 at    0.23,  -1.20,   0.92,-0.011,-0.003,0.022,1.000
        In: sku_medium_gear_vessel16(slot1)
    sku_part_medium_gear18 at    0.15,  -1.20,   0.92,-0.001,-0.001,0.017,1.000
        In: sku_medium_gear_vessel16(slot2)
    sku_part_medium_gear19 at    0.15,  -1.28,   0.92,0.001,-0.003,0.018,1.000
        In: sku_medium_gear_vessel16(slot4)
    sku_part_medium_gear20 at    0.23,  -1.28,   0.92,-0.002,0.001,0.012,1.000
        In: sku_medium_gear_vessel16(slot3)
    sku_large_gear_vessel21 at    0.39,  -1.26,   0.92,0.000,0.000,0.721,0.693
        slot1 sku_part_large_gear sku_part_large_gear23 ( 0.39442 -1.31785  0.91521)
        slot2 sku_part_large_gear sku_part_large_gear22 ( 0.38994 -1.20795  0.91521)
    sku_part_large_gear22 at    0.39,  -1.21,   0.92,0.002,-0.002,0.016,1.000
        In: sku_large_gear_vessel21(slot2)
    sku_part_large_gear23 at    0.39,  -1.32,   0.92,-0.002,-0.001,0.188,0.982
        In: sku_large_gear_vessel21(slot1)
    */
}

int CGearDemo::rawTest()
{

    /// THis is the setup for the raw test - no threading.
    /// Just configure and setup robot raw implementation
    /// WM configuration - needs ROS nh and ns
    rcs_world.configure(CRos::nh.get(), rcs_world.robot);
    rcs_robot.printConfiguration();

    /// Robot configuration - needs robot prefix, ROS nh and ns
    rcs_robot.configure(rcs_world.robot, CRos::nh.get(), rcs_world.robot);
    rcs_robot.printConfiguration();

    crclRobotImpl=std::shared_ptr<crcl::CRobotImpl>(new crcl::CRobotImpl());

    // FIXME: this is hard coded...
    crclRobotImpl->nodeHandle(CRos::nh.get())
            .urdfRobotDescriptionRosParam(Globals.ns+"/robot_description")
            .moveGroupName("fanucarm")
            .gzGripperTopic("/fanuc_lrmate200id/control")
            .jointStatePublisherTopic("/crcl/joint_states")
            .gzEnabled(1)
            .gzRobotModelName("lrmate")
            .init()
            .assertConfigurationValid();

    /// This verifies that the basic kinematic/traj gen works
    /// It also verifies that command to gazebo joint and gripper work
    /// It also verifies communication to joint_state_publisher works
    /// This is verifies that all the configuration parameters
    /// are passed in correclty through ROS param.
    ///
    /// NOTE no threading nor verification of model status

    Globals.bRobotInWorldCoordinates=false;
    return rawimpltest(crclRobotImpl);
}

/**
 * @brief rawimpltest
 * @param robot reference to CRCL implementation (traj gen, kin, gazebo).
 * Do not use CrclAPI to start a high level CRCL implemenation, rather
 * cicrumvents this cascading of message translation, and goes directly to
 * crcl impl message API
 */
int rawimpltest(std::shared_ptr<crcl::CRobotImpl>  robot) {
    try {
        moveit_msgs::RobotTrajectory traj;
        // Reset robot to "home"
        std::vector<double> zero(6,0.0);
        traj=robot->moveJoints(zero);
        robot->move(traj);

        tf::Pose p = robot->FK(zero);

        ROS_DEBUG_STREAM("****" << RCS::dumpStdVector(robot->IK(p)));

        std::vector<double> jnts={-0.04,-0.02,-0.28,-0.26,-0.14, 0.27};
        p = robot->FK(jnts);

        ROS_DEBUG_STREAM("****" << RCS::dumpStdVector(robot->IK(p)));
        ROS_DEBUG_STREAM("****" << "Expect Joints -0.04,-0.02,-0.28,-0.26,-0.14, 0.27");


        robot->eeMoveTo(tf::Pose(rcs_robot.QBend,
                                 tf::Vector3(0.40, -0.06, 0.05)
                                 )
                        );
        robot->doDwell(1.1);
        robot->eeMoveTo(tf::Pose(
                            rcs_robot.QBend,
                            tf::Vector3(0.40, -0.06, 0.01)
                            )
                        );
        robot->doDwell(1.1);
        robot->setGripper(0.0);
        robot->doDwell(2.0);
        robot->eeMoveTo(tf::Pose(
                            rcs_robot.QBend,
                            tf::Vector3(0.40, -0.06, 0.05)));
        robot->doDwell(1.1);
        robot->eeMoveTo(tf::Pose(
                            rcs_robot.QBend,
                            tf::Vector3(0.41, 0.04, 0.06)));
        robot->doDwell(1.1);
        robot->eeMoveTo(tf::Pose(
                            rcs_robot.QBend,
                            tf::Vector3(0.41, 0.04, 0.02)));
        robot->doDwell(1.1);
        robot->setGripper(1.);
        robot->eeMoveTo(tf::Pose(
                            rcs_robot.QBend,
                            tf::Vector3(0.41, 0.04, 0.06)));
        robot->doDwell(1.1);
        robot->eeMoveTo(tf::Pose(
                            rcs_robot.QBend,
                            tf::Vector3(0.32, -0.06, 0.05)));
        robot->doDwell(1.1);
        robot->eeMoveTo(tf::Pose(
                            rcs_robot.QBend,
                            tf::Vector3(0.32, -0.06, 0.01)));
        robot->doDwell(1.1);
        robot->setGripper(0.);
        robot->doDwell(2.);

        robot->eeMoveTo(tf::Pose(
                            rcs_robot.QBend,
                            tf::Vector3(0.41, 0.12, 0.06)));
        robot->doDwell(1.1);
        robot->eeMoveTo(tf::Pose(
                            rcs_robot.QBend,
                            tf::Vector3(0.41, 0.12, 0.02)));
        robot->doDwell(1.1);
        robot->setGripper(1.);
        robot->doDwell(2.);
        robot->eeMoveTo(tf::Pose(
                            rcs_robot.QBend,
                            tf::Vector3(0.41, 0.12, 0.06)));
        robot->doDwell(1.1);
        robot->eeMoveTo(tf::Pose(
                            rcs_robot.QBend,
                            tf::Vector3(0.32, -0.14, 0.05)));
        robot->doDwell(1.1);
        robot->eeMoveTo(tf::Pose(
                            rcs_robot.QBend,
                            tf::Vector3(0.32, -0.14, 0.01)));
        robot->doDwell(1.1);
        robot->setGripper(0.);
        robot->doDwell(2.);
        robot->eeMoveTo(tf::Pose(
                            rcs_robot.QBend,
                            tf::Vector3(0.32, -0.14, 0.05)));
        robot->doDwell(1.1);
        robot->eeMoveTo(tf::Pose(
                            rcs_robot.QBend,
                            tf::Vector3(0.62, 0.05, 0.06)));
        robot->doDwell(1.1);
        robot->eeMoveTo(tf::Pose(
                            rcs_robot.QBend,
                            tf::Vector3(0.62, 0.05, 0.02)));
        robot->doDwell(1.1);
        robot->setGripper(1.);
        robot->doDwell(2.);
        robot->eeMoveTo(tf::Pose(
                            rcs_robot.QBend,
                            tf::Vector3(0.62, 0.05, 0.06)));
        robot->doDwell(1.1);
        robot->moveTo(tf::Pose(
                          rcs_robot.QBend,
                          tf::Vector3(0.40, -0.14, 0.05)));
        robot->doDwell(1.1);
        robot->moveTo(tf::Pose(
                          rcs_robot.QBend,
                          tf::Vector3(0.40, -0.14, 0.01)));
        robot->doDwell(1.1);
        robot->setGripper(0.);
        robot->doDwell(2.);
        robot->eeMoveTo(tf::Pose(
                            rcs_robot.QBend,
                            tf::Vector3(0.40, -0.14, 0.05)));
        robot->doDwell(1.1);
        robot->eeMoveTo(tf::Pose(
                            rcs_robot.QBend,
                            tf::Vector3(0.62, 0.13, 0.06)));
        robot->doDwell(1.1);
        robot->eeMoveTo(tf::Pose(
                            rcs_robot.QBend,
                            tf::Vector3(0.62, 0.13, 0.02)));
        robot->doDwell(1.1);
        robot->setGripper(1.);
        robot->doDwell(2.);
        robot->eeMoveTo(tf::Pose(
                            rcs_robot.QBend,
                            tf::Vector3(0.62, 0.13, 0.06)));
        robot->doDwell(1.1);
        robot->eeMoveTo(tf::Pose(
                            rcs_robot.QBend,
                            tf::Vector3(0.56, -0.07, 0.05)));
        robot->doDwell(1.1);
        robot->eeMoveTo(tf::Pose(
                            rcs_robot.QBend,
                            tf::Vector3(0.56, -0.07, 0.01)));
        robot->doDwell(1.1);
        robot->setGripper(0.);
        robot->doDwell(2.);
        robot->eeMoveTo(tf::Pose(
                            rcs_robot.QBend,
                            tf::Vector3(.56, -0.07, 0.05)));
        robot->doDwell(1.1);
        robot->moveTo(tf::Pose(
                          rcs_robot.QBend,
                          tf::Vector3(0.53, 0.10, 0.06)));
        robot->doDwell(1.1);
        robot->moveTo(tf::Pose(
                          rcs_robot.QBend,
                          tf::Vector3(0.53, 0.10, 0.02)));
        robot->doDwell(1.1);
        robot->setGripper(1.);
        robot->doDwell(2.);
        robot->eeMoveTo(tf::Pose(
                            rcs_robot.QBend,
                            tf::Vector3(0.53, 0.10, 0.06)));
        robot->doDwell(1.1);
        robot->eeMoveTo(tf::Pose(
                            rcs_robot.QBend,
                            tf::Vector3(0.56, -0.18, 0.05)));
        robot->doDwell(1.1);
        robot->eeMoveTo(tf::Pose(
                            rcs_robot.QBend,
                            tf::Vector3(0.56, -0.18, 0.01)));
        robot->doDwell(1.1);
        robot->setGripper(0.);
        robot->doDwell(2.);
        robot->eeMoveTo(tf::Pose(
                            rcs_robot.QBend,
                            tf::Vector3(0.56, -0.18, 0.05)));
        robot->doDwell(1.1);
        robot->eeMoveTo(tf::Pose(
                            rcs_robot.QBend,
                            tf::Vector3(0.31, 0.09, 0.06)));
        robot->doDwell(1.1);
        robot->eeMoveTo(tf::Pose(
                            rcs_robot.QBend,
                            tf::Vector3(0.31, 0.09, 0.02)));
        robot->doDwell(1.1);
        robot->setGripper(1.);
        robot->doDwell(2.);
        robot->eeMoveTo(tf::Pose(
                            rcs_robot.QBend,
                            tf::Vector3(0.31, 0.09, 0.06)));
        robot->doDwell(1.1);

    } catch (std::exception ex) {
        std::cerr <<  " fanuckitting test exception " << ex.what() <<"\n";
        return -1;
    }
    return 0;
}

/**
 * @brief kittingtest
 * @param crcl  crclapi reference truns crcl api calls into crcl messages to be translated
 * into ROS representations,
 * then passed into crcl impl which interprets the converted crcl2ros commands.
 */
int kittingtest(std::shared_ptr<crcl::CCrcl2RosMsgApi>  robot) {
    try {

        robot->moveTo(tf::Pose(rcs_robot.QBend,
                               tf::Vector3(0.40, -0.06, 0.05)
                               )
                      );
        robot->doDwell(1.1);
        robot->moveTo(tf::Pose(
                          rcs_robot.QBend,
                          tf::Vector3(0.40, -0.06, 0.01)
                          )
                      );
        robot->doDwell(1.1);
        robot->setGripper(0.0);
        robot->doDwell(2.0);
        robot->moveTo(tf::Pose(
                          rcs_robot.QBend,
                          tf::Vector3(0.40, -0.06, 0.05)));
        robot->doDwell(1.1);
        robot->moveTo(tf::Pose(
                          rcs_robot.QBend,
                          tf::Vector3(0.41, 0.04, 0.06)));
        robot->doDwell(1.1);
        robot->moveTo(tf::Pose(
                          rcs_robot.QBend,
                          tf::Vector3(0.41, 0.04, 0.02)));
        robot->doDwell(1.1);
        robot->setGripper(1.);
        robot->moveTo(tf::Pose(
                          rcs_robot.QBend,
                          tf::Vector3(0.41, 0.04, 0.06)));
        robot->doDwell(1.1);
        robot->moveTo(tf::Pose(
                          rcs_robot.QBend,
                          tf::Vector3(0.32, -0.06, 0.05)));
        robot->doDwell(1.1);
        robot->moveTo(tf::Pose(
                          rcs_robot.QBend,
                          tf::Vector3(0.32, -0.06, 0.01)));
        robot->doDwell(1.1);
        robot->setGripper(0.);
        robot->doDwell(2.);

        robot->moveTo(tf::Pose(
                          rcs_robot.QBend,
                          tf::Vector3(0.41, 0.12, 0.06)));
        robot->doDwell(1.1);
        robot->moveTo(tf::Pose(
                          rcs_robot.QBend,
                          tf::Vector3(0.41, 0.12, 0.02)));
        robot->doDwell(1.1);
        robot->setGripper(1.);
        robot->doDwell(2.);
        robot->moveTo(tf::Pose(
                          rcs_robot.QBend,
                          tf::Vector3(0.41, 0.12, 0.06)));
        robot->doDwell(1.1);
        robot->moveTo(tf::Pose(
                          rcs_robot.QBend,
                          tf::Vector3(0.32, -0.14, 0.05)));
        robot->doDwell(1.1);
        robot->moveTo(tf::Pose(
                          rcs_robot.QBend,
                          tf::Vector3(0.32, -0.14, 0.01)));
        robot->doDwell(1.1);
        robot->setGripper(0.);
        robot->doDwell(2.);
        robot->moveTo(tf::Pose(
                          rcs_robot.QBend,
                          tf::Vector3(0.32, -0.14, 0.05)));
        robot->doDwell(1.1);
        robot->moveTo(tf::Pose(
                          rcs_robot.QBend,
                          tf::Vector3(0.62, 0.05, 0.06)));
        robot->doDwell(1.1);
        robot->moveTo(tf::Pose(
                          rcs_robot.QBend,
                          tf::Vector3(0.62, 0.05, 0.02)));
        robot->doDwell(1.1);
        robot->setGripper(1.);
        robot->doDwell(2.);
        robot->moveTo(tf::Pose(
                          rcs_robot.QBend,
                          tf::Vector3(0.62, 0.05, 0.06)));
        robot->doDwell(1.1);
        robot->moveTo(tf::Pose(
                          rcs_robot.QBend,
                          tf::Vector3(0.40, -0.14, 0.05)));
        robot->doDwell(1.1);
        robot->moveTo(tf::Pose(
                          rcs_robot.QBend,
                          tf::Vector3(0.40, -0.14, 0.01)));
        robot->doDwell(1.1);
        robot->setGripper(0.);
        robot->doDwell(2.);
        robot->moveTo(tf::Pose(
                          rcs_robot.QBend,
                          tf::Vector3(0.40, -0.14, 0.05)));
        robot->doDwell(1.1);
        robot->moveTo(tf::Pose(
                          rcs_robot.QBend,
                          tf::Vector3(0.62, 0.13, 0.06)));
        robot->doDwell(1.1);
        robot->moveTo(tf::Pose(
                          rcs_robot.QBend,
                          tf::Vector3(0.62, 0.13, 0.02)));
        robot->doDwell(1.1);
        robot->setGripper(1.);
        robot->doDwell(2.);
        robot->moveTo(tf::Pose(
                          rcs_robot.QBend,
                          tf::Vector3(0.62, 0.13, 0.06)));
        robot->doDwell(1.1);
        robot->moveTo(tf::Pose(
                          rcs_robot.QBend,
                          tf::Vector3(0.56, -0.07, 0.05)));
        robot->doDwell(1.1);
        robot->moveTo(tf::Pose(
                          rcs_robot.QBend,
                          tf::Vector3(0.56, -0.07, 0.01)));
        robot->doDwell(1.1);
        robot->setGripper(0.);
        robot->doDwell(2.);
        robot->moveTo(tf::Pose(
                          rcs_robot.QBend,
                          tf::Vector3(.56, -0.07, 0.05)));
        robot->doDwell(1.1);
        robot->moveTo(tf::Pose(
                          rcs_robot.QBend,
                          tf::Vector3(0.53, 0.10, 0.06)));
        robot->doDwell(1.1);
        robot->moveTo(tf::Pose(
                          rcs_robot.QBend,
                          tf::Vector3(0.53, 0.10, 0.02)));
        robot->doDwell(1.1);
        robot->setGripper(1.);
        robot->doDwell(2.);
        robot->moveTo(tf::Pose(
                          rcs_robot.QBend,
                          tf::Vector3(0.53, 0.10, 0.06)));
        robot->doDwell(1.1);
        robot->moveTo(tf::Pose(
                          rcs_robot.QBend,
                          tf::Vector3(0.56, -0.18, 0.05)));
        robot->doDwell(1.1);
        robot->moveTo(tf::Pose(
                          rcs_robot.QBend,
                          tf::Vector3(0.56, -0.18, 0.01)));
        robot->doDwell(1.1);
        robot->setGripper(0.);
        robot->doDwell(2.);
        robot->moveTo(tf::Pose(
                          rcs_robot.QBend,
                          tf::Vector3(0.56, -0.18, 0.05)));
        robot->doDwell(1.1);
        robot->moveTo(tf::Pose(
                          rcs_robot.QBend,
                          tf::Vector3(0.31, 0.09, 0.06)));
        robot->doDwell(1.1);
        robot->moveTo(tf::Pose(
                          rcs_robot.QBend,
                          tf::Vector3(0.31, 0.09, 0.02)));
        robot->doDwell(1.1);
        robot->setGripper(1.);
        robot->doDwell(2.);
        robot->moveTo(tf::Pose(
                          rcs_robot.QBend,
                          tf::Vector3(0.31, 0.09, 0.06)));
        robot->doDwell(1.1);

    } catch (std::exception ex) {
        std::cerr <<  " fanuckitting test exception " << ex.what() <<"\n";
        return -1;
    }
    return 0;
}
int CGearDemo::demoKittingTest(bool bSingleThread, bool bCLI)
{
    // Setup gnu readline command line interface
    RCS::CComandLineInterface cli;

    //#define CLI
    if(bCLI)
    {
        cli.start();
    }
    else
    {
        CGlobals::bPaused=false;
        Globals.bCannedDemo()=true;

    }
    // initialize the demostate state table
    int demostate=0;
    int cmdnum=-1;

    do {

        if(isDone(demostate))
        {
            demostate=0;
            //break;
        }

        // This will hang on auto as it now changes the state back to noop after every empty fetch.
        // Keep last state in cli?
        int clistate;
        if(bCLI)
            clistate= cli.inputState();
        else
            clistate=-1;

        if(clistate==rcs_state::EXITING)
        {
            CGlobals::bRunning=false;
            break;
        }
        //            if(clistate==rcs_state::STEP)
        //            {
        //                // cycle through one pick/place cycle.
        //                bool bFail;
        //                int nStateMax=0;
        //                // issue commands until state goes back to zero
        //                do {
        //                    bFail=geardemo->issueCommands(demostate);
        //                    nStateMax=std::max(demostate,nStateMax );
        //                    Globals.sleep(100);
        //                }
        //                while( !geardemo->isDone(demostate)  && bFail >= 0 );
        //
        //                // Now wait till all commands done
        //                std::cout << "STEP" << std::flush;
        //                bool bflag;
        //                do
        //                {
        //                    std::cout << "." << std::flush;
        //                    sleep(1)   ;
        //                    bflag=geardemo->isWorking();
        //                } while (! bflag );
        //                std::cout << "DONE\n";
        //            }
        if(clistate==rcs_state::PAUSED)
            CGlobals::bPaused=true;

        if(clistate==rcs_state::TEST)
        {
            kittingtest(_crclapiPtr);
        }
        if(clistate==rcs_state::NORMAL)
            CGlobals::bPaused=false;

        if(clistate==rcs_state::ONCE)
        {
            CGlobals::bPaused=false;
            Globals.bCannedDemo()=true;
        }
        if(clistate==rcs_state::AUTO)
        {
            CGlobals::bPaused=false;
            Globals.bCannedDemo()=true;
        }
        if(clistate==rcs_state::REPEAT)
        {
            CGlobals::bPaused=false;
            Globals.bCannedDemo()=true;
            Globals.bRepeatCannedDemo=true;
        }

        // If canned demo AND finished last commands
        if(!CGlobals::bPaused && Globals.bCannedDemo() && ! rcs_robot.isBusy(cmdnum))
        {
            if(issueCommands(demostate,cmdnum)<0)
                //  if(geardemo->issueRobotCommands(demostate)<0)
            {
                if(Globals.bRepeatCannedDemo)
                {
                    // FIXME: add again
                    //rosCrclClient->reset();
                    ::sleep(2.0);
                }
                else
                    Globals.bCannedDemo()=false;
            }
        }

        if(clistate==rcs_state::ONCE)
        {
            CGlobals::bPaused=true;
        }
        if(bSingleThread)
            RCS::Thread::cycleAll();
        else
            Globals.thread_sleep(100);



    } while (Globals.ok());
    return 0;
}
