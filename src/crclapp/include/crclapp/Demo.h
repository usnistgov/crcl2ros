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

#ifndef DEMO_H
#define DEMO_H

#include "rcs/Core.h"
#include "rcs/IRcs.h"

#include "crclapp/Shape.h"
#include "crclapp/Crcl2RosMsgApi.h"


// Gear Demo 
struct CGearDemo
{
    CGearDemo(std::shared_ptr<crcl::CCrcl2RosMsgApi>  crclApi);
    int init(ros::NodeHandle *nh, std::string ns);
    //int issueRobotCommands(int & state);
    int issueCommands(int & state, int & cmdnum);
    int isDone(int & state );

    WorldModel::CShape * findFreeGear(WorldModel::CInstances &now_instances, std::string geartype);

    bool findOpenKittingGearSlot(WorldModel::CInstances &now_instances,
                                 WorldModel::CShape & kit,
                                 std::map<std::string, std::string> &slotprop);

    std::shared_ptr<crcl::CCrcl2RosMsgApi>  _crclapiPtr;
    static  int bDebug;
    int isWorking(int cmdnum);
    int kittingMotionTest();
    int kittingModelTest();
    int rawTest();
    int demoKittingTest(bool bSingleThread, bool bCLI);


protected:
    WorldModel::CShape * _gear;
    WorldModel::CShape  _kit;
    std::map<std::string, std::string> _openslotprop;
    WorldModel::CShapes _shapes;
    std::string _path;
    tf::Pose _baseoffset;

    // script variables
    tf::Pose pickpose;
    std::string gearname;
    tf::Pose affpose;
    tf::Pose gripperoffset;
    tf::Quaternion bend;
    tf::Vector3 offset;
    tf::Pose slotpose;
    tf::Pose slotoffset;
    tf::Pose placepose;
    tf::Pose retractPose;
 };
extern int kittingtest(std::shared_ptr<crcl::CCrcl2RosMsgApi>  robot);
extern int rawimpltest(std::shared_ptr<crcl::CRobotImpl>  robot);
#endif
