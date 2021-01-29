#ifndef _ROSMSGHANDLER_H
#define _ROSMSGHANDLER_H

///  RosMsgHandler.h

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

// C++ includes
#include <list>

// boost includes
#include <boost/shared_ptr.hpp>
#include "boost/iostreams/stream.hpp"
#include "boost/iostreams/device/null.hpp"

// eigen math lcibrary includes
#ifdef EIGEN
#include "Eigen/Core"
#include "Eigen/Geometry"
#endif

#include <rcs/RCSThreadTemplate.h>
#include <rcs/RCSMsgQueueThread.h>
#include <rcs/IRcs.h>

#include "crclapp/CrclSocketServer.h"
#include "crclapp/CrclSubscriberInterface.h"
//#include "crclapp/CrclPublisherInterface.h"
#include "crclapp/CrclPrimitives.h"
#include "crclapp/CrclWm.h"
#include "crclapp/CrclRobotImpl.h"

namespace crcl {
class CRosCmdStatusHandler  : public RCS::Thread
{
public:
    crcl::CMessageQueue msgq;
    CRosCmdStatusHandler();
    virtual int action();
    virtual void setup();
    void setCmdQueue(RCS::CrclMessageQueue *crclmsgq)
    {
        // should all referencces to crclcmdsq be mutexed?
        std::lock_guard<std::mutex> guard(crcl::mutex);
        this->crclmsgq=crclmsgq;
    }
    /**
     * @brief StatusUpdate accepts rcs/ros status message and translates into CRCL status.
     * @param statusmsg ros message of status
     */
    void statusUpdate(crcl_rosmsgs::CrclStatusMsg & statusmsg);

    boost::shared_ptr<crcl::CrclSubscriberDelegateInterface> crclinterface;
    // interface to sending crcl2ros message to RCS
    static RCS::CrclMessageQueue *crclmsgq;
    static std::string length_units;
    static std::string angle_units;

};

/**
 * @brief The CCrcl2RosMsg class  handles the command/status interface to ROS.
 * For commands, CCrcl2RosMsg publishes an  CrclCommandMsg
 */
class CRosCmdMsgHandler  : public RCS::Thread
{

public:

    /**
     * @brief CCrcl2RosMsg handles translation from crcl representation into ros/tf representation
     */
    CRosCmdMsgHandler(crcl::CRosCmdStatusHandler * rosCmdStatMsgHandler);

    /**c
     * @brief createRosStatus use ROS command to create and load ROS status message
     * @param crclCommandStatus
     * @param crclcommandnum
     * @param currentpose
     * @param robotjoints
     * @param eepercent
     * @return
     */
    crcl_rosmsgs::CrclStatusMsg createRosStatus(
            long crclCommandStatus,
            long crclcommandnum,
            tf::Pose currentpose,
            sensor_msgs::JointState robotjoints,
            double eepercent
            );

    /**
    * @brief Cyclic loop for the controller. Reads Crcl input mexsage queue, interprets into canon cmds if any, reads canon
    * cmds queue, icnterprets into robot command messages.
    * @return  1 sucess,  0 problem
    */
    virtual int action();

    ////////////////////////////////////////////
    static long last_cmdnum;
    crcl::CRosCmdStatusHandler * _rosCmdStatMsgHandler;

};
}
#endif
