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

#ifndef CROS_H
#define CROS_H


#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <ros/master.h>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#ifdef INCROSMSG
#include <crcl_rosmsgs/CrclCommandMsg.h>
#include <crcl_rosmsgs/CrclMaxProfileMsg.h>
#include <crcl_rosmsgs/CrclStatusMsg.h>
#endif
#include "rcs/RCSThreadTemplate.h"
#include "rcs/IRcs.h"

#include "crclapp/Globals.h"

#ifdef GAZEBO
#include <gazebo_msgs/ModelStates.h>
#endif

namespace crcl {
class CRosCmdStatusHandler;
}
/**
 * @brief The CRosCrclClient class handles. At this point no ros cmd/status topics
 * are used to communiate with ROS. INstead model information from gazebo is used.
 * Possible in the future that real robot control would communicate through
 * ROS cmd/status controller.
 * topic subscription for CRCL status and advertising for
 * CRCL commands.
 */
class CRosCrclClient : public RCS::Thread
{
public:

    ///////////////////////////////////////////////////////////
    /**
     * @brief CRosCrclClient constructor.
     */
    CRosCrclClient();
    //boost::shared_ptr<crcl::CServer> crcl2ros;  // decode ROS status into CRCL status
    RCS::CrclMessageQueue crcltopiccmds; /**< queue of commands interpreted from Crcl messages */

#ifdef SERVER
    /**
     * @brief CmdCallback when a CRCL message arrives it uses
     * this method as a callback
     * to post the message to the CRCL message queue for the controller assigned to this class.
     * @param cmdmsg
     */
    void cmdCallback(const crcl_rosmsgs::CrclCommandMsg::ConstPtr& cmdmsg);
    /**
     * @brief PublishCrclStatus
     * @param statusmsg
     */
    void publishCrclStatus(crcl_rosmsgs::CrclStatusMsg &statusmsg);

#endif
#ifdef ROSMSG
    /**
     * @brief statusCallback when a CRCL status message arrives this method
     * is the callback notification
     * @param statusmsg currently crcl specific status
     */
    //void statusCallback(const crcl_rosmsgs::CrclStatusMsg::ConstPtr& statusmsg);
    void statusCallback(crcl_rosmsgs::CrclStatusMsg* statusmsg);

    /**
     * @brief PublishCrclStatus
     * @param statusmsg
     */
    void publishCrclCommand(crcl_rosmsgs::CrclCommandMsg &cmdmsg);

#endif
#ifdef GAZEBO
    void gzModelStatesCallback(const gazebo_msgs::ModelStates &gzstate_current);
    ros::Subscriber _gzWorldModel;
    ros::Publisher _crclWorldModel;
    std::mutex _mymutex;
    void reset();

#endif
    /**
     * @brief setup establish which controller is connected to the ROS CRCL handler.
     * @param prefix prefix name of the robot
     */
    void setup(std::string prefix);

    /**
     * @brief init subscribes to command  topic messages and advertises status topic messages.
     */
    virtual void init();

    /**
     * @brief Stop stops subscriptioin to command  topic messages and advertising status topic messages.
     */
    virtual void stop();

    virtual int action();
    /**
     * @brief _crclStatus susbcribe to status from robot controller
     */
    ros::Subscriber  _crclStatus; /**< ros publisher information used for crcl status updates */
    ros::Publisher _crclCmd; /**< ros subscriber information used for crcl command updates */
    std::string _prefix;

};

/**
 * @brief The CRos class handles establishing a connection to ROS. It creates a ROS node that
 * is reusable by all. It is important to establish this ROS connection to the master as all of the ROS
 * routines and data structures rely on rosout to log messages. If rosout doesn't exist, problems.
 */
class CRos
{
public:
    /**
     * @brief CRos handles establishing a connection to ROS
     */
    CRos();

    /**
     * @brief Init initializes a ROS session application.
     *  Waits for ROS master to be running -  look for rosout, because rosmaster is python program.
     * Initialize ROS environment ROS_MASTER_URI MUST BE SET or seg fault.
     * Creates universally usable in this application NodeHandlePtr.
     * Creates a ROS asynchronous spinner required for multithreaded ROS communication.
     */
    void init();

    /**
     * @brief Close call ros::shutdown()
     */
    void close();

    void setupLogger(std::string logger_level);

    static  ros::NodeHandlePtr nh; /**< node handle for ros app calls */

    static bool isRunning()
    {
        return ros::master::check();
    }

    void setRosLogPath(std::string exepath)
    {
        logpath=exepath.substr(0, exepath.find_last_of('/') + 1);
        logpath+="exec.qtmovemove.log";
     }
    void parseRosLogPath(std::string exepath)
    {
        app=exepath.substr(exepath.find_last_of('/') + 1);
        app=Globals.replaceAll(app, ".exe","");

        logpath=exepath.substr(0, exepath.find_last_of('/') + 1);
        logpath+="../../../logs/";
        boost::filesystem::create_directory(logpath);
        logpath+=app+"/";
        boost::filesystem::create_directory(logpath);
        logpath+="exec.movemove.log";
        std::ofstream output(logpath);
        ROS_ERROR("LOG %s",logpath.c_str());
    }
    void saveRosParam(ros::NodeHandle& node_handle)
    {
        std::ofstream output(logpath);
        std::vector<std::string> keys;
        std::string rd;

#if 1
        node_handle.getParamNames(keys);
        std::sort(keys.begin(), keys.end());
       //           [] (myclass const& a, myclass const& b) { return a.v < b.v; });
        for(size_t i=0; i< keys.size(); i++)
        {
            XmlRpc::XmlRpcValue param;
            node_handle.getParam(keys[i], param);
            XmlRpc::XmlRpcValue::Type t = param.getType();
            std::string field;
            std::stringstream ss;
            switch(t)
            {
            case  XmlRpc::XmlRpcValue::TypeBoolean:
            case  XmlRpc::XmlRpcValue::TypeInt:
            case  XmlRpc::XmlRpcValue::TypeDouble:
            case  XmlRpc::XmlRpcValue::TypeString:
                param.write(ss);
                field=ss.str();
                break;
            case XmlRpc::XmlRpcValue::TypeStruct:
                std::cout << " XmlRpc::XmlRpcValue::TypeStruct="<< keys[i] << "\n";
                break;
            case XmlRpc::XmlRpcValue::TypeArray:
                std::cout << " XmlRpc::XmlRpcValue::TypeArray="<< keys[i] << "\n";
                //ss << "[";
                for(int j = 0; j < param.size(); j++)
                {
                    param[j].write(ss);
                    ss << ",";
                 }
                //ss << "<";
                field=ss.str();
                break;
            case XmlRpc::XmlRpcValue::TypeDateTime:
                std::cout << " XmlRpc::XmlRpcValue::TypeDateTime="<< keys[i] << "\n";
                break;
            case XmlRpc::XmlRpcValue::TypeBase64:
                std::cout << " XmlRpc::XmlRpcValue::TypeBase64="<< keys[i] << "\n";
                break;
            default:
                break;
            }

            //node_handle.getParam(keys[i], rd);
            output << keys[i] << "=" << field << "\n";
        }
        output.flush();
        output.close();
#endif
    }

protected:
    static ros::AsyncSpinner * _spinner; /**< aynchronous spinning for ROS communication - not sure required */
    std::string logpath ;
    std::string app;
};
extern CRos Ros;
#endif // CROS_H
