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

#include <algorithm>
#include <stdlib.h>
#include <termios.h>
#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>
#include <fstream>
#include <readline/readline.h>
#include <readline/history.h>

#include <boost/algorithm/string.hpp>

#include "crclapp/CommandLineInterface.h"
#include "crclapp/CrclWm.h"
#include "crclapp/RosMsgHandler.h"
#include "crclapp/Ros.h"
#include "crclapp/Demo.h"

#include "rcs/Conversions.h"
#include "rcs/Debug.h"
//#include "aprs_headers/env.h"
#include "rcs/Path.h"
#include "rcs/File.h"

// THanks to: http://cc.byexamples.com/2007/04/08/non-blocking-user-input-in-loop-without-ncurses/
#define NB_ENABLE 1
#define NB_DISABLE 2

extern std::shared_ptr<CGearDemo> geardemo;
extern std::shared_ptr<CRosCrclClient> rosCrclClient;

using namespace RCS;
using namespace crcl;

////////////////////////////////////////////////////////////////////////////////
static std::string shell(const char* cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
    if (!pipe)
    {
        if(!Globals.bIgnoreExceptions)
            throw std::runtime_error("popen() failed!");
        return "popen() failed!";
    }
    while (!feof(pipe.get())) {
        if (fgets(buffer.data(), 128, pipe.get()) != nullptr)
            result += buffer.data();
    }
    return result;
}

////////////////////////////////////////////////////////////////////////////////
int kbhit()
{
    struct timeval tv;
    fd_set fds;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds); //STDIN_FILENO is 0
    select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
    return FD_ISSET(STDIN_FILENO, &fds);
}

////////////////////////////////////////////////////////////////////////////////
void nonblock(int state)
{
    struct termios ttystate;

    //get the terminal state
    tcgetattr(STDIN_FILENO, &ttystate);

    if (state==NB_ENABLE)
    {
        //turn off canonical mode
        ttystate.c_lflag &= ~ICANON;
        //minimum of number input read.
        ttystate.c_cc[VMIN] = 1;
    }
    else if (state==NB_DISABLE)
    {
        //turn on canonical mode
        ttystate.c_lflag |= ICANON;
    }
    //set the terminal attributes.
    tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);

}
////////////////////////////////////////////////////////////////////////////////
std::vector<double> ConvertV(
        std::vector<std::string> stringVector) {
    std::vector<double> doubleVector;
    std::transform(stringVector.begin(), stringVector.end(), back_inserter(doubleVector),
                   [](std::string const& val) {
        return stod(val);
    });
    return doubleVector;
}

////////////////////////////////////////////////////////////////////////////////
CComandLineInterface::CComandLineInterface()
    : RCS::Thread(0.01, "CLI")
{
    _bDegrees = false;
    _bFlag = true;
    _ncindex=0;
}



////////////////////////////////////////////////////////////////////////////////
bool CComandLineInterface::jog(char c,double amt, int &jnt, int &axis)
{
    sensor_msgs::JointState curpos,nextpos;
    curpos.position = rcs_status.r_curjnts.position;
    double incr=0;
    static std::string jogaxis= "xyzijk";

    switch(c)
    {
    case '!':
    case '\n':
        return false;
    case '+':
        incr=amt;
        break;
    case '-':
        incr=-amt;
        break;
    case 'x':
    case 'y':
    case 'z':
    case 'i':
    case 'j':
    case 'k':
        axis =  jogaxis.find(c);
        jnt = -1;
        break;
    case '0':
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
        jnt = ((int)c - (int) '0');
        axis=-1;
    }

    if(incr==0)
        return true;
    if(jnt>=0)
    {
        nextpos=curpos;
        nextpos.position[jnt]+=incr;
        //crclApi->moveJoints(Globals.allJointNumbers(), nextpos.position);
        crclApi->moveJoints(Globals.allJointNumbers(rcs_robot.numJoints()), nextpos.position);
    }
    if(axis>=0)
    {
        nextpos=curpos;
        tf::Pose r_curpose=rcs_status.r_curpose;
        //Globals.robotKinematics()->FK(curpos.position, r_curpose);
        tf::Matrix3x3  m3x3, newm3x3;
        //tf::Vector3 v = r_curpose.getOrigin();
        if(axis==0)
            r_curpose.getOrigin().setX(r_curpose.getOrigin().x()+incr);
        if(axis==1)
            r_curpose.getOrigin().setY(r_curpose.getOrigin().y()+incr);
        if(axis==2)
            r_curpose.getOrigin().setZ(r_curpose.getOrigin().z()+incr);
        if(axis==3)
        {
            m3x3=r_curpose.getBasis();
            newm3x3.setRPY(incr,0.0,0.0);
            r_curpose.setBasis(m3x3*newm3x3);
        }
        if(axis==4)
        {
            m3x3=r_curpose.getBasis();
            newm3x3.setRPY(0.0,incr,0.0);
            r_curpose.setBasis(m3x3*newm3x3);
        }
        if(axis==5)
        {
            m3x3=r_curpose.getBasis();
            newm3x3.setRPY(0.0, 0.0, incr);
            r_curpose.setBasis(m3x3*newm3x3);
        }

        sensor_msgs::JointState joints;
        try{
            // seed joints
            joints.position=subset(rcs_status.r_curjnts.position,
                                   rcs_robot.numJoints());
            //Globals.robotKinematics()->IK(r_curpose, joints.position);
            crclApi->moveJoints(rcs_robot.allJointNumbers(), joints.position);
        }
        catch(...)
        {
            std::cout << "error jogging - move robot to initialize position since open loop\n" << std::flush;
        }
    }
    return true;
}

////////////////////////////////////////////////////////////////////////////////
void CComandLineInterface::loopCallback(char* line)
{

    //    Non-blocking polled read of std input
    //    struct pollfd fds;
    //    int ret;
    //    fds.fd = 0; /* this is STDIN */
    //    fds.events = POLLIN;
    //    ret = poll(&fds, 1, 0);
    //    if(ret == 0)
    //    {
    //        return CController::NOOP;
    //    }

#if 1
    if(!line)
    {
        lineq.addMsgQueue("quit");
    }
    else
    {
        add_history(line);
        lineq.addMsgQueue(line);
        free(line);
    }

#else
    std::string line;
    if(!std::getline(std::cin, line))
        return CController::EXITING;
    ret =  interpretLine(line);

    if(ret >=0)
        std::cout << "> " << std::flush;
    return ret;
#endif
}
namespace {
boost::function<void(char *)> callback;
extern "C" void wrapper(char * s) {
    callback(s);
}
}
////////////////////////////////////////////////////////////////////////////////
int CComandLineInterface::action()
{    
    if(kbhit()!=0)
        rl_callback_read_char();

    return 1;
}
////////////////////////////////////////////////////////////////////////////////
void CComandLineInterface::init ( )
{
    // http://www.mcld.co.uk/blog/2009/simple-gnu-readline-callback-style-example.html
    const char *prompt = "RCS> ";
    // Install the callback handler
    callback=boost::bind(&CComandLineInterface::loopCallback, this,_1);
    rl_callback_handler_install(prompt, (rl_vcpfunc_t*) wrapper);

}
#if 0
////////////////////////////////////////////////////////////////////////////////
int CComandLineInterface::inputLoop()
{
    // http://www.mcld.co.uk/blog/2009/simple-gnu-readline-callback-style-example.html
    const char *prompt = "RCS> ";
    // Install the handler
    //    rl_callback_handler_install(prompt, (rl_vcpfunc_t*) std::bind(&CComandLineInterface::loopCallback, this,_1));
    callback=boost::bind(&CComandLineInterface::loopCallback, this,_1);
    rl_callback_handler_install(prompt, (rl_vcpfunc_t*) wrapper);

    // Enter the event loop (simple example, so it doesn't do much except wait)
    bRunning = 1;
    while(bRunning){
        usleep(10000);
        rl_callback_read_char();
    };
    return 0;
}

#endif
////////////////////////////////////////////////////////////////////////////////
int CComandLineInterface::inputState()
{
    if(lineq.sizeMsgQueue()>0)
    {
        std::string line = lineq.popBackMsgQueue();
        state =  interpretLine(line);
        return state;
    }
    if(state==rcs_state::EXITING)
    {
        bRunning=false;
    }
    return rcs_state::NOOP;
}


////////////////////////////////////////////////////////////////////////////////
int CComandLineInterface::interpretMacro(std::string macro_name)
{
    std::map<std::string, std::vector<std::string>>::iterator it;
    if((it=rcs_robot.namedCommand.find(macro_name))!= rcs_robot.namedCommand.end())
        for(size_t k=0; k< (*it).second.size(); k++)
        {
            interpretLine((*it).second.at(k));
            // should we wait until done with each command?
        }
    else
        return -1;
    return 0;
}

////////////////////////////////////////////////////////////////////////////////

void thread_sleep(long milliseconds)
{
    std::chrono::microseconds ms (milliseconds);
    auto start = std::chrono::high_resolution_clock::now();
    auto end = start + ms;
    do {
        std::this_thread::yield();
    } while (std::chrono::high_resolution_clock::now() < end);
}

////////////////////////////////////////////////////////////////////////////////
int CComandLineInterface::interpretFile(std::string filename)
{
    std::string line;
    std::vector<std::string> paths= {Globals.appProperties["ExeDirectory"]};
    if(!Globals.appProperties["CurDirectory"].empty())
        paths.push_back(Globals.appProperties["CurDirectory"]);

    std::string filepath=Path::find(paths, filename);
    if(filepath.empty() || !File(filepath).exists() )
    {
        std::cout << "Run file " << filename << " containing CLI commands does not exist\n";
        std::cout << "WARNING if file failed to open - beware file name is converted to ALL LOWER CASE" << filename << "\n";
        return 0;
    }
    std::ifstream myfile( filepath );
    if (myfile)  // same as: if (myfile.good())
    {
        while (getline( myfile, line ))  // same as: while (getline( myfile, line ).good())
        {
            interpretLine(line);
        }
        myfile.close();
    }
    return 0;
}


////////////////////////////////////////////////////////////////////////////////
///
int CComandLineInterface::interpretLine(std::string line)
{
    std::vector<std::string>::iterator sit;

    std::string msg = Globals.trim(line);

    // all input translated to lower case  - so case independent
    std::transform(msg.begin(), msg.end(), msg.begin(), ::tolower);

    if (msg.compare("quit") == 0)
    {
        Globals.bCannedDemo() = false;
        Globals.sleep(1000);
        // Fixme: empty motion command queue.
        _bFlag = false;
        return rcs_state::EXITING;
    }
    else if (msg.compare("pause") == 0)
    {
        return rcs_state::PAUSED;
    }
    else if (msg.compare("step") == 0)
    {
        return rcs_state::STEP;
    }
    else if (msg.compare("resume") == 0)
    {
        return rcs_state::NORMAL;
    }
    else if (msg.compare("step") == 0)
    {
        return rcs_state::ONCE;
    }
    else if (msg.compare("degrees") == 0)
    {
        _bDegrees = true;
    }
    else if (msg.compare("manual") == 0)
    {
        Globals.bCannedDemo() = false;
        return rcs_state::NOOP;
    }
    else if (msg.compare("teststatus") == 0)
    {
        // fixme: make sure ros is running - how?
        if(!CRos::isRunning())
        {
            std::cout << "ROS core not running\n" ;
        }
        else
        {
            std::string status =  "header: auto \n"
                                  "crclcommandnum: 3\n"
                                  "crclstatusnum: 2\n"
                                  "crclcommandstatus: 2\n"
                                  "statuspose:\n"
                                  "  position:\n"
                                  "    x: 0.0\n"
                                  "    y: 0.0\n"
                                  "    z: 0.0\n"
                                  "  orientation:\n"
                                  "    x: 0.0\n"
                                  "    y: 0.0\n"
                                  "    z: 0.0\n"
                                  "    w: 1.0\n"
                                  "statusjoints:\n"
                                  "  header: auto\n"
                                  "  name: []\n"
                                  "  position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]\n"
                                  "  velocity: []\n"
                                  "  effort: []\n"
                                  "eepercent: 1.0\n";


            std::string exec;
            exec=Globals.strFormat(". /opt/ros/kinetic/setup.sh;. /home/michalos/src/gzaprsros-xenial/devel/setup.sh; rostopic pub /fanuc_crcl_status crcl_rosmsgs/CrclStatusMsg \"%s\"");
            std::cout << "Note this is hard coded in rostopic pub: $HOME/src/gzaprsros-xenial/devel/setup.sh;\n";
            std::cout << Globals.exec(exec.c_str());
        }
    }
    else if (msg.compare("test") == 0)
    {
        return rcs_state::TEST;
    }
    else if (msg.compare("auto") == 0)
    {
        interpretLine("macro macro_auto");
        Globals.bCannedDemo() = true;
        return rcs_state::AUTO;
    }
    else if (msg.compare("repeat") == 0)
    {
        Globals.bRepeatCannedDemo = true;
        return rcs_state::REPEAT;
    }
    else if (msg.compare("radian") == 0)
    {
        _bDegrees = false;
    }
    else if (msg.compare("rostopic") == 0)
    {
        // fixme: make sure ros is running - how?
        if(!CRos::isRunning())
            std::cout << "ROS core not running\n" ;
        else
            std::cout << Globals.exec(". /opt/ros/kinetic/setup.sh; rostopic list") << "\n" ;
    }

    else if (msg.compare("config") == 0)
    {
        std::string filename = Globals.appProperties["ConfigFile"];

        try
        {
            std::cout << "============================================================\n";
            std::cout << "Host name = " << Globals.exec("hostname") ;
            std::cout << "OS version = " << Globals.exec("cat /etc/*release  | grep DISTRIB_D | sed 's/.*=//'") ;
            std::cout << "Gnu Compiler version = " << __GNUC__ << "."<< __GNUC_MINOR__ << "\n";
            //            std::cout << "Gazebo Version " << (int) GAZEBO_MAJOR_VERSION << "." <<GAZEBO_MINOR_VERSION << "\n" ;

            std::cout << "Robot name = " <<   rcs_robot.robotName << "\n";
            std::cout << "Robot joint names = " <<   RCS::dumpStdVector(rcs_robot.jointNames) << "\n";
            std::cout << "ROS version = " << Globals.exec(". /opt/ros/kinetic/setup.sh; rosversion -d") << "\n" ;

            std::cout << "App name = " << Globals.appProperties["appName"] << "\n";
            std::cout << "App path = " << Globals.appProperties["appPath"]  << "\n";
            std::cout << "App version = " << Globals.appProperties["version"] << "\n";
            boost::filesystem::path apppath = Globals.appProperties["appPath"];
            std::time_t appt = last_write_time(apppath);
            std::cout << "App modification time = " << std::ctime(&appt);

        }
        catch (boost::filesystem::filesystem_error &e)
        {
            std::cerr << e.what() << '\n';
        }

        //        RCS::CController::dumpRobotNC(std::cout, ncs[0]);

    }

    else if (msg.compare( 0, strlen("setcwd"),"setcwd") == 0)
    {
        msg=msg.erase(0,std::string("setcwd").size());
        msg=Globals.trim(msg);
        Globals.appProperties["CurDirectory"]=msg;
    }
    else if (msg.compare( 0, strlen("run"),"run") == 0)
    {
        msg=msg.erase(0,std::string("run").size());
        msg=Globals.trim(msg);
        interpretFile(msg);
    }
    else if (msg.compare("timing") == 0)
    {
        std::cout << RCS::Thread::cpuLoads();
    }
    else if (msg.compare( 0, strlen("robot"),"robot") == 0)
    {
            std::cout << rcs_world.robot << "\n";
    }
    else if (msg.compare( 0, strlen("parts"),"parts") == 0)
    {
        std::cout << vectorDump<std::string>(rcs_world.part_list,"\n") << std::endl;
    }
    else if (msg.compare( 0, strlen("instances"),"instances") == 0)
    {
        std::cout << WorldModel::instances.dumpLocations();
    }
    else if (msg.compare( 0, strlen("inferences"),"inferences") == 0)
    {
        std::cout << WorldModel::instances.dumpInferences();
    }
    else if (msg.compare("robot") == 0)
    {
        std::cout << "Robot: \n";
        //
        // FIXME: use urdf robot name
        //
        std::cout << std::flush;
    }
    else if (msg.compare( 0, strlen("shell "), "shell ") == 0)
    {
        msg=msg.erase(0,std::string("shell ").size());
        msg=Globals.trim(msg);
        std::string echo = shell(msg.c_str());
        std::cout << echo << std::flush;
    }
    else if (msg.compare( 0, strlen("echo"), "echo") == 0)
    {
        bool bCrcl=false, bRos=false,bModel=false,
                bStatus=false,bCmd=false;

        msg=Globals.trim(msg.erase(0,std::string("echo").size()));
        if(msg.empty())
        {
            std::cout << "echo {crcl|ros|model} [status|cmd] \n";
        }
        else
        {
            if((bCrcl=(msg.compare( 0, strlen("crcl"), "crcl")) == 0))
                msg=Globals.trim(msg.erase(0,std::string("crcl").size()));
            else if((bRos=(msg.compare( 0, strlen("ros"), "ros")) == 0))
                msg=Globals.trim(msg.erase(0,std::string("ros").size()));
            else if((bModel=(msg.compare( 0, strlen("model"), "model")) == 0))
                msg=Globals.trim(msg.erase(0,std::string("model").size()));

            if((bCmd=(msg.compare( 0, strlen("cmd"), "cmd")) == 0))
                msg=Globals.trim(msg.erase(0,std::string("cmd").size()));
            else if((bStatus=(msg.compare( 0, strlen("status"), "status")) == 0))
                msg=Globals.trim(msg.erase(0,std::string("status").size()));

        }
    }
    else if (msg.compare( 0, strlen("debug"), "debug") == 0)
    {
        bool bCrcl=false, bRos=false,bModel=false,
                bStatus=false,bCmd=false,
                bOn=false,bOff=false;

        msg=Globals.trim(msg.erase(0,std::string("debug").size()));
        if(msg.empty())
        {
            std::cout << "debug turn on or off console logging\n";
            std::cout << "debug {crcl|ros|model} [status|cmd] {on|off}\n";
        }
        else
        {
            if((bCrcl=(msg.compare( 0, strlen("crcl"), "crcl")) == 0))
                msg=Globals.trim(msg.erase(0,std::string("crcl").size()));
            else if((bRos=(msg.compare( 0, strlen("ros"), "ros")) == 0))
                msg=Globals.trim(msg.erase(0,std::string("ros").size()));
            else if((bModel=(msg.compare( 0, strlen("model"), "model")) == 0))
                msg=Globals.trim(msg.erase(0,std::string("model").size()));

            if((bCmd=(msg.compare( 0, strlen("cmd"), "cmd")) == 0))
                msg=Globals.trim(msg.erase(0,std::string("cmd").size()));
            else if((bStatus=(msg.compare( 0, strlen("status"), "status")) == 0))
                msg=Globals.trim(msg.erase(0,std::string("status").size()));

            if((bOn=(msg.compare( 0, strlen("on"), "on")) == 0))
                msg=Globals.trim(msg.erase(0,std::string("on").size()));
            else if((bOff=(msg.compare( 0, strlen("off"), "off")) == 0))
                msg=Globals.trim(msg.erase(0,std::string("off").size()));

            if(bCrcl && bCmd)
                crcl::dbgCrclCommand=bOn;

            if(bCrcl && bStatus )
                crcl::dbgCrclStatus=bOn;

            if(bModel && bStatus)
                Globals.dbgModelStatus=bOn;


        }
    }

    else if (msg.compare("speeds") == 0)
    {
        std::cout << "speed:\n\trobot joint vel=" << rcs_status.currentRobotJointSpeed()
                  << "speed:\n\tjgripper    vel=" <<  rcs_status.currentGripperJointSpeed()
                  << "\n\tlinear            vel="  <<  rcs_status.currentLinearSpeed()
                  << "\n\trotational vel=       "  <<  rcs_status.currentAngularSpeed()
                  << "\n" << std::flush;
    }
    else if (msg.compare("slower") == 0)
    {
        crclApi->slow();
    }
    else if (msg.compare("faster") == 0)
    {
        crclApi->fast();
    }
    else if (msg.compare("reset") == 0)
    {
        // reset speeds and move "home"
        crclApi->medium();
        crclApi->moveJoints(rcs_robot.allJointNumbers(), rcs_robot.namedJointMove["joints.home"]);
        sleep(1);
        //        Globals.setRobotJointSpeeds(1.);
        //        Globals.setGripperJointSpeeds(1.);
        //        Globals.setLinearSpeeds(Globals.base_linearmax()[0]);
        //        Globals.setRotationalSpeeds(Globals.base_rotationmax()[0]);
        // reset gears back to original locations.
        rosCrclClient->reset();

    }
    else if (msg.compare( 0, strlen("goto "), "goto ") == 0)
    {
        msg=msg.erase(0,std::string("goto ").size());
        msg=Globals.trim(msg);
        std::transform(msg.begin(), msg.end(), msg.begin(), ::tolower);
        // see if existing shape
        if(msg =="?" || msg.empty())
        {
            // macros
            std::map<std::string, std::vector<std::string>>::iterator it;
            std:: cout << "MACROS" << "\n";
            for(it=rcs_robot.namedCommand.begin(); it!= rcs_robot.namedCommand.end(); it++)
            {
                std:: cout << (*it).first << "\n";
            }
            // poses
            std::map<std::string, tf::Pose>::iterator it1;
            std:: cout << "POSES" << "\n";
            for(it1=rcs_robot.namedPoseMove.begin(); it1!= rcs_robot.namedPoseMove.end(); it1++)
            {
                std:: cout << (*it1).first << "\n";
            }

            // joint moves
            std::map<std::string, std::vector<double>>::iterator it2;
            std:: cout << "JOINTS" << "\n";
            for(it2=rcs_robot.namedJointMove.begin(); it2!= rcs_robot.namedJointMove.end(); it2++)
            {
                std:: cout << (*it2).first << "\n";
            }
        }
        //        else if(ShapeModel::instances.findInstance(msg)!=NULL)
        //        {
        //            crclApi->moveTo(msg);
        //        }
        else if(rcs_robot.namedPoseMove.find(msg)!= rcs_robot.namedPoseMove.end())
        {
            crclApi->moveTo(rcs_robot.namedPoseMove[msg]);

        }
        // see if existing joint move name
        else if(rcs_robot.namedJointMove.find(msg)!= rcs_robot.namedJointMove.end())
        {
            crclApi->moveJoints(rcs_robot.allJointNumbers(), rcs_robot.namedJointMove[msg]);
        }
        else
        {
            std::cout <<  " Goto error: no joint move or instance named " << msg << " found\n";
            return 0;
        }
    }

    else if (msg.compare("home") == 0)
    {
        std::cout << "home" << RCS::dumpStdVector(rcs_robot.namedJointMove["joints.home"])<<"\n";

        crclApi->moveJoints(rcs_robot.allJointNumbers(), rcs_robot.namedJointMove["joints.home"]);
    }
    //    else if (msg.compare("safe") == 0)
    //    {
    //            crclApi->move_joints(ncs[ncindex]->robotKinematics()->allJointNumbers(), ncs[ncindex]->NamedJointMove["safe"]);
    //    }

    else if (msg.compare( 0, strlen("joints "), "joints ") == 0 )
    {
        msg=msg.erase(0,std::string("joints ").size());
        msg=Globals.trim(msg);
        std::vector<std::string> dbls = Globals.split(msg, ',');

        std::vector<double> positions = ConvertV(dbls);
        if (_bDegrees)
            positions = ScaleVector<double>(positions, M_PI / 180.0); //
        crclApi->moveJoints(rcs_robot.allJointNumbers(), positions);
    }
    else if (msg.compare( 0, strlen("jog "), "jog ") == 0 )
    {
        // jog jnt# incr
        msg=msg.erase(0,std::string("jog ").size());
        msg=Globals.trim(msg);
        char c;
        int jnt=-1,axis=-1;
        // this won't throw
        double amt;

        // Get jog type and amount
        if(sscanf(msg.c_str(), "%c %lf", &c, &amt)!=2)
            return rcs_state::NORMAL;

        // Parse jog type
        jog(c,0.0,jnt,axis);

        // Now loop accepting keyboard input asynchronously
        int doFlag=true;
        //       fflush(stdin);
        nonblock(NB_ENABLE);
        while(doFlag)
        {
            Globals.sleep(25);
            if(kbhit()==0)
            {
                continue;
            }
            c=fgetc(stdin);
            doFlag=jog(c,amt,jnt,axis);

            //crclApi->move_joints(jointnum,positions);
            // wait till curpos != nextpos UNLESS ERROR
        }
        nonblock(NB_DISABLE);
    }
    else if (msg.compare("open") == 0)
    {
        crclApi->openGripper();
    }
    else if (msg.compare("close") == 0)
    {
        crclApi->closeGripper();
    }
    else if (msg.compare(0, strlen("force "),"force ") == 0)
    {
        msg=msg.erase(0,std::string("force ").size());
        msg=Globals.trim(msg);
        std::vector<std::string> str_dbls = Globals.split(msg, ',');
        // Fixme: trim the strings.
        if(str_dbls.size() != 2)
        {
            fprintf(stderr,"force needs \"vel,fmax\" comma separaeted pair of values\n");
            return rcs_state::NORMAL;
        }
        for(size_t i=0; i< str_dbls.size(); i++)
        {
            double d;
            if(sscanf(str_dbls[i].c_str(), "%lf", &d)!=1)
            {
                fprintf(stderr,"vel and force must be doubles\n");
                return rcs_state::NORMAL;
            }
        }

        std::vector<double> dbls= ConvertStringVector<double>(str_dbls);
        crclApi->setVelGripper(dbls[0], dbls[1]);
    }
    else if (msg.compare(0, strlen("set dwell "),"set dwell ") == 0)
    {
        // dwell is seconds
        msg=msg.erase(0,std::string("set dwell ").size());
        msg=Globals.trim(msg);
        double d = FromStr<double>(msg);
        crclApi->setDwell(d);
    }
    else if (msg.compare(0, strlen("dwell "),"dwell ") == 0)
    {
        // dwelintl is seconds
        msg=msg.erase(0,std::string("dwell ").size());
        msg=Globals.trim(msg);
        double ee = FromStr<double>(msg);
        crclApi->doDwell(ee);
    }

    // move x,y,z,r,p,y
    else if (msg.compare( 0, strlen("pmove "), "pmove ") == 0 )
    {
        msg=msg.erase(0,std::string("pmove ").size());
        msg=Globals.trim(msg);
        tf::Pose pose;
        std::vector<std::string> str_dbls = Globals.split(msg, ',');
        std::vector<double> dbls= ConvertStringVector<double>(str_dbls);
        pose = Convert<std::vector<double>, tf::Pose> (dbls);
        crclApi->moveTo(pose);
    }
    //    else if (msg.compare( 0, strlen("move_to "), "move_to ") == 0 )
    //    {
    //        msg=msg.erase(0,std::string("move_to ").size());
    //        msg=Globals.trim(msg);  // object name
    //        // message should now contain object - can't really detect if exists
    //        crclApi->moveTo(msg);
    //    }



#ifdef GAZEBOCLI
    else if (msg.compare( 0, strlen("where "), "where ") == 0 )
    {
        msg=msg.erase(0,std::string("where ").size());
        msg=Globals.trim(msg);
        ShapeModel::CShape * shape = ShapeModel::instances.findInstance(msg);
        if(shape != NULL)
        {
            std::cout << msg << "=" << RCS::dumpPose(shape->_location) << "\n";
        }
        else
        {
            std::cout << msg << " not found \n" << std::flush;
        }
    }
#endif
    else if (msg.compare( 0, strlen("canned"), "canned") == 0 )
    {
        std::map<std::string, std::vector<double>>::iterator it;
        for(it=rcs_robot.namedJointMove.begin();
            it!=rcs_robot.namedJointMove.end(); it++)
        {
            std::cout << "namedJointMove:" << (*it).first;
            std::cout << RCS::dumpStdVector((*it).second) << "\n";
        }
        std::map<std::string, std::vector<std::string>>::iterator it1;
        for(it1=rcs_robot.namedCommand.begin();
            it1!=rcs_robot.namedCommand.end(); it1++)
        {
            std::cout << "namedCommand:" << (*it1).first;
            std::cout << RCS::dumpStdVector((*it1).second) << "\n";
        }
    }
    // use robot setup vector variable to command robot via lines of CLI
    else if (msg.compare( 0, strlen("macro "), "macro ") == 0 )
    {
        // Get name of macro
        msg=msg.erase(0,std::string("macro ").size());
        msg=Globals.trim(msg);

        // Find corresponding macro for robot
        std::map<std::string, std::vector<std::string>>::iterator it;
        if((it=rcs_robot.namedCommand.find(msg))== rcs_robot.namedCommand.end())
            return rcs_state::NORMAL;

        // Execute the command strings in the macro
        for(size_t k=0; k< (*it).second.size(); k++)
        {
            interpretLine((*it).second.at(k));
        }
    }
    else if (msg.compare( 0, strlen("status"), "status") == 0 )
    {
        std::vector<double> joints = rcs_robot._status.statusjoints.position;
        tf::Pose r_curpose=RCS::Convert<geometry_msgs::Pose, tf::Pose>(rcs_robot._status.statuspose);

        std::cout << "\tCommand num   = " << rcs_robot._status.crclcommandnum << "\n" ;
        std::cout << "\tStatus  num   = " << rcs_robot._status.crclstatusnum << "\n" ;
        std::cout << "\tStatus type   = " << rcs_robot._status.crclcommandstatus << "\n" ;
        std::cout << "\tJoints        = " << vectorDump(joints, ",", "%6.3f") << "\n" ;
        std::cout << "\tPose          = " << RCS::dumpPoseSimple(r_curpose) << "\n";
        std::cout << "\tPose          = " << rcs_robot._status.eepercent << "\n";

    }
    else if (msg.compare( 0, strlen("where"), "where") == 0 )
    {
        std::vector<double> joints = rcs_robot._status.statusjoints.position;
        std::cout << "Status Joints   =" << vectorDump(joints, ",", "%6.3f") << "\n" ;
        tf::Pose r_curpose=RCS::Convert<geometry_msgs::Pose, tf::Pose>(rcs_robot._status.statuspose);
        std::cout << "FK Robot Pose   =" << RCS::dumpPoseSimple(r_curpose) << "\n";
    }
    else if (msg.compare("help") == 0)
    {
        std::cout << "> help\t gives this output for joint or Cartesian moves.\n" ;
        std::cout << "> quit\t stops the controller and all its threads.\n" ;
        std::cout << "> slow\t slow rate of motion for joint or Cartesian moves.\n" ;
        std::cout << "> medium\t medium rate of motion for joint or Cartesian moves.\n" ;
        std::cout << "> fast\t fast rate of motion for joint or Cartesian moves.\n" ;
        std::cout << "> reset\t resets the rate of motion to medium.\n" ;
        std::cout << "> instances\t list of gear instances and position.\n" ;
        std::cout << "> slower\t slower rate of motion for joint or Cartesian moves (time 1/2.)\n" ;
        std::cout << "> faster\t faster rate of motion for joint or Cartesian moves (times 2). \n" ;
        std::cout << "> jog [x|y|z|# amt\t jog in xyz or joint # a certain amount. Follow by + or - for direction.\n" ;
        std::cout << "> set gripper amt\t moves the gipper to an absolute position but can't be 1 or 0.\n" ;
        std::cout << "> open\t moves the gipper to the open position as specified in the config.ini file\n" ;
        std::cout << "> close\t moves the gipper to the closed position as specified in the config.ini file\n" ;
        std::cout << "> pickup obj\t given a gear instance name will pickup the gear.\n" ;
        std::cout << "> retract\t moves away in positive z direction from current location.\n" ;
        std::cout << "> approach obj\t moves toward the gear instance given as a full name offset by an approach distance.\n" ;
        std::cout << "> where\t provides the current joint and Cartesian pose of the robot.\n" ;
        std::cout << "> record name\t records the current position to the filw with name header.\n" ;
        std::cout << "> goto name\t name is a stored names in config.ini file describing a set of joint position which the robot will move to..\n" ;
        std::cout << "> move_to obj\t moves to the gear instance given as a full name with a little offset from centroid of object.\n" ;
        std::cout << "> move x,y,z,r,p,y\t moves robot to the given pose given as xyz and roll, pitch, yaw.\n" ;
    }
    else
    {
        if(!msg.empty())
            std::cout << msg  << " :command not found \n" << std::flush;
        //std::cout << "> " << std::flush;
        return rcs_state::NOOP;
    }
    //std::cout << "> " << std::flush;

    return rcs_state::NOOP;
}
