#include <QCoreApplication>
#include <memory>
#include <string>
#include <fstream>
#include <chrono>
#include <thread>

#include <crclapi/nistcrcl.h>

#include <nist/Logger.h>

std::shared_ptr<crcl::crcl_server> pCrclServer;

///////////////////////////////////////////////////////////////////////////////
static bool ReadFile (std::string filename, std::string & contents)
{
    std::ifstream     in(filename.c_str( ), std::ifstream::in );
    std::stringstream buffer;

    if(!in.is_open())
    {
        logFatal("CGlobals::ReadFile failed file does not exist %s\n" , filename.c_str());

    }
    buffer << in.rdbuf( );
    contents = buffer.str( );
    return true;
}

///////////////////////////////////////////////////////////////////////////////
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
bool bRunning=true;

////////////////////////////////////////////////////////////////////////////////
void my_handler(int s)
{
    std::cout << "Caught ^C \n"<< std::flush;
    bRunning=false;
}

////////////////////////////////////////////////////////////////////////////////
struct sigaction sigIntHandler;

void CatchControlC()
{
    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);
}

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    CatchControlC();

    GLogger.Open(getexefolder()+"crcltest.log");
    GLogger.DebugLevel ( )=0;

    std::string robot_urdf;
    std::string base_link("fanuc_base_link");
    std::string tip_link("fanuc_link_6");

    ReadFile(getexefolder()+"config/FanucLRMate200iD.urdf",robot_urdf);

    RCS::CrclMessageQueue crclcmds;
    pCrclServer=std::shared_ptr<crcl::crcl_server>( new crcl::crcl_server(
                                                        "127.0.0.1",
                                                        64444,
                                                        0.1,
                                                        robot_urdf,
                                                        base_link,
                                                        tip_link ));

    pCrclServer->SetCmdQueue(&crclcmds);
    pCrclServer->Start();
    while(bRunning)
    {
        if (crclcmds.size()>0)
        {
            RCS::CanonCmd cc;
            nistcrcl::CrclCommandMsg msg = crclcmds.pop();
            cc.Set(msg);
            std::cout << cc.Dump();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    pCrclServer->Stop();
    return a.exec();
}
