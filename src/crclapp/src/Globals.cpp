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
//#pragma message "Compiling " __FILE__ 

#include <map>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "rcs/File.h"
#include "crclapp/Globals.h"
#include "crclapp/Demo.h"


CGlobals Globals;



// Static declarations
bool CGlobals::bRunning=true;
bool CGlobals::bPaused=false;

////////////////////////////////////////////////////////////////////////////////
static std::string  LeftTrim (std::string  str, std::string trim = " \t\r\n")
{
    size_t startpos = str.find_first_not_of(trim);

    if ( std::string::npos != startpos )
    {
        str = str.substr(startpos);
    }
    return str;
}
////////////////////////////////////////////////////////////////////////////////
static std::string  RightTrim (std::string  str, std::string trim = " \t\r\n")
{
    size_t endpos = str.find_last_not_of(trim);

    if ( std::string::npos != endpos )
    {
        str = str.substr(0, endpos + 1);
    }
    return str;
}



////////////////////////////////////////////////////////////////////////////////
void my_handler(int s)
{
    std::cout << "Caught ^C \n"<< std::flush;
    CGlobals::bRunning=false;
}

////////////////////////////////////////////////////////////////////////////////
struct sigaction sigIntHandler;

void CGlobals::catchControlC()
{
    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);
}


////////////////////////////////////////////////////////////////////////////////
CGlobals::CGlobals()
{
    srand((unsigned int) time(NULL)); //activates the simple randome number generator

    bIgnoreExceptions=true;
    bReadAllInstances=false;
    bCannedDemo()=false;
    bRepeatCannedDemo=0;

    bSingleThread=1;
    bCmdLineInterface=0;
    bCrclStreaming=0;

    // choose one or none of the following tests
    bDemoTest=1;
    bRawTest=0;
    bModelTest=0;
    bHighLevelCrclApiTest=0;

    // moveit flags
    bMoveit=0;
    eef_step=0.01;
    jump_threshold=0.0;
    avoid_collisions=0;


    // Global debugging files
    dbgModelStatus=0;
    dbgModelInferences=0;
    dbgTransforms=0;

    // moveit integration
    bRobotInWorldCoordinates=false;

    Globals.appProperties["appPath"] = getexepath();
    Globals.appProperties["appName"] = getexepath().substr(getexepath().find_last_of('/') + 1);
    Globals.appProperties["PackageSrcPath"] = getexefolder();
    Globals.appProperties["version"] = std::string("")+std::to_string(MAJOR) +":"+std::to_string(MINOR) +":"+std::to_string(BUILD) ;

    PLANNING_GROUP="fanucarm"; /**< name of move group of joints (defined in sdrf) */


}

////////////////////////////////////////////////////////////////////////////////
CGlobals::~CGlobals() {
    // Doesn't matter if never opened

}

void CGlobals::appConfig(ros::NodeHandlePtr nh)
{
    // Read application configuration parameters
    if(!nh->getParam(Globals.ns + "/app/bSingleThread", bSingleThread))
        ROS_ERROR_STREAM("Missing ROS param" << Globals.ns  << "/app/bSingleThread");
    if(!nh->getParam(Globals.ns + "/app/bCmdLineInterface", bCmdLineInterface))
        ROS_ERROR_STREAM("Missing ROS param" << Globals.ns  << "/app/bCmdLineInterface");
    if(!nh->getParam(Globals.ns + "/app/ignoreExceptions", bIgnoreExceptions))
        ROS_ERROR_STREAM("Missing ROS param" << Globals.ns  << "/app/ignoreExceptions");
    if(! nh->getParam(Globals.ns + "/app/bCrclStreaming", bCrclStreaming))
        ROS_ERROR_STREAM("Missing ROS param" << Globals.ns  << "/app/bCrclStreaming");

    // if false, use makeshift scurve trajectory
    if(!nh->getParam(Globals.ns + "/app/moveit/use", bMoveit))
        ROS_ERROR_STREAM("Missing ROS param" << Globals.ns  << "/app/moveit/uses");

    if(nh->hasParam(Globals.ns + "/app/moveit/eef_step"))
        nh->getParam(Globals.ns + "/app/moveit/eef_step", eef_step);
    if(nh->hasParam(Globals.ns + "/app/moveit/jump_threshold"))
        nh->getParam(Globals.ns + "/app/moveit/jump_threshold", jump_threshold);
    if(nh->hasParam(Globals.ns + "/app/moveit/avoid_collisions"))
        nh->getParam(Globals.ns + "/app/moveit/avoid_collisions", avoid_collisions);


    // App Debug flags
    if(nh->hasParam(Globals.ns + "/app/debug/transforms"))
        nh->getParam(Globals.ns + "/app/debug/transforms", dbgTransforms);
    if(!nh->getParam(Globals.ns + "/app/debug/demo", CGearDemo::bDebug))
        ROS_ERROR_STREAM("Missing ROS param" << Globals.ns  << "/app/debug/demo");


    // Read testing configuration parameters
    if(nh->hasParam(Globals.ns + "/app/test/bDemoTest"))
        nh->getParam(Globals.ns + "/app/test/bDemoTest", bDemoTest);
    if(nh->hasParam(Globals.ns + "/app/test/bRawTest"))
        nh->getParam(Globals.ns + "/app/test/bRawTest", bRawTest);
    if(nh->hasParam(Globals.ns + "/app/test/bModelTest"))
        nh->getParam(Globals.ns + "/app/test/bModelTest", bModelTest);
    if(nh->hasParam(Globals.ns + "/app/test/bHighLevelCrclApiTest"))
        nh->getParam(Globals.ns + "/app/test/bHighLevelCrclApiTest", bHighLevelCrclApiTest);


    ROS_DEBUG_STREAM( "********** Applicationo and testing configuration ");
    ROS_DEBUG_STREAM( "\tbSingleThread " << bSingleThread);
    ROS_DEBUG_STREAM( "\tbCrclStreaming= " << bCrclStreaming);
    ROS_DEBUG_STREAM( "\tbCmdLineInterface= " << bCmdLineInterface);
    ROS_DEBUG_STREAM( "\tbDemoTest= " << bDemoTest);
    ROS_DEBUG_STREAM( "\tbRawTest= " << bRawTest);
    ROS_DEBUG_STREAM( "\tbModelTest= " << bModelTest);
    ROS_DEBUG_STREAM( "\tbHighLevelCrclApiTest= " << bHighLevelCrclApiTest);
    ROS_DEBUG_STREAM( "\tGearDemo bDebug= " << CGearDemo::bDebug );
    ROS_DEBUG_STREAM( "\tUse moveit= " << bMoveit );
    ROS_DEBUG_STREAM( "\tmoveit eef_step= " << eef_step );
    ROS_DEBUG_STREAM( "\tmoveit jump_threshold= " << jump_threshold );
    ROS_DEBUG_STREAM( "\tmoveit avoid_collisions= " << avoid_collisions );
}

////////////////////////////////////////////////////////////////////////////////
bool CGlobals::ok()
{
    return bRunning;
}

////////////////////////////////////////////////////////////////////////////////
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/types.h>
pid_t CGlobals::procFind(const char* name)
{
    DIR* dir;
    struct dirent* ent;
    char* endptr;
    char buf[512];

    if (!(dir = opendir("/proc"))) {
        perror("can't open /proc");
        return -1;
    }

    while((ent = readdir(dir)) != NULL) {
        /* if endptr is not a null character, the directory is not
         * entirely numeric, so ignore it */
        long lpid = strtol(ent->d_name, &endptr, 10);
        if (*endptr != '\0') {
            continue;
        }

        /* try to open the cmdline file */
        snprintf(buf, sizeof(buf), "/proc/%ld/cmdline", lpid);
        FILE* fp = fopen(buf, "r");

        if (fp) {
            if (fgets(buf, sizeof(buf), fp) != NULL) {
                /* check the first token in the file, the program name */
                std::string first = strtok(buf, " ");
                if(first.find("/")!=std::string::npos)
                    first=first.substr(first.find_last_of('/') + 1);

                if (!strcmp(first.c_str(), name)) {
                    fclose(fp);
                    closedir(dir);
                    return (pid_t)lpid;
                }
            }
            fclose(fp);
        }

    }

    closedir(dir);
    return -1;
}




std::string CGlobals::replaceAll(std::string str, std::string oldstr, std::string newstr)
{
    std::string::size_type n = 0;

    while ( ( n = str.find( oldstr, n ) ) != std::string::npos )
    {
        str.replace( n, oldstr.size(), newstr );
        n += newstr.size();
    }

    return str;
}

////////////////////////////////////////////////////////////////////////////////
std::string CGlobals::strFormat(const char *fmt, ...)
{
    va_list argptr;
    va_start(argptr, fmt);
    int m;
    int n = (int) strlen(fmt) + 1028;
    std::string tmp(n, '0');
    while ((m = vsnprintf(&tmp[0], n - 1, fmt, argptr)) < 0) {
        n = n + 1028;
        tmp.resize(n, '0');
    }
    va_end(argptr);
    return tmp.substr(0, m);
}

////////////////////////////////////////////////////////////////////////////////
void CGlobals::sleep(unsigned int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms)) ;
}

////////////////////////////////////////////////////////////////////////////////
std::vector<std::string> CGlobals::split(const std::string &text, char sep, bool bKeepEmpty) {
    std::vector<std::string> tokens;
    std::size_t start = 0, end = 0;
    //bKeepEmpty; // unused for now;
    std::string token;
    while ((end = text.find(sep, start)) != std::string::npos) {
        token = text.substr(start, end - start);
//        if (!token.empty())
            tokens.push_back(token);
        start = end + 1;
    }
    token = trim(text.substr(start));
//    if (!token.empty())
        tokens.push_back(token);
    return tokens;
}

////////////////////////////////////////////////////////////////////////////////
void CGlobals::assignOfs(std::ostream *inOfs, std::ostream *replacementOfs)
{

    (*inOfs).copyfmt((*replacementOfs));
    (*inOfs).clear((*replacementOfs).rdstate()); //2
    (*inOfs).basic_ios<char>::rdbuf((*replacementOfs).rdbuf());           //3

}

////////////////////////////////////////////////////////////////////////////////
std::string  CGlobals::trim (std::string  s)
{
    return LeftTrim(RightTrim(s) );
}
////////////////////////////////////////////////////////////////////////////////
bool CGlobals::readFile (std::string filename, std::string & contents)
{
    std::ifstream     in(filename.c_str( ), std::ifstream::in );
    std::stringstream buffer;

    if(!in.is_open())
    {
        std::cerr << "CGlobals::ReadFile failed file does not exist" << filename << "\n" << std::flush;

    }
    buffer << in.rdbuf( );
    contents = buffer.str( );
    return true;
}
////////////////////////////////////////////////////////////////////////////////
void CGlobals::writeFile (std::string filename, std::string & contents)
{
    std::ofstream outFile(filename.c_str( ) );

    outFile << contents.c_str( );
}
////////////////////////////////////////////////////////////////////////////////
void CGlobals::appendFile (std::string filename, std::string  contents)
{
    std::ofstream outFile;

    outFile.open(filename.c_str( ), std::ofstream::out | std::ofstream::app);
    outFile << contents.c_str( );
}

/**
* @brief Tokenize takes a string and delimiters and parses into vector
* @param str string to tokenize
* @param delimiters string containing delimiters
* @return  std vector of tokens from parsed string
*/
std::vector<std::string> CGlobals::tokenize (const std::string & str,
    const std::string & delimiters)
{
    std::vector<std::string> tokens;
    std::string::size_type   delimPos = 0, tokenPos = 0, pos = 0;

    if ( str.length( ) < 1 )
    {
        return tokens;
    }

    while ( 1 )
    {
        delimPos = str.find_first_of(delimiters, pos);
        tokenPos = str.find_first_not_of(delimiters, pos);

        if ( std::string::npos != delimPos )
        {
            if ( std::string::npos != tokenPos )
            {
                if ( tokenPos < delimPos )
                {
                    tokens.push_back(str.substr(pos, delimPos - pos));
                }
                else
                {
                    tokens.push_back("");
                }
            }
            else
            {
                tokens.push_back("");
            }
            pos = delimPos + 1;
        }
        else
        {
            if ( std::string::npos != tokenPos )
            {
                tokens.push_back(str.substr(pos));
            }
            else
            {
                tokens.push_back("");
            }
            break;
        }
    }

    return tokens;
}

static std::string trim (std::string source, std::string delims = " \t\r\n")
{
    std::string result = source.erase(source.find_last_not_of(delims) + 1);
    return result.erase(0, result.find_first_not_of(delims));
}
/**
* @brief TrimmedTokenize takes a string and delimiters and parses into
* vector,
* but trims tokens of leading and trailing spaces before saving
* @param str string to tokenize
* @param delimiters string containing delimiters
* @return  std vector of tokens from parsed string trimmed
*  tokens of leading and trailing spaces
*/
std::vector<std::string> CGlobals::trimmedTokenize (std::string value,
    std::string delimiter)
{
    std::vector<std::string> tokens = tokenize(value, delimiter);

    for ( size_t i = 0; i < tokens.size( ); i++ )
    {
        if ( tokens[i].empty( ) )
        {
            tokens.erase(tokens.begin( ) + i);
            i--;
            continue;
        }
        tokens[i] = trim(tokens[i]);
    }
    return tokens;
}

#ifdef _WINDOWS
#include "targetver.h"
#include "Windows.h"

// #include "StdStringFcn.h"
#include <Lmcons.h>

#define SECURITY_WIN32
#include "security.h"
#pragma comment(lib, "Secur32.lib")

#if 0
unsigned int CGlobals::ErrorMessage (std::string errmsg)
{
    OutputDebugString(errmsg.c_str( ) );
    std::cout << errmsg;
    return E_FAIL;
}
unsigned int CGlobals::DebugMessage (std::string errmsg)
{
    OutputDebugString(errmsg.c_str( ) );
    return E_FAIL;
}
#endif
// std::string CGlobals::ExeDirectory()
// { unsigned int CGlobals::DebugStrFormat(const char *fmt, ...) {
    va_list argptr;

    va_start(argptr, fmt);
    std::string str = FormatString(fmt, argptr);
    va_end(argptr);
    return -1; // FIXME: return DebugMessage( str);
}
//	TCHAR buf[1000];
//	GetModuleFileName(NULL, buf, 1000);
//	std::string path(buf);
//	path=path.substr( 0, path.find_last_of( '\\' ) +1 );
//	return path;
// }

#if 0
std::string CGlobals::GetUserName ( )
{
    TCHAR username[UNLEN + 1];
    DWORD size = UNLEN + 1;

    ::GetUserName( (TCHAR *) username, &size);
    return username;
}
std::string CGlobals::GetUserDomain ( )
{
    TCHAR username[UNLEN + 1];
    DWORD size = UNLEN + 1;

    // NameDnsDomain campus.nist.gov
    if ( GetUserNameEx(NameSamCompatible, (TCHAR *) username, &size) )
    {
        std::string domain = username;
        domain = domain.substr(0, domain.find_first_of('\\') );
        return domain;
    }
    return "";
}
#endif
std::string CGlobals::GetTimeStamp (TimeFormat format)
{
    SYSTEMTIME st;
    char       timestamp[64];

    GetSystemTime(&st);
    sprintf(timestamp, "%4d-%02d-%02dT%02d:%02d:%02d", st.wYear, st.wMonth,
        st.wDay, st.wHour, st.wMinute, st.wSecond);

    if ( format == GMT_UV_SEC )
    {
        sprintf(timestamp + strlen(timestamp), ".%04dZ", st.wMilliseconds);
    }
    else
    {
        strcat(timestamp, "Z");
    }

    return timestamp;
}
#else
#if 0
unsigned int CGlobals::ErrorMessage (std::string errmsg)
{
    std::cout << errmsg;
    return -1;
}
unsigned int CGlobals::DebugMessage (std::string errmsg)
{
    std::cout << errmsg;
    return -1;
}
#endif
static inline std::string FormatString(const char *fmt, va_list ap) {
    int m, n = (int) strlen(fmt) + 1028;
    std::string tmp(n, '0');

    while ((m = vsnprintf(&tmp[0], n - 1, fmt, ap)) < 0) {
        n = n + 1028;
        tmp.resize(n, '0');
    }

    return tmp.substr(0, m);
}

#include <sys/time.h>
std::string CGlobals::getTimeStamp (TimeFormat format)
{
    char            timeBuffer[50];
    struct tm *     timeinfo;
    struct timeval  tv;
    struct timezone tz;

    gettimeofday(&tv, &tz);
    timeinfo = ( format == LOCAL ) ? localtime(&tv.tv_sec) : gmtime(&tv.tv_sec);

    switch ( format )
    {
    case HUM_READ:
        {
            strftime(timeBuffer, 50, "%a, %d %b %Y %H:%M:%S %Z", timeinfo);
        }
        break;

    case GMT:
        {
            strftime(timeBuffer, 50, "%Y-%m-%dT%H:%M:%SZ", timeinfo);
        }
        break;

    case GMT_UV_SEC:
        {
            strftime(timeBuffer, 50, "%Y-%m-%dT%H:%M:%S", timeinfo);
        }
        break;
    case LOGFILE:
        {
            strftime(timeBuffer, 50, "%Y-%m-%d-%H-%M-%S", timeinfo);
        }
        break;
    case LOCAL:
        {
            strftime(timeBuffer, 50, "%Y-%m-%dT%H:%M:%S%z", timeinfo);
        }
        break;
    }

//    if ( format == GMT_UV_SEC )
//    {
//        sprintf(timeBuffer + strlen(timeBuffer), ".%06dZ", tv.tv_usec);
//    }

    return std::string(timeBuffer);

}
#endif
