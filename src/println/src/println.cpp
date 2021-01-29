

#include <ros/ros.h>
#include <ctime>
#include <string>
#include <regex>
static     std::string replace(std::string str, std::string oldstr, std::string newstr)
{
    std::string::size_type n = 0;
    if ( ( n = str.find( oldstr, n ) ) != std::string::npos )
    {
        str.replace( n, oldstr.size(), newstr );
        n += newstr.size();
    }
    return str;
}

int main(int argc, char** argv)
{

    const std::string node_name = "printf"+ std::to_string(std::time(0));
    ros::M_string remappings;
    remappings["__master"]=  "http://localhost:11311";
    remappings["__name"]= node_name;

    // Initialize ROS environment ROS_MASTER_URI MUST BE SET or seg fault
    ros::init(remappings,node_name) ;

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nh("~");
    for(int i=1; i< argc ; ++i)
    {
        std::string arg(argv[i]);
        size_t n;
        if((n=arg.find("--ignore"))!= std::string::npos)
            return 0;

    }
    for(int i=1; i< argc ; ++i)
    {
        std::string arg(argv[i]);
        size_t n;

        if((n=arg.find("$"))!= std::string::npos)
        {

            std::string str(arg);
            std::regex regexp("\\$[/A-Z_a-z0-9]+");             // Getting the regex object
            std::smatch sm;

            // regex_search that searches pattern regexp in the string mystr
            while(regex_search(str, sm, regexp))
            {
                str=sm.suffix();
                std::string _param=sm[0];

                _param=_param.substr(1,_param.size()-1); // length is 2 less $ + ' '
                if (!ros::param::has(_param))
                    std::cerr << "Couldn't find param " << _param << "\n";
                std::string _value;
                ros::param::get(_param, _value);
                arg=replace(arg, "$"+_param, _value);
            }

        }

        std::cout << arg << "\n";
    }

    std::cout << std::flush;
    ros::Rate r(1); // 1 hz
    for(int i=0; i< 5 &&ros::ok(); ++i){
        r.sleep();
    }
    return 0;
}

