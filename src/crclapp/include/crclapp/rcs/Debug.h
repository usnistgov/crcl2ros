
#ifndef _DEBUG_H
#define _DEBUG_H

/*
DISCLAIMER:
This software was produced by the National Institute of Standards
and Technology (NIST), an agency of the U.S. government, and by statute is
not subject to copyright in the United States.  Recipients of this software
assume all responsibility associated with its operation, modification,
maintenance, and subsequent redistribution.

See NIST Administration Manual 4.09.07 b and Appendix I.
 */

// C++
#include <stdarg.h>
#include <vector>

// Boost
#include <boost/format.hpp>

// Ros
#include <tf/tf.h>

#include "Conversions.h"


namespace RCS {

const double pscale=1.0;

// This gives tf rpy
//inline void getRPY(const tf::Pose pose, double &roll, double &pitch, double &yaw) {
//    tf::Matrix3x3 rot = pose.getBasis();
//    rot.getRPY(roll, pitch, yaw);
//}

/**
 * @brief toDegree convert angle from radian to degree
 * @param ang
 * @return
 */
inline double toDegree(double ang) {
    return ang * 180.0 / M_PI;
}


////////////////////////////////////////////////////////////////////////////////
/**
 * @brief vectorDump to a dump of a vector to a stringstream, return string.
 * @param v vector of template type T
 * @param separator separator between type ostream.
 * @return std string of stringstream
 */
template<typename T>
inline std::string vectorDump(std::vector<T> v, std::string separator=",") {
    std::stringstream s;

    for (size_t i = 0; i < v.size(); i++) {
        if(i>0)
            s << separator;
        s << v[i] ;
    }
    return s.str();
}

////////////////////////////////////////////////////////////////////////////////

/**
 * @brief vectorDump of vector of doubles
 * @param v vector of doubles
 * @param separator separator between double outputs
 * @param format string to format double, default %5.2f
 * @return std string from stringstream conversion
 */
inline std::string vectorDump(std::vector<double> v, std::string separator, std::string format="%5.2f")
{
    std::stringstream s;

    for (size_t i = 0; i < v.size(); i++) {
        if(i>0)
            s << separator;
        s << boost::format(format.c_str()) % v[i] ;
    }
    return s.str();
}

////////////////////////////////////////////////////////////////////////////////
/**
* \brief dumpPose takes a urdf pose  and generates a string describing pose.
* Can be used as std::cout << DumpPose(pose);
*/
inline std::string dumpPose(tf::Pose & pose) {
    std::stringstream s;

    s << "\nXYZ   = " << pscale * pose.getOrigin().x() << "," << pscale * pose.getOrigin().y() << "," << pscale * pose.getOrigin().z() << std::endl;
    double roll=0, pitch=0, yaw=0;
    tf::Quaternion2Rpy(pose.getRotation(), roll, pitch, yaw);
    //getRPY(pose, roll, pitch, yaw);
    s << "RPY = " << Rad2Deg(roll) << "," << Rad2Deg(pitch) << "," << Rad2Deg(yaw) << "\n";
    s << "Q   = " << pose.getRotation().x() << "," << pose.getRotation().y() << "," << pose.getRotation().z() << "," << pose.getRotation().w();
    //s << Crcl::DumpRotationAsCrcl(pose)<< std::endl;
    return s.str();
}


////////////////////////////////////////////////////////////////////////////////
/**
 * @brief DumpPoseSimple generatseriallinkrobotes string of xyz origin and rpy rotation from a tf pose.
 * @param pose tf pose
 * @return std::string
 */
inline std::string dumpPoseSimple(tf::Pose pose) {
    std::stringstream s;


    s << std::setprecision(3) << boost::format("%7.2f") % (pscale * pose.getOrigin().x()) << "," <<
         boost::format("%7.2f") % (pscale * pose.getOrigin().y()) << "," <<
         boost::format("%7.2f") % (pscale * pose.getOrigin().z()) << ",";
    double roll=0, pitch=0, yaw=0;
    //getRPY(pose, roll, pitch, yaw);
    //tf::Matrix3x3(pose.getRotation()).getRPY(roll, pitch, yaw);
    tf::Quaternion2Rpy(pose.getRotation(), roll, pitch, yaw);

//    s << "RPY= " << boost::format("%5.2f") % Rad2Deg(roll) << "," <<
//         boost::format("%5.2f") % Rad2Deg(pitch) << "," <<
//         boost::format("%5.2f") % Rad2Deg(yaw);
    s << std::setprecision(3) << boost::format("%5.3f") % pose.getRotation().x() << ","
      << boost::format("%5.3f") % pose.getRotation().y() << ","
      << boost::format("%5.3f") % pose.getRotation().z() << ","
      << boost::format("%5.3f") % pose.getRotation().w();

    return s.str();
}
////////////////////////////////////////////////////////////////////////////////
/**
* \brief dumpEVector generates a debug string for an  Vector.
* Can be used as std::cout << DumpEPosition(v);
*/
template<typename T>
inline std::string dumpEVector(const T & v) {
    std::stringstream s;
    for (int i = 0; i < v.size(); i++)
        s << boost::format("%8.5f:") % v(i);
    return s.str();
}

////////////////////////////////////////////////////////////////////////////////

/**
* \brief dumpVector generates a debug string for an  Vector.
* Can be used as std::cout << DumpVector(v);
*/
inline std::string dumpVector(const tf::Vector3 & v) {
    std::stringstream s;
    s << boost::format("%8.5f ") % v.x();
    s << boost::format("%8.5f ") % v.y();
    s << boost::format("%8.5f") % v.z();
    return s.str();
}

////////////////////////////////////////////////////////////////////////////////
/**
* \brief dump std Vector generates a debug string for an Eigen Vector.
* Can be used as std::cout << DumpEPosition(v);
*/
template<typename T>
inline std::string dumpStdVector(const T & v) {
    std::stringstream s;
    for (int i = 0; i < v.size(); i++)
        s <<  v[i] << ":";
    return s.str();
}

////////////////////////////////////////////////////////////////////////////////
/**
* \brief DumpPose takes a urdf pose  and generates a string describing pose.
* Can be used as std::cout << DumpPose(pose);
*/
inline std::ostream & operator<<(std::ostream & os, tf::Pose & pose) {
    std::stringstream s;
    s << "Translation = " << pscale * pose.getOrigin().x() << "," << pscale * pose.getOrigin().y() << "," << pscale * pose.getOrigin().z() << std::endl;
    double roll=0, pitch=0, yaw=0;
    tf::Quaternion2Rpy(pose.getRotation(), roll, pitch, yaw);
//    getRPY(pose, roll, pitch, yaw);
    s << "Rotation = " << Rad2Deg(roll) << "," << Rad2Deg(pitch) << "," << Rad2Deg(yaw) << std::endl;
    s << "Quaterion = " << pose.getRotation().x() << "," << pose.getRotation().y() << "," << pose.getRotation().z() << "," << pose.getRotation().w();
    os << s.str();
    return os;
}

////////////////////////////////////////////////////////////////////////////////
/**
* \brief dumpQuaterion takes a urdf quaterion  and generates a string describing x,y,z,w coordinates.
* Can be used as std::cout << DumpQuaterion(urdf::rotation);
*/
inline std::string dumpQuaterion(const tf::Quaternion & rot) {
    std::stringstream s;
     s << boost::format("[X=%8.4f,") % rot.x();
    s << boost::format("Y=%8.4f,") % rot.y();
    s << boost::format("Z=%8.4f,") % rot.z();
    s << boost::format("W=%8.4f]") % rot.w();
    return s.str();
}



inline std::string dumpMatrix(const tf::Matrix3x3& m){
    std::stringstream s;
    s << "|" << m[0][0] << " " << m[0][1] << " " << m[0][2]  << "|" << "\n"
              << "|" << m[1][0] << " " << m[1][1] << " " << m[1][2]  << "|" << "\n"
              << "|" << m[2][0] << " " << m[2][1] << " " << m[2][2]  << "|" << "\n" ;
    return s.str();

}




} /*namespace RCS */

inline std::ostream& operator<<(std::ostream& os, const
                                tf::Quaternion& q)
{
    return os << "[ " << q.x() << " " << q.y() << " " << q.z() << " " <<  q.w() << " ]";
}
inline  std::ostream& operator<<(std::ostream& os,const tf::Matrix3x3& m){
    os << "|" << m[0][0] << " " << m[0][1] << " " << m[0][2]  << "|" << "\n"
              << "|" << m[1][0] << " " << m[1][1] << " " << m[1][2]  << "|" << "\n"
              << "|" << m[2][0] << " " << m[2][1] << " " << m[2][2]  << "|" << "\n" ;
    return os;

}
#endif
