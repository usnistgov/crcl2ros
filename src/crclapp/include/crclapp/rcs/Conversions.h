#ifndef Conversion_Header
#define Conversion_Header

/**
DISCLAIMER:
This software was produced by the National Institute of Standards
and Technology (NIST), an agency of the U.S. government, and by statute is
not subject to copyright in the United States.  Recipients of this software
assume all responsibility associated with its operation, modification,
maintenance, and subsequent redistribution.

See NIST Administration Manual 4.09.07 b and Appendix I.
 */


#ifndef ROS
// This can work but ROS include path seems to be better.
#include <tf/tf.h>
#else
#include <tf/tf.h>
#include "tf/transform_datatypes.h"
#endif

// ROS messages
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
//#include <geometry_msgs/Transform.h>
//#include <geometry_msgs/Twist.h>
//#include <geometry_msgs/Vector3.h>
//#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/JointState.h>

// URDF
#include <urdf/model.h>

// CRCL representations
#include "Core.h"


// Error at compile time for non handled convert
#include <boost/static_assert.hpp>

#define MATH_DEFINES
#include <math.h>

#ifndef Deg2Rad
#define Deg2Rad(Ang)    ( (double) ( Ang * M_PI / 180.0 ) )
#define Rad2Deg(Ang)    ( (double) ( Ang * 180.0 / M_PI ) )
#define MM2Meter(d)     ( (double) ( d / 1000.00 ) )
#define Meter2MM(d)     ( (double) ( d * 1000.00 ) )
#endif

#ifndef SQ
#define SQ(X) (X*X)
#endif

#ifndef RPY_P_FUZZ
#define RPY_P_FUZZ (0.000001)
#endif

template<class ValueType>
inline ValueType FromStr(std::string str) {
    std::istringstream stream(str);
    ValueType val;
    try {
        stream>> val;
    } catch (...) {
    }
    return val;
}

template<class ValueType>
inline std::string  ToStr(ValueType val) {
    std::stringstream stream;
    try {
        stream<<val;
    } catch (...) {
    }
    return stream.str();
}

// tf documentation found at:
// Transform http://docs.ros.org/jade/api/tf/html/c++/classtf_1_1Transform.html
// Quaternion http://docs.ros.org/jade/api/tf/html/c++/classtf_1_1Quaternion.html

namespace tf {

/*!
 * \brief Return identity tf pose.
 */
inline tf::Pose Identity() {
    tf::Pose p;
    p.setIdentity();
    return p;
}

/*!
* \brief Return identity tf quaternion.
*/
inline tf::Quaternion QIdentity() {
    return tf::Quaternion(0.0, 0.0, 0.0, 1.0);
}

/*!
 * \brief Return all zero tf vector3.
 */
inline tf::Vector3 VIdentity() {
    return tf::Vector3(0.0, 0.0, 0.0);
}

/*!
 * \brief Return tf vector3 Z axis.
 */
inline tf::Vector3 ZAxis() {
    return tf::Vector3(0.0, 0.0, 1.0);
}

/*!
 * \brief Return tf vector3 Y axis.
*/
inline tf::Vector3 YAxis() {
    return tf::Vector3(0.0, 1.0, 0.0);
}

/*!
* \brief Return tf vector3 X axis.
*/
inline tf::Vector3 XAxis() {
    return tf::Vector3(1.0, 0.0, 0.0);
}

/*!
 * \brief Return tf quaternion with the rotation set using axis angle notation.
 */
inline tf::Quaternion AxisAngle(double angle, tf::Vector3 axis) {
    tf::Quaternion q;
    q.setRotation(axis, angle);
    return q;
}

inline tf::Matrix3x3 Rpy2RotMatrix(double r, double p, double y)
{
    double sa, sb, sg;
    double ca, cb, cg;
    tf::Matrix3x3 m;

    sa = sin(y);
    sb = sin(p);
    sg = sin(r);


    ca = cos(y);
    cb = cos(p);
    cg = cos(r);


    // m is row/column
    m[0][0] = ca * cb;
    m[1][0] = ca * sb * sg - sa * cg;
    m[2][0] = ca * sb * cg + sa * sg;


    m[0][1] = sa * cb;
    m[1][1] = sa * sb * sg + ca * cg;
    m[2][1] = sa * sb * cg - ca * sg;


    m[0][2] = -sb;
    m[1][2] = cb * sg;
    m[2][2] = cb * cg;


    return m;
}

inline tf::Quaternion Rpy2Quaternion(double r, double p, double y)
{
    tf::Matrix3x3 m = Rpy2RotMatrix(r, p, y);
    tf::Quaternion q;
    m.getRotation(q);
    return q;

}


inline int RotMatrix2Rpy(tf::Matrix3x3 m, double &r, double &p, double &y)
{
    p = atan2(-m[0][2], sqrt(SQ(m[0][0]) + SQ(m[0][1])));


    if (fabs(p - M_PI_2) < RPY_P_FUZZ) {
        r = atan2(m[1][0], m[1][1]);
        p = M_PI_2;
        /* force it */
        y = 0.0;
    } else if (fabs(p + M_PI_2) < RPY_P_FUZZ) {
        r = -atan2(m[1][2], m[1][1]);
        p = -M_PI_2;
        /* force it */
        y = 0.0;
    } else {
        r = atan2(m[1][2], m[2][2]);
        y = atan2(m[0][1], m[0][0]);
    }


    return  0;
}

inline void Quaternion2Rpy(tf::Quaternion q, double &r, double &p, double &y)
{
    tf::Matrix3x3 m ;
    m.setRotation(q);
    RotMatrix2Rpy(m,r,p,y);
}
}


/**
 * \brief The namespace Conversion provides some utilities to convert from one representation into another.
 * Representations include tf, Eigen::Affine3d, std::vector, geometry_msgs::Pose,
 * geometry_msgs::Point, and CRCL.
 *
 * Clearly there may be faster const references, but they require special line to convert,
 * cannot be done in line since you cannot pass a const reference to a constructor on the
 * stack in g++ unless you override a warning.
 *
 * For g++, compilation would be faster if these conversion routines were placed in source
 * file (cpp) OR you used precompiled header in g++.
 * here is a "silent" error when exceeding precompiled header limits  in g++. (Or was at one time).
 *
 * */
namespace RCS {

/*!
 * \brief ToVector calling parameters MUST match e.g., double or long depending on template, OR wont work
 * \param n number of items to push into std vector.
 * \param ... list of n template type items
 * \return std vector of type T
  */
template<typename T>
inline std::vector<T> ToVector(int n, ...) {
    std::vector<T> ds;
    va_list args; // define argument list variable
    va_start(args, n); // init list; point to last
    //   defined argument

    for (int i = 0; i < n; i++) {
        double d = va_arg(args, T); // get next argument
        ds.push_back(d);
    }
    va_end(args); // clean up the system stack
    return ds;
}

/*!
 * \brief Convert a list of std vector  type T from degrees to radians
 * Change template from typep into double conversion factor.
 *  you can't use float literals as template parameters
 */
template<typename T>
inline std::vector<T> ScaleVector(std::vector<T> goaljts, double multiplier =  M_PI / 180.0) {
    // transform angles from degree to radians
    std::transform(goaljts.begin(), goaljts.end(), goaljts.begin(),
                   std::bind1st(std::multiplies<double>(), multiplier));
    return goaljts;
}
/*!
 * \brief Convert a list of std vector  type T from degrees to radians
 * Change template from typep into double conversion factor.
 *  you can't use float literals as template parameters
 */
template<typename T>
inline std::vector<T> ToRadianVector(std::vector<T> goaljts) {
    // transform angles from degree to radians
    std::transform(goaljts.begin(), goaljts.end(), goaljts.begin(),
                   std::bind1st(std::multiplies<double>(), M_PI / 180.0));
    return goaljts;
}

/*!
 * \brief Convert a list of std vector  type string into a vector of doubles
 * \return converted set of std::vector<double> returned if error assume 0 put into array.
 */
//    template<>
//    inline std::vector<double> Convert<std::vector<std::string>,  std::vector<double> >(
//            std::vector<std::string> stringVector) {
//        std::vector<double> doubleVector;
//        std::transform(stringVector.begin(), stringVector.end(), back_inserter(doubleVector),
//                [](std::string const& val) {
//                    return stod(val);
//                });
//        return doubleVector;
//    }

/**
 * Convert a list of std vector  type string into a vector of Type template T
 * \return converted vector
 */
template<typename To>
inline std::vector<To> ConvertStringVector(std::vector<std::string> From) {
    std::vector<To> toVector;
    for (size_t i = 0; i < From.size(); i++) {
        // Do I need a catch?
        std::istringstream ss(From[i]);
        To result;
        try { ss >> result; } catch(...){ result=To(); }
        toVector.push_back(result);
    }
    return toVector;
}

/*!
 * \brief Empty conversion of type from into type to. If called, asserts.
 * \param f is defined in the template corresponding to the "From" typename.
 * \return to is defined in the template corresponding "To"  typename
 */
template<typename From, typename To>
inline To Convert(From f) {
    BOOST_STATIC_ASSERT(sizeof (To) == 0);
    assert(0);
}

// tf


/*!
 * \brief Convert geometry_msgs::Pose into tf::Pose.
 * \param pose is copy constructor of geometry_msgs::Pose.
 * \return tf::Pose
 */
template<>
inline tf::Pose Convert<geometry_msgs::Pose, tf::Pose>(geometry_msgs::Pose m) {
    return tf::Pose(tf::Quaternion(m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w),
                    tf::Vector3(m.position.x, m.position.y, m.position.z));
}


/*!
 * \brief Convert std::vector<double> into tf::Pose.
 * \param ds are an array of 6 doubles to define tf Pose.
 * \return tf::Pose
 */
template<>
inline tf::Pose Convert<std::array<double, 6>, tf::Pose>(std::array<double, 6> ds)
{
    // yaw pitch roll order but who knows
    return tf::Pose(tf::Quaternion(ds[5], ds[4], ds[3]), tf::Vector3(ds[0], ds[1], ds[2]));
}

/**
 * \brief Convert std::vector<double> into tf::Pose.
 * \param ds are an array of 7 doubles to define tf Pose.
 * Because 7 doubles use quaternion.
 * \return tf::Pose
 */

template<>
inline tf::Pose Convert<std::array<double, 7>, tf::Pose>(std::array<double, 7> ds)
{
    return tf::Pose(tf::Quaternion(ds[3], ds[4], ds[5], ds[6]), tf::Vector3(ds[0], ds[1], ds[2]));
}

/**
 * \brief Convert std::vector<double> into tf::Pose.
 * \param ds are an array of 6 or 7 doubles to define tf Pose.
 * If 6 double use rpy if 7 use quaternion.
 * \return tf::Pose
 */
template<>
inline tf::Pose Convert<std::vector<double>, tf::Pose>(std::vector<double> ds)
{
    tf::Pose pose;
    if(ds.size() == 6)
    {
        std::array<double, 6> arr;
        std::copy_n(ds.begin(), 6, arr.begin());
        pose = Convert<std::array<double, 6>, tf::Pose> (arr);
    }
    else if(ds.size() == 7)
    {
        std::array<double, 7> arr;
        std::copy_n(ds.begin(), 7, arr.begin());
        pose = Convert<std::array<double, 7>, tf::Pose> (arr);
        // message should now contain object - can't really detect if exists
    }
    else
    {
        assert(0);
    }
    return pose;
}

inline tf::Pose ConvertDblVectorTf(std::vector<double> ds) {
    assert(ds.size() > 6);
    return tf::Pose(tf::Quaternion(ds[3], ds[4], ds[5], ds[6]), tf::Vector3(ds[0], ds[1], ds[2]));
}
/*!
 * \brief Convert std::vector<double> into tf::Vector3.
 * \param ds are an array of 3 doubles to define tf Vector3.
 * \return tf::Vector3
 */
template<>
inline tf::Vector3 Convert<std::vector<double>, tf::Vector3>(std::vector<double> ds) {
    assert(ds.size() > 2);
    return tf::Vector3(ds[0], ds[1], ds[2]);
}

/*!
 * \brief Convert tf::Quaternion into tf::Pose.
 * \param q rotation is converted into a tf::Pose.
 * \return tf::Pose
 */
template<>
inline tf::Pose Convert<tf::Quaternion, tf::Pose> (tf::Quaternion q) {
    return tf::Pose(q, tf::Vector3(0., 0., 0.));
}

/*!
 * \brief Convert tf::Vector3 into tf::Pose.
 * \param  t is translation is converted into a tf::Pose.
 * \return tf::Pose
 */
template<>
inline tf::Pose Convert<tf::Vector3, tf::Pose>(tf::Vector3 t) {
    return tf::Pose(tf::Quaternion(0.0, 0.0, 0.0, 1.0), t);
}
/*!
 * \brief Convert  tf::Pose into tf::Vector3.
 * \param  p is tf::Pose.
 * \return tf::Vector3
 */
template<>
inline tf::Vector3 Convert<tf::Pose, tf::Vector3>(tf::Pose t) {
    return t.getOrigin();
}


/*!
 * \brief CreateRPYPose taks array of double and create a tf::Pose.
 * \param ds is a  std array of 6 doubles to create pose (rpy + xyz).
 * \return tf::Pose
 * NOTE:
 *   @brief Constructor from Euler angles
   * @param yaw Angle around Y unless TF_EULER_DEFAULT_ZYX defined then Z
   * @param pitch Angle around X unless TF_EULER_DEFAULT_ZYX defined then Y
   * @param roll Angle around Z unless TF_EULER_DEFAULT_ZYX defined then X
   * tf::Quaternion(const tfScalar& yaw, const tfScalar& pitch, const tfScalar& roll) __attribute__((deprecated))

 */
inline tf::Pose CreateRPYPose(std::vector<double> ds) {
    assert(ds.size() > 5);
    return tf::Pose(tf::Quaternion(ds[5], ds[4], ds[3]), tf::Vector3(ds[0], ds[1], ds[2]));
}

inline tf::Quaternion Rpy2Quaternion(double roll, double pitch, double yaw)
{
    return tf::Quaternion(yaw, pitch, yaw);
}

/*!
 * \brief Create Pose from a axis and angle rotation representation.
 * \param axis is the unit vector to rotation around.
 * \param angle is the angle of rotation in radians.
 * \return tf::Pose
 */
inline tf::Pose CreatePose(tf::Vector3 axis, double angle) {
    return tf::Pose(tf::Quaternion(axis, angle), tf::Vector3(0.0, 0.0, 0.0));
}




/*!
 * \brief Create Quaternion from a rpy rotation representation designated in radians.
 * \param roll rotation around x axis in radians.
 * \param pitch rotation around y axis in radians.
 * \param yaw rotation around z axis in radians.
 * \return tf::Quaternion
 */
inline tf::Quaternion RPYRadians(double roll, double pitch, double yaw) {
    return tf::Quaternion(yaw, pitch, roll);
}

/*!
 * \brief Create Quaternion from a rpy rotation representation designated in degrees.
 * \param roll rotation around x axis in degrees.
 * \param pitch rotation around y axis in degrees.
 * \param yaw rotation around z axis in degrees.
 * \return tf::Quaternion
 */
inline tf::Quaternion RPYDegrees(double roll, double pitch, double yaw) {
    return tf::Quaternion(Deg2Rad(yaw), Deg2Rad(pitch), Deg2Rad(roll));
}







/*!
 * \brief Create Identity Pose
 * \return tf::Pose
 */
inline tf::Pose Identity() {
    tf::Transform t;
    t.setIdentity();
    return t;
}
#if 0
// This is a repeat of conversion from affine3d to tf::pose

/*!
 * \brief Create Affine Pose matrix into tf Transform.
 * \param e is a Eigen affine3d matrix.
 * \return tf::Transform
 */
template<>
inline tf::Transform Convert<Eigen::Affine3d, tf::Transform> (Eigen::Affine3d e) {
    tf::Transform t;
    t.setOrigin(tf::Vector3(e.matrix()(0, 3), e.matrix()(1, 3), e.matrix()(2, 3)));
    t.setBasis(tf::Matrix3x3(e.matrix()(0, 0), e.matrix()(0, 1), e.matrix()(0, 2),
                             e.matrix()(1, 0), e.matrix()(1, 1), e.matrix()(1, 2),
                             e.matrix()(2, 0), e.matrix()(2, 1), e.matrix()(2, 2)));
}
#endif


// geometry_msgs - constructor nightmare.

/*!
 * \brief Convert tf::Pose pose into an  geometry_msgs::Pose pose.
 * \param m is a tf::Pose transform matrix.
 * \return  geometry_msgs::Pose pose
 */
template<>
inline geometry_msgs::Pose Convert< tf::Pose, geometry_msgs::Pose> (tf::Pose m) {
    geometry_msgs::Pose p;
    p.position.x = m.getOrigin().x();
    p.position.y = m.getOrigin().y();
    p.position.z = m.getOrigin().z();
    p.orientation.x = m.getRotation().x();
    p.orientation.y = m.getRotation().y();
    p.orientation.z = m.getRotation().z();
    p.orientation.w = m.getRotation().w();
    return p;
}

/*!
 * \brief Convert geometry_msgs::Point point into an  geometry_msgs::Pose pose.
 * \param point geometry_msgs::Point is translation.
 * \return  geometry_msgs::Pose pose.
 */
template<>
inline geometry_msgs::Pose Convert<geometry_msgs::Point, geometry_msgs::Pose>(geometry_msgs::Point point) {
    geometry_msgs::Pose shared_pose_msg_;
    shared_pose_msg_.orientation.x = 0.0;
    shared_pose_msg_.orientation.y = 0.0;
    shared_pose_msg_.orientation.z = 0.0;
    shared_pose_msg_.orientation.w = 1.0;
    shared_pose_msg_.position = point;
    return shared_pose_msg_;
}


/*!
 * \brief Convert tf::Vector3 vector  translation into an  geometry_msgs::Point.
 * \param point is translation  defined as a tf::Vector3 vector.
 * \return  geometry_msgs::Point vector
 */
template<>
inline geometry_msgs::Vector3 Convert<tf::Vector3, geometry_msgs::Vector3>(tf::Vector3 point) {
    geometry_msgs::Vector3 v;
    v.x = point.x();
    v.y = point.y();
    v.z = point.z();
    return v;
}
template<>
inline geometry_msgs::Point Convert<tf::Vector3, geometry_msgs::Point>(tf::Vector3 point) {
    geometry_msgs::Point pt;
    pt.x = point.x();
    pt.y = point.y();
    pt.z = point.z();
    return pt;
}

/*!
 * \brief Convert Eigen::Vector3d translation into an  geometry_msgs Vector3.
 * \param vector is defined as a Eigen::Vector3d.
 * \return  geometry_msgs :Vector3e
 */
template<>
inline geometry_msgs::Vector3 Convert<double, geometry_msgs::Vector3 >(double scale) {
    geometry_msgs::Vector3 v;
    v.x=scale; v.y=scale; v.z=scale;
    return v;
}


// URDF



/*!
 * \brief Convert urdf::Vector into an tf vector.
 * \param v is a urdf::Vector3t.
 * \return  tf::Vector3 vector.
 */
template<>
inline tf::Vector3 Convert<urdf::Vector3, tf::Vector3> (urdf::Vector3 v) {
    return tf::Vector3(v.x, v.y, v.z);
}


// CRCL
/*!
 * \brief Convert array of std::vector<double> doubles into an JointState position, but blanking velcity, and effort.
 * \param src is a std::vector of doubles defining the value for each joint.
 * \return  sensor_msgs::JointState_<std::allocator<void> > definition.
 */
template<>
inline sensor_msgs::JointState Convert<std::vector<double>, sensor_msgs::JointState>(std::vector<double> src) {
    sensor_msgs::JointState joints;
    joints.position = src;
    joints.velocity.resize(src.size(), 0.0);
    joints.effort.resize(src.size(), 0.0);
    return joints;
}

//////////////////////////////////////////////////////////////////////////////////////////



// KDL
#ifdef KDL

#include <kdl/frames_io.hpp>

// KDL
//http://docs.ros.org/kinetic/api/kdl_parser/html/kdl__parser_8cpp_source.html
template<>
inline KDL::Vector Convert<urdf::Vector3, KDL::Vector>(urdf::Vector3 v) {
    return KDL::Vector(v.x, v.y, v.z);
}

template<>
inline KDL::Rotation Convert<urdf::Rotation, KDL::Rotation >(urdf::Rotation r) {
    return KDL::Rotation::Quaternion(r.x, r.y, r.z, r.w);
}

template<>
inline KDL::Frame Convert<urdf::Pose, KDL::Frame>(urdf::Pose p) {
    return KDL::Frame(
                Convert < urdf::Rotation, KDL::Rotation > (p.rotation),
                Convert<urdf::Vector3, KDL::Vector>(p.position)
                );
}
template<>
inline tf::Pose  Convert<KDL::Frame, tf::Pose >(KDL::Frame k)
{
    tf::Pose m;

    m.getOrigin().setX(k.p[0]);
    m.getOrigin().setY(k.p[1]);
    m.getOrigin().setZ(k.p[2]);

    double x,y,z,w;
    k.M.GetQuaternion(x,y,z,w);
    m.setRotation(tf::Quaternion(x,y,z,w));
    return m;
}
template<>
inline KDL::Frame Convert<tf::Pose, KDL::Frame >(tf::Pose m)
{
    KDL::Frame k;
    k.p[0] = m.getOrigin().x();
    k.p[1] = m.getOrigin().y();
    k.p[2] = m.getOrigin().z();

    k.M = KDL::Rotation::Quaternion( m.getRotation().x(), m.getRotation().y(), m.getRotation().z(),m.getRotation().w());
    return k;
}
template<>
inline KDL::JntArray  Convert<std::vector<double>, KDL::JntArray >(std::vector<double> joints)
{
    KDL::JntArray  joint_list(joints.size());

    // Fill in KDL joint list
    for(size_t i=0; i< joints.size(); i++)
        joint_list(i)=joints[i];
    return joint_list;
}
template<>
inline  std::vector<double> Convert<KDL::JntArray,std::vector<double> >(KDL::JntArray joint_list)
{
    std::vector<double>  joints;

    // Fill in KDL joint list
    for(size_t i=0; i< joint_list.data.size(); i++)
        joints.push_back(joint_list(i));

    return joints;
}

#endif

// Eigen
#ifdef EIGEN
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <tf_conversions/tf_eigen.h>

/*!
 * \brief Convert Eigen::Affine3d into tf::Pose.
 * \param pose is copy constructor of Eigen::Affine3d.
 * \return tf::Pose
 */
template<>
inline tf::Pose Convert<Eigen::Affine3d, tf::Pose>(Eigen::Affine3d pose) {
#if 1
    Eigen::Quaterniond q(pose.rotation());
    return tf::Pose(tf::Quaternion(q.x(), q.y(), q.z(), q.w()),
                    tf::Vector3(pose.translation().x(), pose.translation().y(), pose.translation().z()));
#else
    tf::Pose p;
    p.getOrigin().setX(pose.translation().x());
    p.getOrigin().setY(pose.translation().y());
    p.getOrigin().setZ(pose.translation().z());

    Eigen::Quaterniond q(pose.rotation());
    tf::Quaternion qtf(q.x(), q.y(), q.z(), q.w());
    p.setRotation(qtf);
    return p;
#endif
}
/*!
 * \brief Convert Eigen::Quaterniond into tf::Quaternion.
 * \param e is copy constructor of Eigen::Quaterniond.
 * \return tf::Quaternion
 */
template<>
inline tf::Quaternion Convert<Eigen::Quaterniond, tf::Quaternion> (Eigen::Quaterniond e) {
    return tf::Quaternion(e.x(), e.y(), e.z(), e.w());
}
/*!
 * \brief Convert Eigen::Matrix4d into tf::Pose.
 * \param  m is Eigen 4x4 matrix to be converted into a tf::Pose.
 * \return tf::Pose
 */
template<>
inline tf::Pose Convert<Eigen::Matrix4d, tf::Pose>(Eigen::Matrix4d m) {
    tf::Pose pose;
    Eigen::Vector3d trans(m.block<3, 1>(0, 3));
    Eigen::Quaterniond q(m.block<3, 3>(0, 0));
    pose.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w())); // pose.getRotation());
    pose.setOrigin(tf::Vector3(trans.x(), trans.y(), trans.z()));
    return pose;
}
/*!
 * \brief Convert geometry_msgs::Pose into tf::Vector3.
 * \param e is copy constructor of Eigen::Vector3d.
 * \return tf::Vector3
 */
template<>
inline tf::Vector3 Convert<Eigen::Vector3d, tf::Vector3> (Eigen::Vector3d e) {
    return tf::Vector3(e(0), e(1), e(2));
}

/*!
 * \brief Convert Eigen::Vector3d into tf::Pose.
 * \param e is copy constructor of Eigen::Vector3d.
 * \return tf::Pose
 */
template<>
inline tf::Pose Convert<Eigen::Vector3d, tf::Pose> (Eigen::Vector3d e) {
    return tf::Pose(tf::Quaternion(0.0, 0.0, 0.0, 1.0), tf::Vector3(e(0), e(1), e(2)));
}
/*!
 * \brief Convert Eigen matrix into tf::Vector3.
 * Example: tf::Vector3 v = matrixEigenToTfVector<Eigen::Matrix3d>(m);
 * \param e is copy constructor of Eigen Matrix, either 3x3, 4x4, double or float.
 * \return tf::Vector3
 */
template<typename T>
inline tf::Vector3 matrixEigenToTfVector(T e) {
    return tf::Vector3(e(0, 3), e(1, 3), e(2, 3));
}

/*!
 * \brief Convert Eigen Matrix3d into tf::Matrix3x3.
 * \param e is copy constructor of Eigen Matrix3d, a 3x3 double matrix.
 * \return tf::Matrix3x3
 */
template<>
inline tf::Matrix3x3 Convert<Eigen::Matrix3d, tf::Matrix3x3> (Eigen::Matrix3d e) {
    tf::Matrix3x3 t;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            t[i][j] = e(i, j);
    return t;
}

// Eigen

/*!
 * \brief Convert<tf::Pose, Eigen::Affine3d> converts tf pose into an  Eigen affine 4x4 matrix  o represent the pose
 * \param pose is the tf pose with position and orientation.
 * \return   Eigen Affine3d pose
 */
template<>
inline Eigen::Affine3d Convert<tf::Pose, Eigen::Affine3d>(tf::Pose pose) {
    Eigen::Quaterniond q(pose.getRotation().w(), pose.getRotation().x(), pose.getRotation().y(), pose.getRotation().z());
    return Eigen::Affine3d(Eigen::Translation3d(pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z()) * q.toRotationMatrix());
}

/*!
 * \brief Convert Eigen::Quaternion into an  Eigen affine 4x4 matrix  o represent the pose
 * \param q is the Eigen::Quaternion orientation.
 * \return   Eigen Affine3d pose
 */
template<>
inline Eigen::Affine3d Convert<Eigen::Quaternion<double>, Eigen::Affine3d> (Eigen::Quaternion<double> q) {
    return Eigen::Affine3d::Identity() * q;
}

/*!
 * \brief Convert geometry_msgs::Pose into an  Eigen affine3d 4x4 matrix  o represent the pose.
 * Uses tf conversion utilities.
 * \param m is defined as a geometry_msgs::Pose..
 * \return  Eigen Affine3d pose
 */
template<>
inline Eigen::Affine3d Convert<geometry_msgs::Pose, Eigen::Affine3d> (geometry_msgs::Pose m) {
    Eigen::Affine3d e = Eigen::Translation3d(m.position.x,
                                             m.position.y,
                                             m.position.z) *
            Eigen::Quaternion<double>(m.orientation.w,
                                      m.orientation.x,
                                      m.orientation.y,
                                      m.orientation.z);
    return e;
}
/*!
 * \brief Convert Eigen::Vector3d into std::vector<double>
 * \param e Eigen::Vector3d.
 * \return vector of 3 doubles
 */
template<>
inline std::vector<double> Convert<Eigen::Vector3d, std::vector<double>>(Eigen::Vector3d e) {
    std::vector<double> ds;
    ds.push_back(e.x());
    ds.push_back(e.y());
    ds.push_back(e.z());
    return ds;
}
/*!
 * \brief Convert tf::Quaternion into an  Eigen::Quaterniond.
 * \param q is defined as a tf::Quaternion..
 * \return  Eigen::Quaterniond vector
 */
template<>
inline Eigen::Quaterniond Convert<tf::Quaternion, Eigen::Quaterniond>(tf::Quaternion q) {
    return Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z());
}

/*!
 * \brief Convert geometry_msgs::Pose into an  Eigen::Translation3d.
 * \param pose is defined as a geometry_msgs::Pose..
 * \return  Eigen::Translation3d vector
 */
template<>
inline Eigen::Translation3d Convert<geometry_msgs::Pose, Eigen::Translation3d>(geometry_msgs::Pose pose) {
    return Eigen::Translation3d(pose.position.x, pose.position.y, pose.position.z);

}

/*!
 * \brief Convert tf::Vector3 into an  Eigen::Vector3d.
 * \param t is translation is defined as a tf::Vector3..
 * \return  Eigen::Vector3d vector
 */
template<>
inline Eigen::Vector3d Convert< tf::Vector3, Eigen::Vector3d>(tf::Vector3 t) {
    return Eigen::Vector3d(t[0], t[1], t[2]);
}

/*!
 * \brief Convert Eigen::Vector3d translation into an  Eigen::Affine3d pose.
 * \param translation is defined as a Eigen::Vector3d.
 * \return  Eigen::Affine3d pose
 */
template<>
inline Eigen::Affine3d Convert<Eigen::Vector3d, Eigen::Affine3d >(Eigen::Vector3d translation) {
    Eigen::Affine3d shared_pose_eigen_ = Eigen::Affine3d::Identity();
    shared_pose_eigen_.translation() = translation;
    return shared_pose_eigen_;
}

/*!
 * \brief Create Eigen::Affine3d as an axis angle definition around z axis.
 * \param zangle is angle of rotation in radians around Z.
 * \return  Eigen::Affine3d pose
 */
inline Eigen::Affine3d CreateEigenPose(double zangle) {
    return Eigen::Affine3d::Identity() * Eigen::AngleAxisd(zangle, Eigen::Vector3d::UnitZ());
}

/*!
 * \brief Convert Eigen::Translation3d translation into an  Eigen::Affine3d pose.
 * \param t is translation  defined as a Eigen::Translation3d.
 * \return  Eigen::Affine3d pose
 */
template<>
inline Eigen::Affine3d Convert<Eigen::Translation3d, Eigen::Affine3d>(Eigen::Translation3d trans) {
    return Eigen::Affine3d::Identity() * trans;
}

/*!
 * \brief Convert geometry_msgs::Point translation into an  Eigen::Vector3d vector.
 * \param point is translation  defined as a geometry_msgs::Point.
 * \return  Eigen::Vector3d vector
 */
template<>
inline Eigen::Vector3d Convert<geometry_msgs::Point, Eigen::Vector3d>(geometry_msgs::Point point) {
    return Eigen::Vector3d(point.x, point.y, point.z);
}

/*!
 * \brief Convert geometry_msgs::Point translation into an  Eigen::Affine3d pose.
 * \param point is translation  defined as a geometry_msgs::Point.
 * \return  Eigen::Affine3d pose
 */
template<>
inline Eigen::Affine3d Convert<>(geometry_msgs::Point point) {
    return Eigen::Affine3d::Identity() * Eigen::Translation3d(point.x, point.y, point.z);
}

/*!
 * \brief Convert Eigen::Affine3d  into an  Eigen::Vector3d vector.
 * \param e is  pose defined as a Eigen Affine3d.
 * \return  Eigen::Vector3d vector
 */
template<>
inline Eigen::Vector3d Convert<Eigen::Affine3d, Eigen::Vector3d>(Eigen::Affine3d e) {
    return Eigen::Vector3d(e.matrix()(0, 3), e.matrix()(1, 3), e.matrix()(2, 3));
}
/*!
 * \brief Convert Eigen::Vector3d point into an geometry_msgs::Point position vector.
 * \param point Eigen::Vector3d is translation.
 * \return  geometry_msgs::Point position vector.
 */
template<>
inline geometry_msgs::Point Convert<Eigen::Vector3d, geometry_msgs::Point>(Eigen::Vector3d point) {
    geometry_msgs::Point pt;
    pt.x = point.x();
    pt.y = point.y();
    pt.z = point.z();
    return pt;
}
/*!
 * \brief Convert Eigen::Vector3d translation into an  geometry_msgs Vector3.
 * \param vector is defined as a Eigen::Vector3d.
 * \return  geometry_msgs :Vector3e
 */
template<>
inline geometry_msgs::Vector3 Convert<Eigen::Vector3d, geometry_msgs::Vector3 >(Eigen::Vector3d e) {
    geometry_msgs::Vector3 v;
    v.x=e.x(); v.y=e.y(); v.z=e.z();
    return v;
}
/*!
 * \brief Convert Eigen::Affine3d pose into an geometry_msgs::Pose pose.
 * \param e is Eigen::Affine3d defining equivalent pose.
 * \return  geometry_msgs::Pose pose.
 */
template<>
inline geometry_msgs::Pose Convert <Eigen::Affine3d, geometry_msgs::Pose> (Eigen::Affine3d e) {
    geometry_msgs::Pose m;
    m.position.x = e.translation()[0];
    m.position.y = e.translation()[1];
    m.position.z = e.translation()[2];
    Eigen::Quaterniond q = (Eigen::Quaterniond)e.linear();
    m.orientation.x = q.x();
    m.orientation.y = q.y();
    m.orientation.z = q.z();
    m.orientation.w = q.w();
    if (m.orientation.w < 0) {
        m.orientation.x *= -1;
        m.orientation.y *= -1;
        m.orientation.z *= -1;
        m.orientation.w *= -1;
    }
    return m;
}

/*!
 * \brief Convert Eigen::Affine3d pose into an geometry_msgs::Point translation element.
 * \param e is Eigen::Affine3d defining pose.
 * \return  geometry_msgs::Point translation element.
 */
template<>
inline geometry_msgs::Point Convert<Eigen::Affine3d, geometry_msgs::Point> (Eigen::Affine3d pose) {
    geometry_msgs::Pose msg = Convert <Eigen::Affine3d, geometry_msgs::Pose> (pose);
    //tf::poseEigenToMsg(pose, shared_pose_msg_);
    return msg.position;
}

/*!
 * \brief Convert urdf::Vector into an Eigen vector.
 * \param v is a urdf::Vector3t.
 * \return  Eigen::Vector3d vector.
 */
template<>
inline Eigen::Vector3d Convert<urdf::Vector3, Eigen::Vector3d> (urdf::Vector3 v) {
    return Eigen::Vector3d(v.x, v.y, v.z);
}

/*!
 * \brief Convert urdf::Pose into an Eigen affine3d.
 * \param pose is a urdf::Pose.
 * \return  Eigen::Affine3d pose.
 */
template<>
inline Eigen::Affine3d Convert<urdf::Pose, Eigen::Affine3d>(urdf::Pose pose) {
    // http://answers.ros.org/question/193286/some-precise-definition-or-urdfs-originrpy-attribute/
    Eigen::Quaterniond q(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);
    Eigen::Affine3d af(Eigen::Translation3d(pose.position.x, pose.position.y, pose.position.z) * q.toRotationMatrix());
    return af;
}

template<>
inline Eigen::VectorXd Convert<std::vector<double>, Eigen::VectorXd>(std::vector<double> v) {
    Eigen::VectorXd p(v.size());
    for (size_t i = 0; i < v.size(); i++)
        p(i) = v[i];
    return p;
}

template<>
inline std::vector<double> Convert<Eigen::VectorXd, std::vector<double>>(Eigen::VectorXd ev) {
    std::vector<double> v;
    for (int i = 0; i < ev.size(); i++)
        v.push_back(ev(i));
    return v;
}
#endif

}
#endif
