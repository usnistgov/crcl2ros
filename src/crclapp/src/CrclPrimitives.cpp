// CrclPrimitives.cpp

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
#include "crclapp/CrclPrimitives.h"
#include "crclapp/CrclPublisherInterface.h"

#define ROWMAJOR

using namespace RCS;

// http://wiki.ros.org/APIs
// http://docs.ros.org/api/tf/html/c++/annotated.html
namespace crcl {
static const char *sStateEnums[] = {"CRCL_Done", "CRCL_Error", "CRCL_Working", "CRCL_Ready"};

bool GetRPY(tf::Vector3 Xrot, tf::Vector3 Zrot, double & dRoll, double & dPitch, double & dYaw);
PoseType NullPose();
crcl::VectorType VectorZero();
crcl::PoseType IdentityPose();

////////////////////////////////////////////////////////////////////////////////
inline tf::Vector3 GetVector3D(crcl::VectorType & vector) {
    return tf::Vector3(vector.I(), vector.J(), vector.K());
}

////////////////////////////////////////////////////////////////////////////////
inline tf::Vector3 GetVector3D(crcl::PointType & point) {
    return tf::Vector3(point.X(), point.Y(), point.Z());
}

////////////////////////////////////////////////////////////////////////////////
inline tf::Vector3 GetTfVector(crcl::PointType & point) {
    return tf::Vector3(point.X(), point.Y(), point.Z());
}

////////////////////////////////////////////////////////////////////////////////
inline tf::Vector3 GetTfVector(crcl::VectorType & vector) {
    return tf::Vector3(vector.I(), vector.J(), vector.K());
}

////////////////////////////////////////////////////////////////////////////////
inline ::PointType GetPoint(tf::Vector3 & point) {
    return ::PointType(point.x(), point.y(), point.z());
}

////////////////////////////////////////////////////////////////////////////////
inline ::VectorType GetVector(tf::Vector3 & point) {
    return ::VectorType(point.x(), point.y(), point.z());
}

////////////////////////////////////////////////////////////////////////////////
tf::Matrix3x3 GetTfRotMatrix(tf::Vector3 Xrot, tf::Vector3 Zrot) {

    tf::Vector3 YRot = Zrot.cross(Xrot);

    tf::Matrix3x3 m(Xrot.getX(), YRot.getX(), Zrot.getX(),
                    Xrot.getY(), YRot.getY(), Zrot.getY(),
                    Xrot.getZ(), YRot.getZ(), Zrot.getZ());
    return m;
}

////////////////////////////////////////////////////////////////////////////////
bool GetRPY(tf::Vector3 Xrot, tf::Vector3 Zrot, double &roll, double &pitch, double &yaw) {
    tf::Matrix3x3 m = GetTfRotMatrix(Xrot, Zrot);
    m.getRPY(roll, pitch, yaw);
    return true;

}

////////////////////////////////////////////////////////////////////////////////
tf::Pose Convert(crcl::PoseType & pose, double lengthConversion) {
    tf::Pose p;
    p.setOrigin(GetTfVector(pose.Point()) * lengthConversion);
    p.setBasis(GetTfRotMatrix(GetVector3D(pose.XAxis()), GetVector3D(pose.ZAxis())));
    return p;
}

////////////////////////////////////////////////////////////////////////////////
RCS::PoseTolerance Convert(crcl::PoseToleranceType tolerance)
{
    RCS::PoseTolerance tol;
    double x = tolerance.XPointTolerance().present()? tolerance.XPointTolerance().get(): 0.0;
    double y = tolerance.YPointTolerance().present()? tolerance.YPointTolerance().get(): 0.0;
    double z = tolerance.ZPointTolerance().present()? tolerance.ZPointTolerance().get(): 0.0;

    // So what does nit mean
    double xrot = tolerance.XAxisTolerance().present()? tolerance.XAxisTolerance().get() : 0.0;
    double zrot = tolerance.ZAxisTolerance().present()? tolerance.ZAxisTolerance().get() : 0.0;

    // Assign permissible error pose tolerance
    //    tolpose.setOrigin(tf::Vector3(x,y,z));
    //    tolpose.setRotation(tf::Quaternion(0,0,0,1));  // x,y,z,w
    tol[RCS::PoseTolerance::POSITION]=std::min(x,std::min(y,z));
    tol[RCS::PoseTolerance::ORIENTATION]=std::min(xrot,zrot);

    return tol;
}


////////////////////////////////////////////////////////////////////////////////
inline crcl::VectorType VectorZero() {
    return crcl::VectorType(0.0, 0.0, 0.0);
}


////////////////////////////////////////////////////////////////////////////////
bool GetPoseToRPY(crcl::PoseType & pose, double & dRoll, double & dPitch, double & dYaw) {
    return GetRPY(GetVector3D(pose.XAxis()), GetVector3D(pose.ZAxis()), dRoll, dPitch, dYaw);
}

////////////////////////////////////////////////////////////////////////////////
tf::Quaternion Convert(tf::Vector3 Xrot, tf::Vector3 Zrot) {
    double roll, pitch, yaw;
    GetRPY(Xrot, Zrot, roll, pitch, yaw);
    tf::Quaternion rotation;
    rotation.setRPY(roll, pitch, yaw);
    return rotation;
}

////////////////////////////////////////////////////////////////////////////////
crcl::PoseType Convert(tf::Pose pose) {
    crcl::PoseType crclPose(NullPose());
    crclPose.Point() = GetPoint(pose.getOrigin());
#if 0
    crclPose.Point().X() = pose.getOrigin().x();
    crclPose.Point().Y() = pose.getOrigin().y();
    crclPose.Point().Z() = pose.getOrigin().z();
#endif
    tf::Matrix3x3 m = pose.getBasis();
    tf::Vector3 v1, v2, v3;

    v1 = m.getColumn(0);
    v2 = m.getColumn(1);
    v3 = m.getColumn(2);
    crclPose.XAxis(GetVector(v1));
    crclPose.ZAxis(GetVector(v3));

    return crclPose;
}

////////////////////////////////////////////////////////////////////////////////
std::string DumpCrclJoints(crcl::JointStatusSequence jin) {
    std::stringstream str;
    for (unsigned int i = 0; i < jin.size(); i++) {
        double pos = jin[i].JointPosition().present() ? jin[i].JointPosition().get() : nan("");
        str << " [" << i << "]=" << pos;
    }
    return str.str();
}

////////////////////////////////////////////////////////////////////////////////
sensor_msgs::JointState Convert(crcl::JointStatusSequence jout, double angleConversion) {
    sensor_msgs::JointState joint;
    for (unsigned int i = 0; i < jout.size(); i++)
    {
        //::JointStatusType  jointstatus(jout[i ].JointNumber());
        double pos = jout[i].JointPosition().present() ? jout[i].JointPosition().get(): 0.0;

        joint.position.push_back(pos); // *(jout[i].JointPosition()) * angleConversion);

        // FIXME: how do you determine if there are values for velocity and force/torque
        //joint.velocity.push_back(*(jout[i].JointVelocity()));
        joint.effort.push_back(0.0);
    }
    return joint;
}

////////////////////////////////////////////////////////////////////////////////
crcl::JointStatusSequence Convert(sensor_msgs::JointState joints, double _angleConversion) {
    JointStatusSequence jout;

    for (unsigned int i = 0; i < joints.position.size(); i++) {
        ::JointStatusType jointstatus(i + 1);
        jointstatus.JointPosition(joints.position[i] * _angleConversion);
        jout.push_back(jointstatus);
    }
    return jout;
}

////////////////////////////////////////////////////////////////////////////////
JointStatusSequence Convert(crcl::ActuatorJointSequence joints, double _angleConversion) {
    JointStatusSequence jout;

    for (unsigned int i = 0; i < joints.size(); i++) {
        ::ActuateJointType & jointcmd(joints[i]);
        unsigned long long n = jointcmd.JointNumber();
        ::JointStatusType jointstatus(jointcmd.JointNumber());
        jointstatus.JointPosition(jointcmd.JointPosition() * _angleConversion);
        jout.push_back(jointstatus);
    }
    return jout;
}

////////////////////////////////////////////////////////////////////////////////
crcl::PoseType NullPose() {
    crcl::PointType point(0.0, 0.0, 0.0);
    crcl::VectorType xrot(1.0, 0.0, 0.0);
    crcl::VectorType zrot(0.0, 0.0, 1.0);

    return crcl::PoseType(point, xrot, zrot);
}

////////////////////////////////////////////////////////////////////////////////
crcl::PoseType IdentityPose() {
    crcl::PointType point(0.0, 0.0, 0.0);
    crcl::VectorType xrot(1.0, 0.0, 0.0);
    crcl::VectorType zrot(0.0, 0.0, 1.0);

    return crcl::PoseType(point, xrot, zrot);
}

////////////////////////////////////////////////////////////////////////////////
crcl::PoseType PoseHome() {
    static double home[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    std::vector<double> h;
    std::copy(&home[0], &home[6], std::back_inserter(h));
    tf::Pose pose;
    // maybe hard code this
    pose.getOrigin() = tf::Vector3(0, 0, 0);
    pose.getRotation().setRPY(0, 0, 0);
    //return Convert(IkFastKinematics::JointsToPose(h) );
    return Convert(pose);
}

////////////////////////////////////////////////////////////////////////////////
std::vector<double> ConvertToAnglePositionVector(crcl::ActuatorJointSequence & joints, double dAngleConversion) {
    std::vector<double> angles;

    for (unsigned int i = 0; i < joints.size(); i++) {
        ::ActuateJointType & jointcmd(joints[i]);
        angles.push_back(jointcmd.JointPosition() * dAngleConversion);
    }
    return angles;
}
////////////////////////////////////////////////////////////////////////////////
std::string DumpRotationAsCrcl(tf::Pose rcspose, std::string separator) {
    crcl::PoseType pose = Convert(rcspose);
    return DumpRotationAsCrcl(pose);
}
////////////////////////////////////////////////////////////////////////////////
std::string DumpRotationAsCrcl(crcl::PoseType pose, std::string separator) {
    std::stringstream str;
    str << "XAxis = ";
    str << boost::format("%8.4f") % pose.XAxis().I() << separator.c_str();
    str << boost::format("%8.4f") % pose.XAxis().J() << separator.c_str();
    str << boost::format("%8.4f") % pose.XAxis().K() << separator.c_str();
    str << "\tZAxis = ";
    str << boost::format("%8.4f") % pose.ZAxis().I() << separator.c_str();
    str << boost::format("%8.4f") % pose.ZAxis().J() << separator.c_str();
    str << boost::format("%8.4f") % pose.ZAxis().K() ;
    return str.str();
}

////////////////////////////////////////////////////////////////////////////////
std::string DumpCrclCommand(::CRCLCommandType & crclCommand) {
    std::stringstream str;
    try {
        if (crclCommand.Name().present())
            str << *crclCommand.Name();
    } catch (...) {
    }
    str << "CommandID=" << crclCommand.CommandID() << std::endl;
    return str.str();
}

////////////////////////////////////////////////////////////////////////////////
std::string DumpStatusReply(CrclStatus *wm)
{
    std::stringstream str;
    //const char * sEnums[] = {"CRCL_Done", "CRCL_Error", "CRCL_Working", "CRCL_Ready"};
    str << "GetStatusReply=" << wm->CommandID() << ":";
    str << wm->CommandStatus() << std::endl;

    str << "Crcl Status Joints       =" << crcl::DumpCrclJoints(wm->_CurrentJoints).c_str()<< std::endl;
    str << "Crcl Status Position    ="  << crcl::DumpPosition(wm->_CurrentPose).c_str()<< std::endl;

    tf::Matrix3x3 m =  GetTfRotMatrix(GetVector3D(wm->_CurrentPose.XAxis()), GetVector3D(wm->_CurrentPose.ZAxis()));
    tf::Quaternion q;
    m.getRotation(q);
    str << "Crcl Status Quaternion  ="  << q.x() <<":" << q.y() <<":" << q.z() <<":" <<  q.w() <<  std::endl;
    str << "Crcl Status Orientation ="  << crcl::DumpRotationAsCrcl(wm->_CurrentPose).c_str()<< std::endl;

    double r,p,y;
    GetPoseToRPY(wm->_CurrentPose,r,p,y);
    str << "Crcl Status RPY         ="  << StrFormat("%8.4f,%8.4f,%8.4f",Rad2Deg(r),Rad2Deg(p),Rad2Deg(y)).c_str()<< std::endl;
    return str.str();
}
////////////////////////////////////////////////////////////////////////////////
std::string DumpPosition(crcl::PoseType pose, std::string separator) {
    std::stringstream str;
    str << boost::format("%8.4f") % pose.Point().X() << separator.c_str();
    str << boost::format("%8.4f") % pose.Point().Y() << separator.c_str();
    str << boost::format("%8.4f") % pose.Point().Z() ;
    return str.str();
}
////////////////////////////////////////////////////////////////////////////////
std::string DumpPose(crcl::PoseType pose, std::string separator) {
    std::stringstream str;
    str << boost::format("%8.4f") % pose.Point().X() << separator.c_str();
    str << boost::format("%8.4f") % pose.Point().Y() << separator.c_str();
    str << boost::format("%8.4f") % pose.Point().Z() << separator.c_str();
    str << boost::format("%8.4f") % pose.XAxis().I() << separator.c_str();
    str << boost::format("%8.4f") % pose.XAxis().J() << separator.c_str();
    str << boost::format("%8.4f") % pose.XAxis().K() << separator.c_str();
    str << boost::format("%8.4f") % pose.ZAxis().I() << separator.c_str();
    str << boost::format("%8.4f") % pose.ZAxis().J() << separator.c_str();
    str << boost::format("%8.4f") % pose.ZAxis().K() ;
    return str.str();
}

////////////////////////////////////////////////////////////////////////////////
CrclStatus::CrclStatus() :
    _CommandStatus("CRCL_Done"),
    _GoalPose(crcl::PoseHome()),
    _CurrentPose(crcl::PoseHome())
{
    _GoalJoints = JointsHome();
    _CurrentJoints = JointsHome();
    CommandID() = 1;
    StatusID() = 1;
    _lengthConversion = 1.0;

    // opposite of what documentation says are defaults
    _angleUnit = RCS::CanonAngleUnit::DEGREE;
    _angleConversion = M_PI / 180.0;

    _lengthUnit = RCS::CanonLengthUnit::METER;
    _lengthConversion = 1.0;

    _forceUnit = RCS::CanonForceUnit::NEWTON;
    _forceConversion = 1.0;

    _torqueUnit = RCS::CanonTorqueUnit::NEWTONMETER;
    _torqueConversion = 1.0;

    _bCoordinatedMotion = true;

    Rates().JointVelLimit().resize(8, DEFAULT_JOINT_MAX_VEL);
    Rates().JointAccLimit().resize(8, DEFAULT_JOINT_MAX_ACCEL);
}


////////////////////////////////////////////////////////////////////////////////
void CrclStatus::Update(WorldModel::CShape & inshape)
{
    if(!Globals.bReadAllInstances)
        return;
    // Multithreading locked thread copy
    WorldModel::CShape shape(inshape.name(), inshape.type(), inshape.centroid());
    {
        std::unique_lock<std::mutex> lock(WorldModel::shapemutex);
        shape=inshape;
    }

    std::string name = shape.name();
    tf::Pose pose = shape.centroid();
    _ModelStates[name]=pose;  // basic logical model

    std::vector<inference_type> inferences;

    // Gears are different than kit/vessels
    if(shape.isGear())
    {
        inference_type inference;
        inference.name = "parent";
        inference.parent=shape._attributes["parent"];
        inference.slot=shape._attributes["slot"];
        inferences.push_back(inference);
    }
    else
    {
        std::map<std::string, std::map<std::string, std::string>>::iterator it;
        for(it=shape._properties.begin(); it !=shape._properties.end(); ++it)
        {
            inference_type inference;
            inference.name = (*it).first;
            std::map<std::string, std::string> properties = (*it).second;
            inference.type = properties["type"];
            inference.location = properties["location"];
            inference.state = properties["state"];

            inferences.push_back(inference);
        }
    }
    // name shoud be of the shape, with a potential list of inferences
    _ModelInferences[name]=inferences;  // inferences from logical model
 }

////////////////////////////////////////////////////////////////////////////////
void CrclStatus::Update(std::string name, tf::Pose & pose)
{
    _ModelStates[name]=pose;
 }

////////////////////////////////////////////////////////////////////////////////
void CrclStatus::Update(sensor_msgs::JointState & joints)
{
    JointStatusSequence jout;

    for (unsigned int i = 0; i < joints.position.size(); i++) {

        ::JointStatusType jointstatus(i + 1);

        // Convert from radians to degrees?
        double dConverter = 1.0;

        if (_angleUnit == RCS::CanonAngleUnit::DEGREE) {
            dConverter = (double) (180.0 / M_PI);
        }
        jointstatus.JointPosition(dConverter * joints.position[i]);
        jout.push_back(jointstatus);
    }
    _CurrentJoints = _GoalJoints = jout;
}
// FIXME

////////////////////////////////////////////////////////////////////////////////
crcl::JointStatusSequence CrclStatus::JointsHome() {
    static double home[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    if (_GoalJoints.size() > 0) {
        _GoalJoints.clear();
    }

    for (int i = 0; i < 6; i++) {
        crcl::JointStatus j(i + 1);
        j.JointPosition(home[i]);
        j.JointVelocity(0.0);
        j.JointTorqueOrForce(0.0);
        _GoalJoints.push_back(j);
    }
    return _GoalJoints;
}

////////////////////////////////////////////////////////////////////////////////
void CrclStatus::Update(unsigned long long _CommandID) {
    StatusID() = CommandID() = _CommandID;
}

////////////////////////////////////////////////////////////////////////////////
void CrclStatus::Update(crcl::CommandStateEnum state) {
    CommandStatus() = state;
}

////////////////////////////////////////////////////////////////////////////////
void CrclStatus::Update(crcl::JointStatusSequence & joints, int type) {
    if (type == RCS::TrajPointType::WAYPOINT) {
        _CurrentJoints = joints;
    } else {
        _GoalJoints = joints;
    }
}

////////////////////////////////////////////////////////////////////////////////
void CrclStatus::Update(crcl::PoseType & pose, int type) {
    if (type == RCS::TrajPointType::WAYPOINT) {
        _GoalPose = pose;
        _CurrentPose = pose;
    } else {
        _CurrentPose = pose;
        _GoalPose = pose;
    }
}

////////////////////////////////////////////////////////////////////////////////
void CrclStatus::Update(tf::Pose & pose) {
    _CurrentPose = Convert(pose);
#if 0
    _CurrentPose.Point().X() = pose.getOrigin().x();
    _CurrentPose.Point().Y() = pose.getOrigin().y();
    _CurrentPose.Point().Z() = pose.getOrigin().z();
    boost::array<double, 12> m = _matrixFromQuat(pose.rotation);
    crcl::VectorType xunitv(crcl::VectorZero());
    xunitv.I(m[4 * 0 + 0]);
    xunitv.J(m[4 * 0 + 1]);
    xunitv.K(m[4 * 0 + 2]);
    _CurrentPose.XAxis(xunitv);

    crcl::VectorType zunitv(crcl::VectorZero());
    zunitv.I(m[4 * 2 + 0]);
    zunitv.J(m[4 * 2 + 1]);
    zunitv.K(m[4 * 2 + 2]);
    _CurrentPose.ZAxis(zunitv);
#endif
    _GoalPose = _CurrentPose;
}
}
