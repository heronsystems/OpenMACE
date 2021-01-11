#include "rotation_2D.h"
#include "rotation_3D.h"

using namespace Eigen;

namespace mace {
namespace pose {

Rotation_3D::Rotation_3D(const std::string &name):
    AbstractRotation(3, name)
{
    this->name = name;
    this->m_QRotation.setIdentity();
}

Rotation_3D::~Rotation_3D()
{

}

Rotation_3D::Rotation_3D(const Rotation_3D &copy):
    AbstractRotation(copy)
{
    this->m_QRotation.setIdentity();
    this->m_QRotation = copy.m_QRotation;
}

Rotation_3D::Rotation_3D(const Rotation_2D &copy):
    AbstractRotation (3,copy.getObjectName())
{
    double yawRotation = copy.getPhi();
    this->updateYaw(yawRotation);
}

Rotation_3D::Rotation_3D(const double &roll, const double &pitch, const double &yaw, const std::string &name):
    AbstractRotation(3, name)
{
    this->m_QRotation.setIdentity();
    this->updateFromEuler(roll,pitch,yaw);
}

/** Set functions for all rotation angle information **/

void Rotation_3D::updateFromEuler(const double &roll, const double &pitch, const double &yaw)
{
    this->m_QRotation = Eigen::AngleAxisd(yaw, Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Vector3d::UnitX());
}

void Rotation_3D::setQuaternion(const Eigen::Quaterniond &rotation)
{
    this->m_QRotation = rotation;
}

Eigen::Quaterniond Rotation_3D::getQuaternion() const
{
    return this->m_QRotation;
}

void Rotation_3D::setQuaternion(const Eigen::Matrix3d &rotation)
{
    this->m_QRotation = Eigen::Quaterniond(rotation);
}

void Rotation_3D::updateRoll(const double &angle)
{
    EulerAngleRotation currentRotation = EulerAngleRotation::FromRotation<true, false, false>(this->m_QRotation.toRotationMatrix());
    currentRotation.gamma() = angle;
    this->m_QRotation = Eigen::AngleAxisd(currentRotation.alpha(), Vector3d::UnitZ()) *
            Eigen::AngleAxisd(currentRotation.beta(), Vector3d::UnitY()) *
            Eigen::AngleAxisd(currentRotation.gamma(), Vector3d::UnitX());
}

void Rotation_3D::updatePitch(const double &angle)
{
    EulerAngleRotation currentRotation = EulerAngleRotation::FromRotation<true, false, false>(this->m_QRotation);
    currentRotation.beta() = angle;
    this->m_QRotation = Eigen::AngleAxisd(currentRotation.alpha(), Vector3d::UnitZ()) *
            Eigen::AngleAxisd(currentRotation.beta(), Vector3d::UnitY()) *
            Eigen::AngleAxisd(currentRotation.gamma(), Vector3d::UnitX());}


void Rotation_3D::updateYaw(const double &angle)
{
    EulerAngleRotation currentRotation = EulerAngleRotation::FromRotation<true, false, false>(this->m_QRotation.toRotationMatrix());
    currentRotation.alpha() = angle;
    this->m_QRotation = Eigen::AngleAxisd(currentRotation.alpha(), Vector3d::UnitZ()) *
            Eigen::AngleAxisd(currentRotation.beta(), Vector3d::UnitY()) *
            Eigen::AngleAxisd(currentRotation.gamma(), Vector3d::UnitX());}


/** Get functions for all rotation angle information **/

void Rotation_3D::getDiscreteEuler(double &roll, double &pitch, double &yaw) const
{
    double sqx = m_QRotation.x()*m_QRotation.x();
    double sqy = m_QRotation.y()*m_QRotation.y();
    double sqz = m_QRotation.z()*m_QRotation.z();
    double sqw = m_QRotation.w()*m_QRotation.w();

    // This is to calculate whether or not the quaternion is normalized. If it
    // is this answer is one, but if not it becomes a correction factor.
    double unit = sqx + sqy + sqz + sqw;

    // This value is to test for the singularity that occurs when the pitch is
    // at +/-90 degrees.  So check when this value is +/-0.5.
    double test = m_QRotation.y()*m_QRotation.w() - m_QRotation.x()*m_QRotation.z();

    if( test > 0.499*unit )
    {
        yaw 	= -2.0 * atan2( m_QRotation.x(), m_QRotation.w() );
        pitch 	= M_PI / 2.0;
        roll     = 0.0;
    }
    else if (test < -0.499*unit )
    {
        yaw     = 2.0 * atan2( m_QRotation.x(), m_QRotation.w() );
        pitch	= -M_PI/2.0;
        roll     = 0.0;
    }
    else {
//        roll   = atan2(2.0*(m_QRotation.y()*m_QRotation.z() + m_QRotation.w()*m_QRotation.x()), sqw - sqx - sqy - sqz);
        roll   = atan2(2.0*(m_QRotation.y()*m_QRotation.z() + m_QRotation.w()*m_QRotation.x()), sqw - sqx - sqy + sqz);

        pitch = -asin(2.0 * (m_QRotation.x()*m_QRotation.z() - m_QRotation.w()*m_QRotation.y()));
        yaw = atan2(2.0*(m_QRotation.x()*m_QRotation.y() + m_QRotation.w()*m_QRotation.z()), sqw + sqx - sqy - sqz);
    }


}

EulerAngleRotation Rotation_3D::getEulerRotation() const
{
    return EulerAngleRotation::FromRotation<true, false, false>(this->m_QRotation.toRotationMatrix());
}

Eigen::Matrix3d Rotation_3D::getRotationMatrix() const
{
    return this->m_QRotation.toRotationMatrix();
}

double Rotation_3D::getRoll() const
{
    double roll, pitch, yaw;
    getDiscreteEuler(roll,pitch,yaw);
    return roll;
}

double Rotation_3D::getPitch() const
{
    double roll, pitch, yaw;
    getDiscreteEuler(roll,pitch,yaw);
    return pitch;
}

double Rotation_3D::getYaw() const
{
    double roll, pitch, yaw;
    getDiscreteEuler(roll,pitch,yaw);
    return yaw;
}

mavlink_attitude_quaternion_t Rotation_3D::getMACEQuaternion() const
{
    mavlink_attitude_quaternion_t quat;

    return quat;
}

mavlink_attitude_t Rotation_3D::getMACEEuler() const
{
    double roll = 0.0, pitch = 0.0, yaw = 0.0;

    this->getDiscreteEuler(roll,pitch,yaw);
    mavlink_attitude_t euler;
    euler.roll = static_cast<float>(roll);
    euler.pitch = static_cast<float>(pitch);
    euler.yaw = static_cast<float>(yaw);

    return euler;
}

mavlink_message_t Rotation_3D::getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const
{
    mavlink_message_t msg;
    mavlink_attitude_t attitude = getMACEEuler();
    mavlink_msg_attitude_encode_chan(systemID,compID,chan,&msg,&attitude);
    return msg;
}


} //end of namespace pose
} //end of namespace mace
