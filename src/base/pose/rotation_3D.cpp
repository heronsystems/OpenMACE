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
    EulerAngleRotation currentRotation = EulerAngleRotation::FromRotation<true, false, false>(this->m_QRotation);
    currentRotation.alpha() = yaw;
    currentRotation.beta() = pitch;
    currentRotation.gamma() = roll;

    this->m_QRotation = Eigen::AngleAxisd(currentRotation.alpha(), Vector3d::UnitZ()) * Eigen::AngleAxisd(currentRotation.beta(), Vector3d::UnitY()) * Eigen::AngleAxisd(currentRotation.gamma(), Vector3d::UnitX());
}

void Rotation_3D::setRotation(const Eigen::Quaterniond &rotation)
{
    this->m_QRotation = rotation;
}

void Rotation_3D::setRotation(const Eigen::Matrix3d &rotation)
{
    this->m_QRotation = Eigen::Quaterniond(rotation);
}

void Rotation_3D::updateRoll(const double &angle)
{
    EulerAngleRotation currentRotation = EulerAngleRotation::FromRotation<true, false, false>(this->m_QRotation);
    currentRotation.gamma() = angle;
    this->m_QRotation = Eigen::AngleAxisd(currentRotation.alpha(), Vector3d::UnitZ()) * Eigen::AngleAxisd(currentRotation.beta(), Vector3d::UnitY()) * Eigen::AngleAxisd(currentRotation.gamma(), Vector3d::UnitX());
}

void Rotation_3D::updatePitch(const double &angle)
{
    EulerAngleRotation currentRotation = EulerAngleRotation::FromRotation<true, false, false>(this->m_QRotation);
    currentRotation.beta() = angle;
    this->m_QRotation = Eigen::AngleAxisd(currentRotation.alpha(), Vector3d::UnitZ()) * Eigen::AngleAxisd(currentRotation.beta(), Vector3d::UnitY()) * Eigen::AngleAxisd(currentRotation.gamma(), Vector3d::UnitX());
}


void Rotation_3D::updateYaw(const double &angle)
{
    EulerAngleRotation currentRotation = EulerAngleRotation::FromRotation<true, false, false>(this->m_QRotation);
    currentRotation.alpha() = angle;
    this->m_QRotation = Eigen::AngleAxisd(currentRotation.alpha(), Vector3d::UnitZ()) * Eigen::AngleAxisd(currentRotation.beta(), Vector3d::UnitY()) * Eigen::AngleAxisd(currentRotation.gamma(), Vector3d::UnitX());
}


/** Get functions for all rotation angle information **/

void Rotation_3D::getDiscreteEuler(double &roll, double &pitch, double &yaw) const
{
    EulerAngleRotation currentRotation = EulerAngleRotation::FromRotation<false, false, false>(this->m_QRotation.matrix());
    roll = currentRotation.gamma();
    pitch = currentRotation.beta();
    yaw = currentRotation.alpha();
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
    EulerAngleRotation currentRotation = EulerAngleRotation::FromRotation<true, false, false>(this->m_QRotation);
    return currentRotation.gamma();
}

double Rotation_3D::getPitch() const
{
    EulerAngleRotation currentRotation = EulerAngleRotation::FromRotation<true, false, false>(this->m_QRotation);
    return currentRotation.beta();
}

double Rotation_3D::getYaw() const
{
    EulerAngleRotation currentRotation = EulerAngleRotation::FromRotation<true, false, false>(this->m_QRotation);
    return currentRotation.alpha();
}

mace_attitude_quaternion_t Rotation_3D::getMACEQuaternion() const
{
    mace_attitude_quaternion_t quat;

    return quat;
}

mace_attitude_t Rotation_3D::getMACEEuler() const
{
    EulerAngleRotation currentRotation = this->getEulerRotation();

    mace_attitude_t euler;
    euler.roll = static_cast<float>(currentRotation.gamma());
    euler.pitch = static_cast<float>(currentRotation.beta());
    euler.yaw = static_cast<float>(currentRotation.alpha());

    return euler;
}

mace_message_t Rotation_3D::getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const
{
    mace_message_t msg;
    mace_attitude_t attitude = getMACEEuler();
    mace_msg_attitude_encode_chan(systemID,compID,chan,&msg,&attitude);
    return msg;
}


} //end of namespace pose
} //end of namespace mace
