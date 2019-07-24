#include "orientation_3D.h"

using namespace Eigen;

namespace mace {
namespace pose {

Orientation_3D::Orientation_3D(const std::string &name)
{
    this->name = name;
    this->m_QRotation.setIdentity();
}

Orientation_3D::~Orientation_3D()
{

}

Orientation_3D::Orientation_3D(const Orientation_3D &copy)
{
    this->name = copy.name;
    this->m_QRotation.setIdentity();
    this->m_QRotation = copy.m_QRotation;
}

Orientation_3D::Orientation_3D(const double &roll, const double &pitch, const double &yaw, const std::string &name)
{
    this->name = name;
    this->m_QRotation.setIdentity();
    this->updateFromEuler(roll,pitch,yaw);
}

/** Set functions for all rotation angle information **/

void Orientation_3D::updateFromEuler(const double &roll, const double &pitch, const double &yaw)
{
    EulerAngleRotation currentRotation = EulerAngleRotation::FromRotation<true, false, false>(this->m_QRotation);
    currentRotation.alpha() = yaw;
    currentRotation.beta() = pitch;
    currentRotation.gamma() = roll;

    this->m_QRotation = Eigen::AngleAxisd(currentRotation.alpha(), Vector3d::UnitZ()) * Eigen::AngleAxisd(currentRotation.beta(), Vector3d::UnitY()) * Eigen::AngleAxisd(currentRotation.gamma(), Vector3d::UnitX());
}

void Orientation_3D::setRotation(const Eigen::Matrix3d &rotation)
{
    this->m_QRotation = Eigen::Quaterniond(rotation);
}

void Orientation_3D::updateRoll(const double &angle)
{
    EulerAngleRotation currentRotation = EulerAngleRotation::FromRotation<true, false, false>(this->m_QRotation);
    currentRotation.gamma() = angle;
    this->m_QRotation = Eigen::AngleAxisd(currentRotation.alpha(), Vector3d::UnitZ()) * Eigen::AngleAxisd(currentRotation.beta(), Vector3d::UnitY()) * Eigen::AngleAxisd(currentRotation.gamma(), Vector3d::UnitX());
}

void Orientation_3D::updatePitch(const double &angle)
{
    EulerAngleRotation currentRotation = EulerAngleRotation::FromRotation<true, false, false>(this->m_QRotation);
    currentRotation.beta() = angle;
    this->m_QRotation = Eigen::AngleAxisd(currentRotation.alpha(), Vector3d::UnitZ()) * Eigen::AngleAxisd(currentRotation.beta(), Vector3d::UnitY()) * Eigen::AngleAxisd(currentRotation.gamma(), Vector3d::UnitX());
}


void Orientation_3D::updateYaw(const double &angle)
{
    EulerAngleRotation currentRotation = EulerAngleRotation::FromRotation<true, false, false>(this->m_QRotation);
    currentRotation.alpha() = angle;
    this->m_QRotation = Eigen::AngleAxisd(currentRotation.alpha(), Vector3d::UnitZ()) * Eigen::AngleAxisd(currentRotation.beta(), Vector3d::UnitY()) * Eigen::AngleAxisd(currentRotation.gamma(), Vector3d::UnitX());
}


/** Get functions for all rotation angle information **/

void Orientation_3D::getDiscreteEuler(double &roll, double &pitch, double &yaw) const
{
    EulerAngleRotation currentRotation = EulerAngleRotation::FromRotation<false, false, false>(this->m_QRotation.matrix());
    roll = currentRotation.gamma();
    pitch = currentRotation.beta();
    yaw = currentRotation.alpha();
}

EulerAngleRotation Orientation_3D::getEulerRotation() const
{
    return EulerAngleRotation::FromRotation<true, false, false>(this->m_QRotation.toRotationMatrix());
}

Eigen::Matrix3d Orientation_3D::getRotationMatrix() const
{
    return this->m_QRotation.toRotationMatrix();
}

double Orientation_3D::getRoll() const
{
    EulerAngleRotation currentRotation = EulerAngleRotation::FromRotation<true, false, false>(this->m_QRotation);
    return currentRotation.gamma();
}

double Orientation_3D::getPitch() const
{
    EulerAngleRotation currentRotation = EulerAngleRotation::FromRotation<true, false, false>(this->m_QRotation);
    return currentRotation.beta();
}

double Orientation_3D::getYaw() const
{
    EulerAngleRotation currentRotation = EulerAngleRotation::FromRotation<true, false, false>(this->m_QRotation);
    return currentRotation.alpha();
}


} //end of namespace pose
} //end of namespace mace
