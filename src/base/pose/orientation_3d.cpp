#include "orientation_3D.h"

namespace mace {
namespace pose {

Orientation_3D::Orientation_3D():
    updatedEuler(true),
    theta(0.0), psi(0.0), phi(0.0)
{
    this->updateMatrix();
}

Orientation_3D::~Orientation_3D()
{

}

Orientation_3D::Orientation_3D(const Orientation_3D &copy)
{
    this->theta = copy.theta;
    this->psi = copy.psi;
    this->phi = copy.phi;
    this->matrixRot = copy.matrixRot;
    this->updatedEuler = copy.updatedEuler;
}

Orientation_3D::Orientation_3D(const double &roll, const double &pitch, const double &yaw)
{
    this->setEuler(roll,pitch,yaw);
}

/** Set functions for all rotation angle information **/

void Orientation_3D::setEuler(const double &roll, const double &pitch, const double &yaw)
{    
    this->theta = roll;
    this->psi = pitch;
    this->phi = yaw;
    this->updatedEuler = true;

    this->updateMatrix();
}

void Orientation_3D::setRotation(const Eigen::Matrix3d &rotation)
{
    this->matrixRot = rotation;
    this->updatedEuler = false;
}

void Orientation_3D::setRoll(const double &angle)
{
    this->theta = angle;
    this->updateMatrix();
}

void Orientation_3D::setPitch(const double &angle)
{
    this->psi = angle;
    this->updateMatrix();
}


void Orientation_3D::setYaw(const double &angle)
{
    this->phi = angle;
    this->updateMatrix();
}


/** Get functions for all rotation angle information **/

void Orientation_3D::getEuler(double &roll, double &pitch, double &yaw) const
{
   this->updateEuler();
    roll = this->theta;
    pitch = this->psi;
    yaw = this->phi;
}

double Orientation_3D::getRoll() const
{
    return this->theta;
}

double Orientation_3D::getPitch() const
{
    return this->psi;
}

double Orientation_3D::getYaw() const
{
    return this->phi;
}

void Orientation_3D::getRotationMatrix(Eigen::Matrix3d &rotM) const
{
    rotM = this->matrixRot;
}

void Orientation_3D::getRotationVector(Eigen::Vector3d &rotV) const
{
    this->updateEuler();
    rotV(0) = phi;
    rotV(1) = theta;
    rotV(2) = psi;
}



} //end of namespace pose
} //end of namespace mace
