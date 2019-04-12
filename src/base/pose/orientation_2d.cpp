#include "orientation_2D.h"

namespace mace {
namespace pose {

Orientation_2D::Orientation_2D():
  updatedTrig(false),cosPhi(0.0),sinPhi(0.0),
angleFlag(false), phi(0.0)
{

}

Orientation_2D::Orientation_2D(const Orientation_2D &copy):
    updatedTrig(false),cosPhi(0.0),sinPhi(0.0),
  angleFlag(false), phi(0.0)
{
    this->phi = copy.phi;
    this->angleFlag = copy.angleFlag;
}

Orientation_2D::Orientation_2D(const double &angle):
    updatedTrig(false),cosPhi(0.0),sinPhi(0.0),
  angleFlag(false), phi(0.0)
{
    this->setPhi(angle);
}

void Orientation_2D::setPhi(const double &angle)
{
    this->phi = angle;
    this->angleFlag = true;

    this->updatedTrig = false;
}

double Orientation_2D::getPhi() const
{
    return this->phi;
}

void Orientation_2D::getRotationMatrix(Eigen::Matrix2d &rotM) const
{
    this->updateTrigCache();

    rotM(0,0) = cosPhi;
    rotM(1,0) = -sinPhi;
    rotM(1,0) = sinPhi;
    rotM(1,1) = cosPhi;
}

void Orientation_2D::getRotationMatrix(Eigen::Matrix3d &rotM) const
{
    this->updateTrigCache();

    rotM(0,0) = cosPhi;
    rotM(0,1) = -sinPhi;
    rotM(0,2) = 0.0;

    rotM(1,0) = sinPhi;
    rotM(1,1) = cosPhi;
    rotM(1,2) = 0.0;

    rotM(2,0) = 0.0;
    rotM(2,1) = 0.0;
    rotM(2,2) = 1.0;
}



} //end of namespace pose
} //end of namespace mace
