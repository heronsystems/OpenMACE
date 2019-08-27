#include "virtual_force.h"
#include <cmath>
#include <iostream>


VPF_ResultingForce::VPF_ResultingForce(const double &saturation):
    saturationMagnitude(saturation), saturated(false),
    forceX(0), forceY(0), direction(0)
{

}

VPF_ResultingForce::VPF_ResultingForce(const VPF_ResultingForce &copy)
{
    this->saturationMagnitude = copy.saturationMagnitude;
    this->saturated = copy.saturated;
    this->forceX = copy.forceX;
    this->forceY = copy.forceY;
    this->direction = copy.direction;
}

bool VPF_ResultingForce::isSaturated() const
{
    return saturated;
}

void VPF_ResultingForce::updateDirection()
{
    if((std::abs(this->forceX) <= std::numeric_limits<double>::epsilon()) &&
            (std::abs(this->forceY) <= std::numeric_limits<double>::epsilon()))
    {
       this->direction = 0.0;
    }
    else
    {
        this->direction = atan2(forceY, forceX);
    }
}

void VPF_ResultingForce::updateMagnitude()
{
    double magnitude = std::sqrt(std::pow(this->forceX,2) + std::pow(this->forceY,2));
    if(magnitude >= saturationMagnitude)
        saturated = true;
    else
        saturated = false;
}


double VPF_ResultingForce::getDirection() const
{
    return direction;
}

double VPF_ResultingForce::getForceY() const
{
    return forceY;
}

void VPF_ResultingForce::setForceY(const double &value)
{
    forceY = value;
    updateMag_Dir();
}

void VPF_ResultingForce::addForceY(const double &value)
{
    forceY += value;
    updateMag_Dir();
}


double VPF_ResultingForce::getForceX() const
{
    return forceX;
}

void VPF_ResultingForce::setForceX(const double &value)
{
    forceX = value;
    updateMag_Dir();
}

void VPF_ResultingForce::addForceX(const double &value)
{
    forceX += value;
    updateMag_Dir();
}
