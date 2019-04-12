#include "cartesian_position_3D.h"

namespace mace{
namespace pose{

double CartesianPosition_3D::deltaX(const CartesianPosition_3D &that) const
{
    return this->getXPosition() - that.getXPosition();
}

double CartesianPosition_3D::deltaY(const CartesianPosition_3D &that) const
{
    return this->getYPosition() - that.getYPosition();
}

double CartesianPosition_3D::deltaZ(const CartesianPosition_3D &that) const
{
    return this->getZPosition() - that.getZPosition();
}

double CartesianPosition_3D::distanceFromOrigin() const
{
    return sqrt(pow(this->getXPosition(),2) + pow(this->getYPosition(),2) + pow(this->getZPosition(),2));
}

double CartesianPosition_3D::polarBearingFromOrigin() const
{
    return atan2(this->getYPosition(),this->getXPosition());
}

double CartesianPosition_3D::elevationFromOrigin() const
{
    CartesianPosition_3D origin;
    double distanceXY = distanceBetween2D(origin);
    return atan2(this->getZPosition(),distanceXY); //we should check here for the discontinuity if they are both 0
}

double CartesianPosition_3D::elevationAngleTo(const CartesianPosition_3D &position) const
{
    double distance2D = distanceBetween2D(position);
    return atan2(deltaAltitude(position),distance2D);
}

double CartesianPosition_3D::deltaAltitude(const CartesianPosition_3D &pos) const
{
    return this->deltaZ(pos);
}


double CartesianPosition_3D::distanceBetween2D(const CartesianPosition_3D &pos) const
{
    double deltaX = this->deltaX(pos);
    double deltaY = this->deltaY(pos);
    double distance = sqrt(pow(deltaX,2) + pow(deltaY,2));
    return distance;
}

double CartesianPosition_3D::distanceBetween3D(const CartesianPosition_3D &pos) const
{
    return sqrt(pow(this->distanceBetween2D(pos),2) + pow(this->deltaAltitude(pos),2));
}

// ** DEPRECATED ** //
double CartesianPosition_3D::distanceTo(const CartesianPosition_3D &pos) const
{
    return this->distanceBetween2D(pos);
}

//!
//! \brief CartesianPosition_3D::polarBearingTo
//! \param pos
//! \return polar
//!
double CartesianPosition_3D::polarBearingTo(const CartesianPosition_3D &pos) const
{
    return atan2(deltaY(pos),deltaX(pos));
}

//!
//! \brief CartesianPosition_3D::polarBearingTo
//! \param pos
//! \return polar
//!
double CartesianPosition_3D::compassBearingTo(const CartesianPosition_3D &pos) const
{
    return polarToCompassBearing(polarBearingTo(pos));
}

//!
//! \brief CartesianPosition_3D::newPositionFromPolar
//! \param distance
//! \param bearing
//! \return
//!
CartesianPosition_3D CartesianPosition_3D::newPositionFromPolar(const double &distance, const double &bearing) const
{
    CartesianPosition_3D newPos;

    newPos.setXPosition(this->getXPosition() + cos(bearing) * distance);
    newPos.setYPosition(this->getYPosition() + sin(bearing) * distance);
    newPos.setZPosition(this->getZPosition());
    return newPos;
}


//!
//! \brief CartesianPosition_3D::newPositionFromCompass
//! \param distance
//! \param bearing
//! \return
//!
CartesianPosition_3D CartesianPosition_3D::newPositionFromCompass(const double &distance, const double &bearing) const
{
    return newPositionFromPolar(distance,compassToPolarBearing(bearing));
}

void CartesianPosition_3D::applyPositionalShiftFromPolar(const double &distance, const double &bearing)
{
    double changeX = distance * cos(bearing);
    double changeY = distance * sin(bearing);
    this->setXPosition(getXPosition() + changeX);
    this->setYPosition(getYPosition() + changeY);
}

void CartesianPosition_3D::applyPositionalShiftFromPolar(const double &distance, const double &bearing, const double &elevation)
{
    double changeZ = distance * sin(elevation);
    this->setZPosition(changeZ + this->getZPosition());

    double changeXY = distance * cos(elevation);
    applyPositionalShiftFromPolar(changeXY,bearing);
}

void CartesianPosition_3D::applyPositionalShiftFromCompass(const double &distance, const double &bearing)
{
    double polarBearing = compassToPolarBearing(bearing);
    applyPositionalShiftFromPolar(distance,polarBearing);
}

void CartesianPosition_3D::applyPositionalShiftFromCompass(const double &distance, const double &bearing, const double &elevation)
{
    applyPositionalShiftFromPolar(distance,compassToPolarBearing(bearing),elevation);
}

} //end of namespace pose
} //end of namespace mace
