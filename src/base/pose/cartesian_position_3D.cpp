#include "cartesian_position_2D.h"
#include "cartesian_position_3D.h"

namespace mace{
namespace pose{


CartesianPosition_3D::CartesianPosition_3D(const std::string &pointName):
    Abstract_CartesianPosition(CartesianFrameTypes::CF_LOCAL_ENU, pointName), Abstract_Altitude(), State()
{
    this->dimension = 3;
}

CartesianPosition_3D::CartesianPosition_3D(const double &x, const double &y, const double &z):
    Abstract_CartesianPosition(CartesianFrameTypes::CF_LOCAL_ENU, x, y, "Cartesian Point"), Abstract_Altitude(AltitudeReferenceTypes::REF_ALT_RELATIVE,z), State()
{
    this->dimension = 3;
}

CartesianPosition_3D::CartesianPosition_3D(const std::string &pointName, const double &x, const double &y, const double &z):
    Abstract_CartesianPosition(CartesianFrameTypes::CF_LOCAL_ENU, x, y, pointName), Abstract_Altitude(AltitudeReferenceTypes::REF_ALT_RELATIVE,z), State()
{
    this->dimension = 3;
}

CartesianPosition_3D::CartesianPosition_3D(const CartesianPosition_3D &copy):
    Abstract_CartesianPosition(copy), Abstract_Altitude(copy), state_space::State(copy)
{
    this->dimension = 3;
}

CartesianPosition_3D::CartesianPosition_3D(const CartesianPosition_2D &copy):
    Abstract_CartesianPosition(copy), Abstract_Altitude(), state_space::State(copy)
{
    this->dimension = 3;
}

bool CartesianPosition_3D::areEquivalentFrames(const CartesianPosition_3D &obj) const
{
    return this->areEquivalentCartesianFrames(obj) && this->areEquivalentAltitudeFrames(obj);
}

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

bool CartesianPosition_3D::hasAltitudeBeenSet() const
{
    return this->hasZBeenSet();
}

double CartesianPosition_3D::elevationFromOrigin() const
{
    CartesianPosition_3D origin;
    double distanceXY = distanceBetween2D(&origin);
    return atan2(this->getZPosition(),distanceXY); //we should check here for the discontinuity if they are both 0
}

double CartesianPosition_3D::elevationAngleTo(const Abstract_CartesianPosition* pos) const
{
    double elevationAngle = 0.0;

    if(pos->is3D())
    {
        const CartesianPosition_3D* tmpPos = pos->positionAs<CartesianPosition_3D>();
        double distance2D = distanceBetween2D(pos);
        double distanceAlt = deltaAltitude(tmpPos);

        if(fabs(distance2D) > std::numeric_limits<double>::epsilon() || fabs(distanceAlt) > std::numeric_limits<double>::epsilon())
                elevationAngle = atan2(deltaAltitude(tmpPos),distance2D);
    }
    return elevationAngle;
}

double CartesianPosition_3D::deltaAltitude(const Abstract_Altitude* pos) const
{
    return this->getZPosition() - pos->getAltitude();
}


double CartesianPosition_3D::distanceBetween2D(const Abstract_CartesianPosition* pos) const
{
    const CartesianPosition_3D* tmpPos = pos->positionAs<CartesianPosition_3D>();
    double deltaX = this->getXPosition() - tmpPos->getXPosition();
    double deltaY = this->getYPosition() - tmpPos->getYPosition();

    double distance = sqrt(pow(deltaX,2) + pow(deltaY,2));
    return distance;
}

double CartesianPosition_3D::distanceBetween3D(const Abstract_CartesianPosition* pos) const
{
    double distance = 0.0;
    if(pos->is3D())
    {
        const CartesianPosition_3D* tmpPos = pos->positionAs<CartesianPosition_3D>();
        distance = sqrt(pow(this->distanceBetween2D(tmpPos),2) + pow(this->deltaAltitude(tmpPos),2));
    }
    else
    {
        distance = this->distanceBetween2D(pos);
    }

    return distance;
}

// ** DEPRECATED ** //
double CartesianPosition_3D::distanceTo(const Abstract_CartesianPosition* pos) const
{
    return this->distanceBetween2D(pos);
}

//!
//! \brief CartesianPosition_3D::polarBearingTo
//! \param pos
//! \return polar
//!
double CartesianPosition_3D::polarBearingTo(const Abstract_CartesianPosition* pos) const
{
    double polarBearing = 0.0;

    if(pos->isGreaterThan1D())
    {
        const CartesianPosition_2D* tmpPos = pos->positionAs<CartesianPosition_2D>();
        double deltaY = this->getYPosition() - tmpPos->getYPosition();
        double deltaX = this->getXPosition() - tmpPos->getXPosition();
        polarBearing = atan2(deltaY,deltaX);
    }

    return polarBearing;
}

//!
//! \brief CartesianPosition_3D::polarBearingTo
//! \param pos
//! \return polar
//!
double CartesianPosition_3D::compassBearingTo(const Abstract_CartesianPosition* pos) const
{
    return math::polarToCompassBearing(polarBearingTo(pos));
}

//!
//! \brief CartesianPosition_2D::newPositionFromPolar
//! \param distance
//! \param bearing
//! \return
//!
void CartesianPosition_3D::newPositionFromPolar(Abstract_CartesianPosition *newObject, const double &distance, const double &bearing) const
{
    if(newObject->isGreaterThan1D())
    {
        CartesianPosition_3D* tmpPos = newObject->positionAs<CartesianPosition_3D>();
        tmpPos->setXPosition(this->getXPosition() + cos(bearing) * distance);
        tmpPos->setYPosition(this->getYPosition() + sin(bearing) * distance);
    }
}


//!
//! \brief CartesianPosition_3D::newPositionFromCompass
//! \param distance
//! \param bearing
//! \return
//!
void CartesianPosition_3D::newPositionFromCompass(Abstract_CartesianPosition* newObject, const double &distance, const double &bearing) const
{
    return newPositionFromPolar(newObject, distance,compassToPolarBearing(bearing));
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
