#include "state_local_position.h"
#include <cmath>

using namespace DataState;

StateLocalPosition::StateLocalPosition():
    Base3DPosition(Data::CoordinateFrameType::CF_LOCAL_ENU)
{

}

StateLocalPosition::StateLocalPosition(const StateLocalPosition &localPosition):
    Base3DPosition(localPosition)
{

}

StateLocalPosition::StateLocalPosition(const mace::pose::CartesianPosition_3D &position):
    Base3DPosition(Data::CoordinateFrameType::CF_LOCAL_ENU)
{
    if(position.hasXBeenSet())
        setX(position.getXPosition());
    if(position.hasYBeenSet())
        setY(position.getYPosition());
    if(position.hasZBeenSet())
        setY(position.getZPosition());
}

StateLocalPosition::StateLocalPosition(const Data::CoordinateFrameType &frame):
    Base3DPosition(frame)
{

}

StateLocalPosition::StateLocalPosition(const double &posX, const double &posY, const double &posZ):
    Base3DPosition(Data::CoordinateFrameType::CF_LOCAL_ENU,posX,posY,posZ)
{

}

StateLocalPosition::StateLocalPosition(const Data::CoordinateFrameType &frame, const double &posX, const double &posY, const double &posZ):
    Base3DPosition(frame,posX,posY,posZ)
{

}

StateLocalPosition::StateLocalPosition(const mace_local_position_ned_t &pos)
{
    this->setX(pos.x);
    this->setY(pos.y);
    this->setZ(pos.z);
}

void StateLocalPosition::setPosition(const double &posX, const double &posY, const double &posZ)
{
    this->setX(posX);
    this->setY(posY);
    this->setZ(posZ);
}

void StateLocalPosition::setPositionX(const double &value)
{
    this->setX(value);
}

void StateLocalPosition::setPositionY(const double &value)
{
    this->setY(value);
}

void StateLocalPosition::setPositionZ(const double &value)
{
    this->setZ(value);
}

double StateLocalPosition::getPositionX() const
{
    return getX();
}
double StateLocalPosition::getPositionY() const
{
    return this->getY();
}
double StateLocalPosition::getPositionZ() const
{
    return this->getZ();
}

double StateLocalPosition::bearingDegreesFromOrigin() const
{
    double angle = atan2(y,x);
    double bearing = fmod((angle * 180.0/M_PI) + 360.0,360.0);
    return bearing;
}

double StateLocalPosition::distanceFromOrigin() const
{
    double distance = sqrt(x*x + y*y);
    return distance;
}

mace_local_position_ned_t StateLocalPosition::getMACECommsObject()
{
    mace_local_position_ned_t rtnObj;

    rtnObj.x = (int32_t)(this->getPositionX());
    rtnObj.y = (int32_t)(this->getPositionY());
    rtnObj.z = (int32_t)(this->getPositionZ());

    return rtnObj;
}

bool StateLocalPosition::essentiallyEquivalent_Percentage(const StateLocalPosition &rhs, const double &percentage)
{
   double changeX = (fabs(this->x - rhs.x)/fabs(this->x)) * 100.0;
   double changeY = (fabs(this->y - rhs.y)/fabs(this->y)) * 100.0;
   double changeZ = (fabs(this->z - rhs.z)/fabs(this->z)) * 100.0;

   if(changeX > percentage)
       return false;
   if(changeY > percentage)
       return false;
   if(changeZ > percentage)
       return false;

    return true;
}

bool StateLocalPosition::essentiallyEquivalent_Distance(const StateLocalPosition &rhs, const double &distance)
{
   double changeX = fabs(this->x - rhs.x);
   double changeY = fabs(this->y - rhs.y);
   double changeZ = fabs(this->z - rhs.z);

   if(changeX > distance)
       return false;
   if(changeY > distance)
       return false;
   if(changeZ > distance)
       return false;

    return true;
}

//!
//! \brief StateLocalPosition::distanceBetween2D
//! \param position
//! \return
//!
double StateLocalPosition::distanceBetween2D(const StateLocalPosition &position) const
{
    double deltaX = position.x - this->x;
    double deltaY = position.y - this->y;

    return sqrt(deltaX * deltaX + deltaY * deltaY);
}

//!
//! \brief StateLocalPosition::finalBearing
//! \param position
//! \return
//!
double StateLocalPosition::finalBearing(const StateLocalPosition &position) const
{
    UNUSED(position);
    throw std::runtime_error("Not Implimented");
    return 0.0;
}

//!
//! \brief StateLocalPosition::initialBearing
//! \param position
//! \return
//!
double StateLocalPosition::initialBearing(const StateLocalPosition &position) const
{
    UNUSED(position);
    throw std::runtime_error("Not Implimented");
    return 0.0;
}


//!
//! \brief StateLocalPosition::bearingBetween
//! \param position
//! \return
//!
double StateLocalPosition::bearingBetween(const StateLocalPosition &position) const
{
    if (y == position.y && x == position.x)
        return 0;

    double angle = atan2(y - position.y, x - position.x);
    double bearing = fmod((angle * 180.0/M_PI) + 360.0,360.0);
    return bearing;

//    UNUSED(position);
//    throw std::runtime_error("Not Implimented");
//    return 0.0;
}

//!
//! \brief StateLocalPosition::NewPositionFromHeadingBearing
//! \param distance
//! \param bearing
//! \param degreesFlag
//! \return
//!
StateLocalPosition StateLocalPosition::NewPositionFromHeadingBearing(const double &distance, const double &bearing, const bool &degreesFlag) const
{
//    UNUSED(distance);
//    UNUSED(bearing);
//    UNUSED(degreesFlag);
//    throw std::runtime_error("Not Implimented");

    StateLocalPosition newTemp;
    double bearingRadians = bearing;
    if (degreesFlag)
        mace::math::convertDegreesToRadians(bearingRadians);

    newTemp.setPositionX(x + distance * cos(bearingRadians));
    newTemp.setPositionY(y + distance * sin(bearingRadians));
    if (this->getPosZFlag())
        newTemp.setPositionZ(z);
    return newTemp;
}

//!
//! \brief StateLocalPosition::translationTransformation2D
//! \param position
//! \param transVec
//!
void StateLocalPosition::translationTransformation2D(const StateLocalPosition &position, Eigen::Vector2f &transVec) const
{
    double bearing = this->bearingBetween(position);
    double distance = this->distanceBetween2D(position);
    float distanceX = distance * sin(bearing);
    float distanceY = distance * cos(bearing);
    transVec(0) = distanceX;
    transVec(1) = distanceY;
}

//!
//! \brief StateLocalPosition::deltaAltitude
//! \param position
//! \return
//!
double StateLocalPosition::deltaAltitude(const StateLocalPosition &position) const
{
    double deltaZ = position.z - this->z;
    return deltaZ;
}


//!
//! \brief StateLocalPosition::distanceBetween3D
//! \param position
//! \return
//!
double StateLocalPosition::distanceBetween3D(const StateLocalPosition &position) const
{
    double distance2D = this->distanceBetween2D(position);
    double deltaAltitude = fabs(this->z - position.z);
    return(sqrt(deltaAltitude * deltaAltitude + distance2D * distance2D));
}

//!
//! \brief StateLocalPosition::translationTransformation3D
//! \param position
//! \param transVec
//!
void StateLocalPosition::translationTransformation3D(const StateLocalPosition &position, Eigen::Vector3f &transVec) const
{
    UNUSED(position);
    UNUSED(transVec);
}


