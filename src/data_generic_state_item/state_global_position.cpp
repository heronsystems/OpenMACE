#include "state_global_position.h"
#include <cmath>

namespace DataState{

StateGlobalPosition::StateGlobalPosition():
    Base3DPosition(Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT)
{

}

StateGlobalPosition::StateGlobalPosition(const mace::pose::GeodeticPosition_3D &position):
    Base3DPosition(Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT)
{
    if(position.hasLatitudeBeenSet())
        setLatitude(position.getLatitude());
    if(position.hasLongitudeBeenSet())
        setLatitude(position.getLongitude());
    if(position.hasAltitudeBeenSet())
        setLatitude(position.getAltitude());
}

StateGlobalPosition::StateGlobalPosition(const StateGlobalPosition &globalPosition):
    Base3DPosition(globalPosition)
{

}

StateGlobalPosition::StateGlobalPosition(const Data::CoordinateFrameType &frame):
    Base3DPosition(frame)
{

}

StateGlobalPosition::StateGlobalPosition(const float &latitude, const float &longitude, const float &altitude):
    Base3DPosition(Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT,latitude,longitude,altitude)
{

}

StateGlobalPosition::StateGlobalPosition(const Data::CoordinateFrameType &frame, const double &latitude, const double &longitude, const double &altitude):
    Base3DPosition(frame,latitude,longitude,altitude)
{

}

StateGlobalPosition::StateGlobalPosition(const mace_global_position_int_t &pos)
{
    setLatitude(pos.lat / pow(10,7));
    setLongitude(pos.lon / pow(10,7));
    setAltitude(pos.alt / pow(10,3));
}

void StateGlobalPosition::setPosition(const double &latitude, const double &longitude, const double &altitude)
{
    this->setX(latitude);
    this->setY(longitude);
    this->setZ(altitude);
}

double StateGlobalPosition::getLatitude() const
{
    return this->getX();
}
double StateGlobalPosition::getLongitude() const
{
    return this->getY();
}
double StateGlobalPosition::getAltitude() const
{
    return this->getZ();
}

void StateGlobalPosition::setLatitude(const double &value)
{
    this->setX(value);
}
void StateGlobalPosition::setLongitude(const double &value)
{
    this->setY(value);
}

void StateGlobalPosition::setAltitude(const double &value)
{
    this->setZ(value);
}

mace_global_position_int_t StateGlobalPosition::getMACECommsObject() const
{
    mace_global_position_int_t rtnObj;
    rtnObj.lat = (int32_t)(this->getLatitude() * pow(10,7));
    rtnObj.lon = (int32_t)(this->getLongitude() * pow(10,7));
    rtnObj.alt = (int32_t)(this->getAltitude() * 1000.0);
    rtnObj.relative_alt = 0;
    rtnObj.hdg = 0;
    rtnObj.time_boot_ms = 0;

    return rtnObj;
}

mace_message_t StateGlobalPosition::getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const
{
    mace_message_t msg;
    mace_global_position_int_t pos = getMACECommsObject();
    mace_msg_global_position_int_encode_chan(systemID,compID,chan,&msg,&pos);
    return msg;
}

//!
//! \brief StateGlobalPosition::distanceBetween2D
//! \param position
//! \return
//!
double StateGlobalPosition::distanceBetween2D(const StateGlobalPosition &position) const
{
    double earthRadius = 6371000; //approximate value in meters

    double originLatitude = convertDegreesToRadians(this->x);
    double originLongitude = convertDegreesToRadians(this->y);
    double finalLatitude = convertDegreesToRadians(position.x);
    double finalLongitude = convertDegreesToRadians(position.y);

    double deltaLatitude = finalLatitude - originLatitude;
    double deltaLongitude = finalLongitude - originLongitude;

    double tmpA = sin(deltaLatitude/2) * sin(deltaLatitude/2) +
            cos(originLatitude) * cos(finalLatitude) *
            sin(deltaLongitude/2) * sin(deltaLongitude/2);

    double tmpC = 2 * atan2(sqrt(tmpA),sqrt(1-tmpA));

    double distance = earthRadius * tmpC;

    return distance;
}

//!
//! \brief StateGlobalPosition::finalBearing
//! \param position
//! \return
//!
double StateGlobalPosition::finalBearing(const StateGlobalPosition &position) const
{
    throw std::runtime_error("Not Implimented");
    UNUSED(position);
    return 0.0;
    //return (bearingBetween(position) + 180.0) % 360.0;
}

//!
//! \brief StateGlobalPosition::initialBearing
//! \param position
//! \return
//!
double StateGlobalPosition::initialBearing(const StateGlobalPosition &position) const
{
    throw std::runtime_error("Not Implimented");
    UNUSED(position);
    return 0.0;
    //return (bearingBetween(position) + 360.0) % 360.0;
}


//!
//! \brief StateGlobalPosition::bearingBetween
//! \param position
//! \return
//!
double StateGlobalPosition::bearingBetween(const StateGlobalPosition &position) const
{

    double originLatitude = convertDegreesToRadians(this->x);
    double originLongitude = convertDegreesToRadians(this->y);
    double finalLatitude = convertDegreesToRadians(position.x);
    double finalLongitude = convertDegreesToRadians(position.y);

    double deltaLongitude = finalLongitude - originLongitude;

    float tmpY = sin(deltaLongitude) * cos(finalLatitude);
    float tmpX = cos(originLatitude) * sin(finalLatitude) -
            sin(originLatitude) * cos(finalLatitude) *
            cos(deltaLongitude);
    float bearing = atan2(tmpY,tmpX);
    return bearing;
    //return convertRadiansToDegrees(bearing);
}

//!
//! \brief StateGlobalPosition::NewPositionFromHeadingBearing
//! \param distance
//! \param bearing
//! \param degreesFlag
//! \return
//!
StateGlobalPosition StateGlobalPosition::NewPositionFromHeadingBearing(const double &distance, const double &bearing, const bool &degreesFlag) const
{
    double earthRadius = 6371000; //approximate value in meters
    double latitudeRad = convertDegreesToRadians(x);
    double longitudeRad = convertDegreesToRadians(y);
    double bearingRad = 0.0;

    if(degreesFlag == true){
        bearingRad = convertDegreesToRadians(bearing);
    }else{
        bearingRad = bearing;
    }
    double distanceRatio = distance / earthRadius;

    double newLat = asin(sin(latitudeRad) * cos(distanceRatio) + cos(latitudeRad) * sin(distanceRatio) * cos(bearingRad));
    double newLon = longitudeRad + atan2(sin(bearingRad) * sin(distanceRatio) * cos(latitudeRad),
                                         cos(distanceRatio) - sin(latitudeRad) * sin(newLat));

    StateGlobalPosition newPos;
    newPos.x = convertRadiansToDegrees(newLat);
    newPos.y = -convertRadiansToDegrees(newLon);
    newPos.z = z;

    return newPos;
}

//!
//! \brief StateGlobalPosition::translationTransformation2D
//! \param position
//! \param transVec
//!
void StateGlobalPosition::translationTransformation2D(const StateGlobalPosition &position, Eigen::Vector2f &transVec) const
{
    double bearing = this->bearingBetween(position);
    double distance = this->distanceBetween2D(position);
    float distanceX = distance * sin(bearing);
    float distanceY = distance * cos(bearing);
    transVec(0) = distanceX;
    transVec(1) = distanceY;
}

//!
//! \brief StateGlobalPosition::deltaAltitude
//! \param position
//! \return
//!
double StateGlobalPosition::deltaAltitude(const StateGlobalPosition &position) const
{
    return (this->z - position.z);
}


//!
//! \brief StateGlobalPosition::distanceBetween3D
//! \param position
//! \return
//!
double StateGlobalPosition::distanceBetween3D(const StateGlobalPosition &position) const
{
    double distance2D = this->distanceBetween2D(position);
    double deltaAltitude = fabs(this->z - position.z);
    return(sqrt(deltaAltitude * deltaAltitude + distance2D * distance2D));
}

//!
//! \brief StateGlobalPosition::translationTransformation3D
//! \param position
//! \param transVec
//!
void StateGlobalPosition::translationTransformation3D(const StateGlobalPosition &position, Eigen::Vector3f &transVec) const
{
    double bearing = this->bearingBetween(position);
    double distance = this->distanceBetween2D(position);
    float distanceZ = -this->deltaAltitude(position);
    float distanceX = distance * sin(bearing);
    float distanceY = distance * cos(bearing);
    transVec(0) = distanceX;
    transVec(1) = distanceY;
    transVec(2) = distanceZ;
}


/**
 * @brief Position::convertDegreesToRadians
 * @param degrees
 * @return
 */
double StateGlobalPosition::convertDegreesToRadians(const double &degrees)
{
    double pi = 3.14159265358979323846;
    double radians = degrees * (pi/180.0);
    return radians;
}

/**
 * @brief Position::convertRadiansToDegrees
 * @param radians
 * @return
 */
double StateGlobalPosition::convertRadiansToDegrees(const double &radians)
{
    double pi = 3.14159265358979323846;
    double degrees = radians * (180.0/pi);
    return degrees;
}

}
