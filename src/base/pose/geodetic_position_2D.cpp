#include "geodetic_position_2D.h"
#include "geodetic_position_3D.h"

namespace mace{
namespace pose{

//!
//! \brief GeodeticPosition_2D::GeodeticPosition_2D
//!
GeodeticPosition_2D::GeodeticPosition_2D():
    Abstract_GeodeticPosition(GeodeticFrameTypes::CF_GLOBAL_RELATIVE_ALT, "Geodetic Point"), State()
{
    this->dimension = 2;
    this->setDimensionMask(ignoreAllPositions);
}

//!
//! \brief GeodeticPosition_2D
//! \param pointName
//! \param latitude
//! \param longitude
//!
GeodeticPosition_2D::GeodeticPosition_2D(const GeodeticFrameTypes &frameType,
                    const double &latitude, const double &longitude,
                    const std::string &pointName):
    Abstract_GeodeticPosition(frameType, pointName), State(), data(0.0,0.0)
{
    this->dimension = 2;
    this->setLatitude(latitude); this->setLongitude(longitude);
}

GeodeticPosition_2D::GeodeticPosition_2D(const std::string &pointName,
                    const double &latitude, const double &longitude):
    Abstract_GeodeticPosition(GeodeticFrameTypes::CF_GLOBAL_RELATIVE_ALT, pointName), State(), data(0.0,0.0)
{
    this->dimension = 2;
    this->setLatitude(latitude); this->setLongitude(longitude);
}

GeodeticPosition_2D::GeodeticPosition_2D(const double &latitude, const double &longitude):
    Abstract_GeodeticPosition(GeodeticFrameTypes::CF_GLOBAL_RELATIVE_ALT, "Geodetic Point"), State(), data(0.0,0.0)
{
    this->dimension = 2;
    this->setLatitude(latitude); this->setLongitude(longitude);
}


//!
//! \brief GeodeticPosition_2D
//! \param copy
//!
GeodeticPosition_2D::GeodeticPosition_2D(const GeodeticPosition_2D &copy):
    Abstract_GeodeticPosition(copy), state_space::State(copy), data(copy.data)
{

}

GeodeticPosition_2D::GeodeticPosition_2D(const GeodeticPosition_3D &copy):
    Abstract_GeodeticPosition(copy), state_space::State(copy), data(0.0,0.0)
{
    data(0) = copy.data(0);
    data(1) = copy.data(1);
}

bool GeodeticPosition_2D::hasLatitudeBeenSet() const
{
    if((this->dimensionMask&IGNORE_Y_DIMENSION) == 0)
        return true;
    return false;
}

bool GeodeticPosition_2D::hasLongitudeBeenSet() const
{
    if((this->dimensionMask&IGNORE_X_DIMENSION) == 0)
        return true;
    return false;
}

bool GeodeticPosition_2D::areEquivalentFrames(const GeodeticPosition_2D &obj) const
{
    return this->areEquivalentGeodeticFrames(obj);
}

void GeodeticPosition_2D::updateQJSONObject(QJsonObject &obj) const
{
    obj["lat"] = this->getLatitude();
    obj["lng"] = this->getLongitude();
}

double GeodeticPosition_2D::distanceFromOrigin() const
{
    GeodeticPosition_2D origin;
    return distanceBetween2D(&origin);
}

double GeodeticPosition_2D::translationalDistanceFromOrigin() const
{
    return distanceFromOrigin();
}

double GeodeticPosition_2D::polarBearingFromOrigin() const
{
    GeodeticPosition_2D origin;
    return origin.polarBearingTo(this);
}

double GeodeticPosition_2D::distanceBetween2D(const Abstract_GeodeticPosition* pos) const
{
    const GeodeticPosition_2D* tmpPos = pos->positionAs<GeodeticPosition_2D>();

    double earthRadius = 6371000; //approximate value in meters

    double originLatitude = math::convertDegreesToRadians(this->getLatitude());
    double originLongitude = math::convertDegreesToRadians(this->getLongitude());

    double finalLatitude = math::convertDegreesToRadians(tmpPos->getLatitude());
    double finalLongitude = math::convertDegreesToRadians(tmpPos->getLongitude());

    double deltaLatitude = finalLatitude - originLatitude;
    double deltaLongitude = finalLongitude - originLongitude;

    double tmpA = std::sin(deltaLatitude/2) * std::sin(deltaLatitude/2) +
            std::cos(originLatitude) * std::cos(finalLatitude) *
            std::sin(deltaLongitude/2) * std::sin(deltaLongitude/2);

    double tmpC = 2 * atan2(std::sqrt(tmpA),std::sqrt(1-tmpA));

    double distance = earthRadius * tmpC;

    return distance;
}

double GeodeticPosition_2D::distanceTo(const Abstract_GeodeticPosition* pos) const
{
    return this->distanceBetween2D(pos);
}

//!
//! \brief GeodeticPosition_2D::polarBearingTo
//! \param pos
//! \return polar
//!
double GeodeticPosition_2D::polarBearingTo(const Abstract_GeodeticPosition* pos) const
{
    const GeodeticPosition_2D* tmpPos = pos->positionAs<GeodeticPosition_2D>();

    double originLatitude = math::convertDegreesToRadians(this->getLatitude());
    double originLongitude = math::convertDegreesToRadians(this->getLongitude());

    double finalLatitude = math::convertDegreesToRadians(tmpPos->getLatitude());
    double finalLongitude = math::convertDegreesToRadians(tmpPos->getLongitude());

    double deltaLongitude = finalLongitude - originLongitude;

    double tmpY = std::sin(deltaLongitude) * std::cos(finalLatitude);
    double tmpX = std::cos(originLatitude) * std::sin(finalLatitude) -
            std::sin(originLatitude) * std::cos(finalLatitude) *
            std::cos(deltaLongitude);
    double bearing = atan2(tmpY,tmpX);
    return bearing;
}

//!
//! \brief GeodeticPosition_2D::compassBearingTo
//! \param pos
//! \return polar
//!
double GeodeticPosition_2D::compassBearingTo(const Abstract_GeodeticPosition* pos) const
{
    return math::correctBearing(polarBearingTo(pos));
}


//!
//! \brief GeodeticPosition_3D::newPositionFromPolar
//! \param distance
//! \param bearing
//! \return
//!
void GeodeticPosition_2D::newPositionFromPolar(Abstract_GeodeticPosition* newObject, const double &distance, const double &bearing) const
{
    newPositionFromCompass(newObject, distance, math::polarToCompassBearing(bearing));
}

//!
//! \brief GeodeticPosition_3D::newPositionFromCompass
//! \param distance
//! \param bearing
//! \return
//!
void GeodeticPosition_2D::newPositionFromCompass(Abstract_GeodeticPosition* newObject, const double &distance, const double &bearing) const
{
    double earthRadius = 6371000; //approximate value in meters
    double latitudeRad = math::convertDegreesToRadians(this->getLatitude());
    double longitudeRad = math::convertDegreesToRadians(this->getLongitude());

    double distanceRatio = distance / earthRadius;

    double newLat = asin(std::sin(latitudeRad) * std::cos(distanceRatio) + std::cos(latitudeRad) * std::sin(distanceRatio) * std::cos(bearing));
    double newLon = longitudeRad + atan2(std::sin(bearing) * std::sin(distanceRatio) * std::cos(latitudeRad),
                                         std::cos(distanceRatio) - std::sin(latitudeRad) * std::sin(newLat));
    if(newObject->isGreaterThan1D())
    {
        GeodeticPosition_2D* tmpPos = newObject->positionAs<GeodeticPosition_2D>();
        tmpPos->setLatitude(math::convertRadiansToDegrees(newLat));
        tmpPos->setLongitude(math::convertRadiansToDegrees(newLon));
    }
}

void GeodeticPosition_2D::applyPositionalShiftFromPolar(const double &distance, const double &bearing)
{
    GeodeticPosition_2D tmpPosition;
    newPositionFromPolar(&tmpPosition, distance, bearing);
    this->setLatitude(tmpPosition.getLatitude());
    this->setLongitude(tmpPosition.getLongitude());
}

void GeodeticPosition_2D::applyPositionalShiftFromCompass(const double &distance, const double &bearing)
{
    GeodeticPosition_2D tmpPosition;
    newPositionFromCompass(&tmpPosition, distance, bearing);
    this->setLatitude(tmpPosition.getLatitude());
    this->setLongitude(tmpPosition.getLongitude());
}

mavlink_global_position_int_t GeodeticPosition_2D::getMACE_GlobalPositionInt() const
{
    mavlink_global_position_int_t posObj;
    posObj.lat = static_cast<int32_t>((this->getLatitude() * pow(10,7)));
    posObj.lon = static_cast<int32_t>((this->getLongitude() * pow(10,7)));
    posObj.alt = 0;
    posObj.relative_alt = 0;
    posObj.hdg = 0;
    posObj.time_boot_ms = 0;
    return posObj;
}

mavlink_message_t GeodeticPosition_2D::getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const
{
    mavlink_message_t msg;
    mavlink_global_position_int_t positionObj = getMACE_GlobalPositionInt();
    mavlink_msg_global_position_int_encode_chan(systemID,compID,chan,&msg,&positionObj);
    return msg;
}

} //end of namespace pose
} //end of namespace mace
