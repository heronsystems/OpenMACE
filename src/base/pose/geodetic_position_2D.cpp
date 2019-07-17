#include "geodetic_position_2D.h"
#include "geodetic_position_3D.h"

namespace mace{
namespace pose{

//!
//! \brief GeodeticPosition_2D
//! \param pointName
//! \param latitude
//! \param longitude
//!
GeodeticPosition_2D::GeodeticPosition_2D(const GeodeticFrameTypes &frameType,
                    const double &latitude, const double &longitude,
                    const std::string &pointName):
    Abstract_GeodeticPosition(frameType, latitude, longitude, pointName), State()
{
    this->dimension = 2;
}

GeodeticPosition_2D::GeodeticPosition_2D(const std::string &pointName,
                    const double &latitude, const double &longitude):
    Abstract_GeodeticPosition(GeodeticFrameTypes::CF_GLOBAL_RELATIVE_ALT, latitude, longitude, pointName), State()
{
    this->dimension = 2;
}

GeodeticPosition_2D::GeodeticPosition_2D(const double &latitude, const double &longitude):
    Abstract_GeodeticPosition(GeodeticFrameTypes::CF_GLOBAL_RELATIVE_ALT, latitude, longitude, "Geodetic Point"), State()
{
    this->dimension = 2;
}


//!
//! \brief GeodeticPosition_2D
//! \param copy
//!
GeodeticPosition_2D::GeodeticPosition_2D(const GeodeticPosition_2D &copy):
    Abstract_GeodeticPosition(copy), state_space::State(copy)
{

}

GeodeticPosition_2D::GeodeticPosition_2D(const GeodeticPosition_3D &copy):
    Abstract_GeodeticPosition(copy), state_space::State(copy)
{

}

bool GeodeticPosition_2D::areEquivalentFrames(const GeodeticPosition_2D &obj) const
{
    return this->areEquivalentGeodeticFrames(obj);
}


double GeodeticPosition_2D::distanceFromOrigin() const
{
    GeodeticPosition_2D origin;
    return distanceBetween2D(&origin);
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

    double tmpA = sin(deltaLatitude/2) * sin(deltaLatitude/2) +
            cos(originLatitude) * cos(finalLatitude) *
            sin(deltaLongitude/2) * sin(deltaLongitude/2);

    double tmpC = 2 * atan2(sqrt(tmpA),sqrt(1-tmpA));

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

    double tmpY = sin(deltaLongitude) * cos(finalLatitude);
    double tmpX = cos(originLatitude) * sin(finalLatitude) -
            sin(originLatitude) * cos(finalLatitude) *
            cos(deltaLongitude);
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
    return newPositionFromCompass(newObject, distance, math::polarToCompassBearing(bearing));
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

    double newLat = asin(sin(latitudeRad) * cos(distanceRatio) + cos(latitudeRad) * sin(distanceRatio) * cos(bearing));
    double newLon = longitudeRad + atan2(sin(bearing) * sin(distanceRatio) * cos(latitudeRad),
                                         cos(distanceRatio) - sin(latitudeRad) * sin(newLat));
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

} //end of namespace pose
} //end of namespace mace
