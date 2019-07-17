#include "geodetic_position_2D.h"
#include "geodetic_position_3D.h"

namespace mace{
namespace pose{

//!
//! \brief GeodeticPosition_3D::GeodeticPosition_3D
//! \param frameType
//! \param latitude
//! \param longitude
//! \param altitudeType
//! \param altitude
//! \param pointName
//!
GeodeticPosition_3D::GeodeticPosition_3D(const GeodeticFrameTypes &frameType,
                    const double &latitude, const double &longitude,
                    const AltitudeReferenceTypes &altitudeType, const double &altitude,
                    const std::string &pointName):
    Abstract_GeodeticPosition(frameType, latitude, longitude, pointName), Abstract_Altitude(altitudeType, altitude), state_space::State()
{
    this->dimension = 3;
}

//!
//! \brief GeodeticPosition_3D::GeodeticPosition_3D
//! \param pointName
//! \param latitude
//! \param longitude
//! \param altitude
//!
GeodeticPosition_3D::GeodeticPosition_3D(const std::string &pointName,
                    const double &latitude, const double &longitude, const double &altitude):
    Abstract_GeodeticPosition(GeodeticFrameTypes::CF_GLOBAL_RELATIVE_ALT, latitude, longitude, pointName), Abstract_Altitude(AltitudeReferenceTypes::REF_ALT_RELATIVE, altitude), state_space::State()
{
    this->dimension = 3;
}

//!
//! \brief GeodeticPosition_3D::GeodeticPosition_3D
//! \param latitude
//! \param longitude
//! \param altitude
//!
GeodeticPosition_3D::GeodeticPosition_3D(const double &latitude, const double &longitude, const double &altitude):
    Abstract_GeodeticPosition(GeodeticFrameTypes::CF_GLOBAL_RELATIVE_ALT, latitude, longitude), Abstract_Altitude(AltitudeReferenceTypes::REF_ALT_RELATIVE, altitude), state_space::State()
{

}

//!
//! \brief GeodeticPosition_3D::GeodeticPosition_3D
//! \param copy
//!
GeodeticPosition_3D::GeodeticPosition_3D(const GeodeticPosition_3D &copy):
    Abstract_GeodeticPosition(copy), Abstract_Altitude(copy), state_space::State(copy)

{

}

//!
//! \brief GeodeticPosition_3D::GeodeticPosition_3D
//! \param copy
//!
GeodeticPosition_3D::GeodeticPosition_3D(const GeodeticPosition_2D &copy):
    Abstract_GeodeticPosition(copy), Abstract_Altitude(), state_space::State(copy)
{
    this->dimension = 3;
}


bool GeodeticPosition_3D::areEquivalentFrames(const GeodeticPosition_3D &obj) const
{
    return this->areEquivalentGeodeticFrames(obj) && this->areEquivalentAltitudeFrames(obj);
}

//!
//! \brief distanceBetween3D
//! \param position
//! \return
//!
double GeodeticPosition_3D::distanceBetween3D(const GeodeticPosition_3D &position) const
{
    return sqrt(pow(distanceBetween2D(&position),2) + pow(deltaAltitude(&position),2));
}

double GeodeticPosition_3D::distanceFromOrigin() const
{
    GeodeticPosition_3D origin;
    return distanceBetween2D(&origin);
}

double GeodeticPosition_3D::polarBearingFromOrigin() const
{
    GeodeticPosition_3D origin;
    return origin.polarBearingTo(this);
}

double GeodeticPosition_3D::distanceBetween2D(const Abstract_GeodeticPosition* pos) const
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

double GeodeticPosition_3D::distanceTo(const Abstract_GeodeticPosition* pos) const
{
    return this->distanceBetween2D(pos);
}

//!
//! \brief GeodeticPosition_2D::polarBearingTo
//! \param pos
//! \return polar
//!
double GeodeticPosition_3D::polarBearingTo(const Abstract_GeodeticPosition* pos) const
{
    double bearing = compassBearingTo(pos);
    return math::compassToPolarBearing(bearing);
}

//!
//! \brief GeodeticPosition_2D::compassBearingTo
//! \param pos
//! \return polar
//!
double GeodeticPosition_3D::compassBearingTo(const Abstract_GeodeticPosition* pos) const
{
    double compassBearing = 0.0;

    if(pos->isGreaterThan1D())
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
        compassBearing = math::correctBearing(bearing);
    }

    return compassBearing;
}


//!
//! \brief GeodeticPosition_3D::newPositionFromPolar
//! \param distance
//! \param bearing
//! \return
//!
void GeodeticPosition_3D::newPositionFromPolar(Abstract_GeodeticPosition* newObject, const double &distance, const double &bearing) const
{
    return newPositionFromCompass(newObject, distance, math::polarToCompassBearing(bearing));
}


GeodeticPosition_3D GeodeticPosition_3D::newPositionFromPolar(const double &distance, const double &bearing, const double &elevation) const
{
    double distanceZ = distance * sin(elevation);
    double distanceXY = distance * cos(elevation);

    GeodeticPosition_3D newPosition;
    newPositionFromCompass(&newPosition, distanceXY, math::polarToCompassBearing(bearing));
    newPosition.setAltitude(distanceZ + this->getAltitude());

    return newPosition;
}

//!
//! \brief GeodeticPosition_3D::newPositionFromCompass
//! \param distance
//! \param bearing
//! \return
//!
void GeodeticPosition_3D::newPositionFromCompass(Abstract_GeodeticPosition* newObject, const double &distance, const double &bearing) const
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

void GeodeticPosition_3D::applyPositionalShiftFromPolar(const double &distance, const double &bearing)
{
    GeodeticPosition_2D tmpPosition;
    newPositionFromPolar(&tmpPosition, distance, bearing);
    this->setLatitude(tmpPosition.getLatitude());
    this->setLongitude(tmpPosition.getLongitude());
}

void GeodeticPosition_3D::applyPositionalShiftFromCompass(const double &distance, const double &bearing)
{
    GeodeticPosition_2D tmpPosition;
    newPositionFromCompass(&tmpPosition, distance, bearing);
    this->setLatitude(tmpPosition.getLatitude());
    this->setLongitude(tmpPosition.getLongitude());
}


double GeodeticPosition_3D::elevationAngleTo(const GeodeticPosition_3D &position) const
{
    double distance2D = distanceBetween2D(&position);
    double distanceAlt = deltaAltitude(&position);
    if(distance2D < std::numeric_limits<double>::epsilon() && distanceAlt < std::numeric_limits<double>::epsilon())
        return 0.0;
    else
        return atan2(distanceAlt,distance2D);
}


} //end of namespace pose
} //end of namespace mace
