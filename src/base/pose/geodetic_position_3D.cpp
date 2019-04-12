#include "geodetic_position_3D.h"

namespace mace{
namespace pose{

double GeodeticPosition_3D::deltaLatitude(const GeodeticPosition_3D &that) const
{
    return this->getLatitude() - that.getLatitude();
}

double GeodeticPosition_3D::deltaLongitude(const GeodeticPosition_3D &that) const
{
    return this->getLongitude() - that.getLongitude();
}

//!
//! \brief deltaAltitude
//! \param position
//! \return
//!
double GeodeticPosition_3D::deltaAltitude(const GeodeticPosition_3D &position) const
{
    return this->getAltitude() - position.getAltitude();
}

double GeodeticPosition_3D::distanceFromOrigin() const
{
    GeodeticPosition_3D origin;
    return distanceBetween3D(origin);
}

double GeodeticPosition_3D::polarBearingFromOrigin() const
{
    GeodeticPosition_3D origin;
    return origin.polarBearingTo(*this);
}


double GeodeticPosition_3D::distanceBetween2D(const GeodeticPosition_3D &position) const
{
    double earthRadius = 6371000; //approximate value in meters

    double originLatitude = convertDegreesToRadians(this->getLatitude());
    double originLongitude = convertDegreesToRadians(this->getLongitude());
    double finalLatitude = convertDegreesToRadians(position.getLatitude());
    double finalLongitude = convertDegreesToRadians(position.getLongitude());

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
//! \brief distanceBetween3D
//! \param position
//! \return
//!
double GeodeticPosition_3D::distanceBetween3D(const GeodeticPosition_3D &position) const
{
    return sqrt(pow(distanceBetween2D(position),2) + pow(deltaAltitude(position),2));
}

double GeodeticPosition_3D::distanceTo(const GeodeticPosition_3D &pos) const
{
    return this->distanceBetween3D(pos);
}

//!
//! \brief GeodeticPosition_3D::polarBearingTo
//! \param pos
//! \return polar
//!
double GeodeticPosition_3D::polarBearingTo(const GeodeticPosition_3D &pos) const
{
    double bearing = compassBearingTo(pos);
    return compassToPolarBearing(bearing);
}

//!
//! \brief GeodeticPosition_3D::compassBearingTo
//! \param pos
//! \return polar
//!
double GeodeticPosition_3D::compassBearingTo(const GeodeticPosition_3D &pos) const
{
    double originLatitude = convertDegreesToRadians(this->getLatitude());
    double originLongitude = convertDegreesToRadians(this->getLongitude());
    double finalLatitude = convertDegreesToRadians(pos.getLatitude());
    double finalLongitude = convertDegreesToRadians(pos.getLongitude());

    double deltaLongitude = finalLongitude - originLongitude;

    double tmpY = sin(deltaLongitude) * cos(finalLatitude);
    double tmpX = cos(originLatitude) * sin(finalLatitude) -
            sin(originLatitude) * cos(finalLatitude) *
            cos(deltaLongitude);
    double bearing = atan2(tmpY,tmpX);
    return correctBearing(bearing);
}

double GeodeticPosition_3D::elevationAngleTo(const GeodeticPosition_3D &position) const
{
    //Ken we need to be careful here as there is a discontinuity
    double distance2D = distanceBetween2D(position);
    return atan2(deltaAltitude(position),distance2D);
}

//!
//! \brief GeodeticPosition_3D::newPositionFromPolar
//! \param distance
//! \param bearing
//! \return
//!
GeodeticPosition_3D GeodeticPosition_3D::newPositionFromPolar(const double &distance, const double &bearing) const
{
    return newPositionFromCompass(distance,polarToCompassBearing(bearing));
}

GeodeticPosition_3D GeodeticPosition_3D::newPositionFromPolar(const double &distance, const double &bearing, const double &elevation) const
{
    double distanceZ = distance * sin(elevation);
    double distanceXY = distance * cos(elevation);
    GeodeticPosition_3D newPosition = newPositionFromCompass(distanceXY,polarToCompassBearing(bearing));
    newPosition.setAltitude(distanceZ + this->getAltitude());
    return newPosition;
}

//!
//! \brief GeodeticPosition_3D::newPositionFromCompass
//! \param distance
//! \param bearing
//! \return
//!
GeodeticPosition_3D GeodeticPosition_3D::newPositionFromCompass(const double &distance, const double &bearing) const
{
    double earthRadius = 6371000; //approximate value in meters
    double latitudeRad = convertDegreesToRadians(this->getLatitude());
    double longitudeRad = convertDegreesToRadians(this->getLongitude());

    double distanceRatio = distance / earthRadius;

    double newLat = asin(sin(latitudeRad) * cos(distanceRatio) + cos(latitudeRad) * sin(distanceRatio) * cos(bearing));
    double newLon = longitudeRad + atan2(sin(bearing) * sin(distanceRatio) * cos(latitudeRad),
                                         cos(distanceRatio) - sin(latitudeRad) * sin(newLat));

    GeodeticPosition_3D newPos(convertRadiansToDegrees(newLat),convertRadiansToDegrees(newLon), this->getAltitude());

    return newPos;
}

void GeodeticPosition_3D::applyPositionalShiftFromPolar(const double &distance, const double &bearing)
{
    GeodeticPosition_3D newPosition = newPositionFromPolar(distance,bearing);
    this->setLatitude(newPosition.getLatitude());
    this->setLongitude(newPosition.getLongitude());
}

void GeodeticPosition_3D::applyPositionalShiftFromCompass(const double &distance, const double &bearing)
{
    GeodeticPosition_3D newPosition = newPositionFromCompass(distance,bearing);
    this->setLatitude(newPosition.getLatitude());
    this->setLongitude(newPosition.getLongitude());
}

} //end of namespace pose
} //end of namespace mace
