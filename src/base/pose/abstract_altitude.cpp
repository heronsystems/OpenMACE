#include "abstract_altitude.h"

using namespace mace::pose;

Abstract_Altitude::Abstract_Altitude(const AltitudeReferenceTypes &explicitFrame):
    AltitudeInterface(), misc::Data1D(), altitudeFrameType(explicitFrame)
{

}

Abstract_Altitude::Abstract_Altitude(const AltitudeReferenceTypes &explicitFrame, const double &z):
     AltitudeInterface(), misc::Data1D (z), altitudeFrameType(explicitFrame)
{

}


Abstract_Altitude::Abstract_Altitude(const Abstract_Altitude &copy):
    AltitudeInterface(), misc::Data1D (copy)
{
    this->altitudeFrameType = copy.altitudeFrameType;
}


void Abstract_Altitude::setAltitude(const double &altitude)
{
    this->setData_1D(altitude);
}

double Abstract_Altitude::getAltitude() const
{
    return this->getZ();
}

double Abstract_Altitude::deltaAltitude(const Abstract_Altitude *pos) const
{
    return this->getZ() - pos->getZ();
}

bool Abstract_Altitude::hasAltitudeBeenSet() const
{
    return this->getDataZFlag();
}

double Abstract_Altitude::elevationFromOrigin() const
{
    return 0.0;
}

void Abstract_Altitude::setAltitudeReferenceFrame(const AltitudeReferenceTypes &explicitType)
{
    this->altitudeFrameType = explicitType;
}

AltitudeReferenceTypes Abstract_Altitude::getAltitudeReferenceFrame() const
{
    return this->altitudeFrameType;
}

bool Abstract_Altitude::areEquivalentAltitudeFrames(const Abstract_Altitude &obj) const
{
    return this->altitudeFrameType == obj.getAltitudeReferenceFrame();
}
