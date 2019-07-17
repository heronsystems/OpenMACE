#include "abstract_altitude.h"

using namespace mace::pose;

Abstract_Altitude::Abstract_Altitude(const AltitudeReferenceTypes &explicitFrame):
    AltitudeInterface()
{

}

Abstract_Altitude::Abstract_Altitude(const AltitudeReferenceTypes &explicitFrame, const double &z):
    altitudeFrameType(explicitFrame), AltitudeInterface(z)
{

}


Abstract_Altitude::Abstract_Altitude(const Abstract_Altitude &copy):
    AltitudeInterface(copy)
{
    this->altitudeFrameType = copy.altitudeFrameType;
}

AltitudeReferenceTypes Abstract_Altitude::getAltitudeReferenceFrame() const
{
    return this->altitudeFrameType;
}
