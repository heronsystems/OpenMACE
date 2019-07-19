#include "abstract_altitude.h"

using namespace mace::pose;

Abstract_Altitude::Abstract_Altitude(const AltitudeReferenceTypes &explicitFrame):
    AltitudeInterface(), altitudeFrameType(explicitFrame)
{

}

Abstract_Altitude::Abstract_Altitude(const Abstract_Altitude &copy):
    AltitudeInterface()
{
    this->altitudeFrameType = copy.altitudeFrameType;
}

void Abstract_Altitude::setAltitudeReferenceFrame(const AltitudeReferenceTypes &explicitType)
{
    this->altitudeFrameType = explicitType;
}

mace::AltitudeReferenceTypes Abstract_Altitude::getAltitudeReferenceFrame() const
{
    return this->altitudeFrameType;
}

bool Abstract_Altitude::areEquivalentAltitudeFrames(const Abstract_Altitude* obj) const
{
    return this->altitudeFrameType == obj->getAltitudeReferenceFrame();
}

double Abstract_Altitude::deltaAltitude(const Abstract_Altitude *pos) const
{
    if(this->areEquivalentAltitudeFrames(pos))
        return this->getAltitude() - pos->getAltitude();
    else
        throw std::logic_error("The deltaAltitude operation method has been called with incompatible coordinate frames.");
}

