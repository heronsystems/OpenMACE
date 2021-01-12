#include "trackangle.h"

namespace mace {
namespace measurements {

TrackAngle::TrackAngle() :
    TrackAngle(0)
{

}

TrackAngle::TrackAngle(const int &target) :
    angle(0),
    targetID(target)
{

}

TrackAngle::TrackAngle(const TrackAngle &copy)
{
    this->targetID = copy.targetID;
    this->angle = copy.angle;
}

//!
//! \brief calculateFromState Calculate the trackangle based on current pose/attitude and target pose, and store it
//! \param target Global position of the target
//! \param ownPose Global position of the requesting aircraft
//! \param ownAttitude Attitude of the requesting aircraft
//!
void TrackAngle::calculateFromState(const pose::GeodeticPosition_3D &target, const pose::GeodeticPosition_3D &ownPose, const pose::Rotation_3D &ownAttitude)
{
//    pose::CartesianPosition_3D targetLocalPosition;
//    pose::DynamicsAid::GlobalPositionToLocal(&ownPose,&target,&targetLocalPosition);
//    misc::Data3D targetVector(targetLocalPosition.getXPosition(),targetLocalPosition.getYPosition(),targetLocalPosition.getZPosition());
//    misc::Data3D attitudeVector(cos(ownAttitude.getPitch())*sin(ownAttitude.getYaw()),cos(ownAttitude.getPitch())*cos(ownAttitude.getYaw()),sin(ownAttitude.getPitch()));
//    this->setAngle(acos(attitudeVector.norm().dot(targetVector.norm())));
}

//!
//! \brief setAngle
//! \param angle
//!
void TrackAngle::setAngle(const double &newangle)
{
    this->angle = newangle;
}

//!
//! \brief getAngle
//! \return
//!
double TrackAngle::getAngle() const
{
    return this->angle;
}

//!
//! \brief setTarget
//! \param targetID
//!
void TrackAngle::setTarget(const int &target)
{
    this->targetID = target;
}

//!
//! \brief getTarget
//! \return
//!
int TrackAngle::getTarget() const
{
    return this->targetID;
}


} //end of namespace measurement
} //end of namespace mace
