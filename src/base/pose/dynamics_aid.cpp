#include "dynamics_aid.h"

namespace mace {
namespace pose {

//!
//! \brief PositionalAid::GlobalPositionToLocal
//! \param origin
//! \param position
//! \param local
//!
void DynamicsAid::GlobalPositionToLocal(const Abstract_GeodeticPosition* origin, const Abstract_GeodeticPosition* refPosition, Abstract_CartesianPosition* targetPosition)
{
    //First handle the translational components of the position object
    double distance = origin->distanceBetween2D(refPosition);
    double bearing = origin->compassBearingTo(refPosition);
    targetPosition->applyPositionalShiftFromCompass(distance,convertDegreesToRadians(bearing));

    //Second, handle the altitude component if the ref position and the origin contain the appropriate items
    if(targetPosition->is3D()) //Only if the object is considered a 3D position object can we do the following
    {
        CartesianPosition_3D* targetObj = targetPosition->positionAs<CartesianPosition_3D>();
        if(refPosition->is3D() && origin->is3D())
        {
            double deltaAltitude = origin->positionAs<GeodeticPosition_3D>()->deltaAltitude(refPosition->positionAs<GeodeticPosition_3D>());
            targetObj->setZPosition(-deltaAltitude);
        }
        else if(refPosition->is3D())
        {
            targetObj->setZPosition(refPosition->positionAs<GeodeticPosition_3D>()->getAltitude());
        }
    }

}

//!
//! \brief PositionalAid::LocalPositionToGlobal
//! \param origin
//! \param position
//! \param global
//!
void DynamicsAid::LocalPositionToGlobal(const Abstract_GeodeticPosition* origin, const Abstract_CartesianPosition* refPosition, Abstract_GeodeticPosition* targetPosition)
{
    if(targetPosition->is3D())
    {
        if(refPosition->is3D() && origin->is3D())
        {
            double distance = refPosition->distanceFromOrigin();
            double bearing = refPosition->polarBearingFromOrigin();
            double elevation = refPosition->positionAs<CartesianPosition_3D>()->elevationAngleFromOrigin();
            mace::pose::GeodeticPosition_3D newPosition = origin->positionAs<GeodeticPosition_3D>()->newPositionFromPolar(distance, bearing, elevation);
            targetPosition->positionAs<mace::pose::GeodeticPosition_3D>()->updateFromPosition(newPosition);
        }
        else if(refPosition->is3D() && origin->is2D())
        {
            double distance = refPosition->translationalDistanceFromOrigin();
            double bearing = refPosition->polarBearingFromOrigin();

            origin->newPositionFromPolar(targetPosition, distance, bearing);
            targetPosition->positionAs<GeodeticPosition_3D>()->setAltitude(refPosition->positionAs<CartesianPosition_3D>()->getAltitude());
        }
    }
    else if(targetPosition->is2D())
    {
        double distance = refPosition->translationalDistanceFromOrigin();
        double bearing = refPosition->polarBearingFromOrigin();
        origin->newPositionFromPolar(targetPosition, distance, bearing);
    }

}

} //end of namespace pose
} //end of namespace mace
