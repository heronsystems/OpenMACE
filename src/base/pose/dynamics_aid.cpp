#include "dynamics_aid.h"

namespace mace {
namespace pose {

//!
//! \brief PositionalAid::GlobalPositionToLocal
//! \param origin
//! \param position
//! \param local
//!
void DynamicsAid::GlobalPositionToLocal(const GeodeticPosition_3D &origin, const GeodeticPosition_3D &position, CartesianPosition_3D &local)
{
    double distance = origin.distanceBetween2D(position);
    double bearing = origin.compassBearingTo(position);
    double deltaAltitude = origin.deltaAltitude(position);
    local.applyPositionalShiftFromCompass(distance,convertDegreesToRadians(bearing));
    local.setZPosition(-deltaAltitude);
}

//!
//! \brief PositionalAid::LocalPositionToGlobal
//! \param origin
//! \param position
//! \param global
//!
void DynamicsAid::LocalPositionToGlobal(const GeodeticPosition_3D &origin, const CartesianPosition_3D &position, GeodeticPosition_3D &global)
{
    double distance = position.distanceFromOrigin();
    double bearing = position.polarBearingFromOrigin();
    double elevation = position.elevationFromOrigin();
    global = origin.newPositionFromPolar(distance,bearing,elevation);
}

} //end of namespace pose
} //end of namespace mace
