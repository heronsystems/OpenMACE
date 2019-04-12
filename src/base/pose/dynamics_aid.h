#ifndef DYNAMICS_AID_H
#define DYNAMICS_AID_H

#include "../math/helper_pi.h"

#include "cartesian_position_2D.h"
#include "cartesian_position_3D.h"

#include "geodetic_position_2D.h"
#include "geodetic_position_3D.h"

namespace mace {
namespace pose {

class DynamicsAid
{
public:
    DynamicsAid() = default;

    ~DynamicsAid() = default;

    //!
    //! \brief GlobalPositionToLocal
    //! \param origin
    //! \param position
    //! \param local
    //!
    static void GlobalPositionToLocal(const GeodeticPosition_3D &origin, const GeodeticPosition_3D &position, CartesianPosition_3D &local);

    //!
    //! \brief LocalPositionToGlobal
    //! \param origin
    //! \param position
    //! \param global
    //!
    static void LocalPositionToGlobal(const GeodeticPosition_3D &origin, const CartesianPosition_3D &position, GeodeticPosition_3D&global);
};

} //end of namespace pose
} //end of namespace mace

#endif // DYNAMICS_AID_H
