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

    //!
    //! \brief GlobalPositionToLocal
    //! \param origin
    //! \param position
    //! \param local
    //!
    static void GlobalPositionToLocal(const Abstract_GeodeticPosition* origin, const Abstract_GeodeticPosition* refPosition, Abstract_CartesianPosition* targetPosition);

    //!
    //! \brief LocalPositionToGlobal
    //! \param origin
    //! \param position
    //! \param global
    //!
    static void LocalPositionToGlobal(const Abstract_GeodeticPosition* origin, const Abstract_CartesianPosition* refPosition, Abstract_GeodeticPosition* targetPosition);
};

} //end of namespace pose
} //end of namespace mace

#endif // DYNAMICS_AID_H
