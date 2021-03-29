#ifndef POSITIONAL_AID_H
#define POSITIONAL_AID_H

#include <cmath>
#include <Eigen/Dense>

#include "state_local_position.h"
#include "state_global_position.h"

namespace DataState {

/**
 * @brief The PositionalAid class provides functions to convert between positional frames.
 *
 * @see StateLocalPosition, StateGlobalPosition.
 */

class PositionalAid
{
public:
    //!
    //! \brief PositionalAid
    //!
    PositionalAid();

    //!
    ~PositionalAid();

    //!
    //! \brief GlobalPositionToLocal
    //! \param origin
    //! \param position
    //! \param local
    //!
    static void GlobalPositionToLocal(const StateGlobalPosition &origin, const StateGlobalPosition &position, StateLocalPosition &local);

    //!
    //! \brief LocalPositionToGlobal
    //! \param origin
    //! \param position
    //! \param global
    //!
    static void LocalPositionToGlobal(const StateGlobalPosition &origin, const StateLocalPosition &position, StateGlobalPosition&global);
};

} //end of namespace DataState
#endif // POSITIONAL_AID_H
