#include "state_local_position_ex.h"

namespace DataState {

StateLocalPositionEx::StateLocalPositionEx():
    StateLocalPosition(),heading(0.0)
{

}

StateLocalPositionEx::StateLocalPositionEx(const StateLocalPosition &localPosition, const double &localHeading):
    StateLocalPosition(localPosition),heading(localHeading)
{

}

} //end of namespace DataState
