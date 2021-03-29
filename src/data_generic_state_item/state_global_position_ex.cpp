#include "state_global_position_ex.h"

namespace DataState {

StateGlobalPositionEx::StateGlobalPositionEx():
    StateGlobalPosition(),heading(0.0)
{

}

StateGlobalPositionEx::StateGlobalPositionEx(const StateGlobalPosition &globalPosition, const double &globalHeading):
    StateGlobalPosition(globalPosition),heading(globalHeading)
{

}

} //end of namespace DataState

