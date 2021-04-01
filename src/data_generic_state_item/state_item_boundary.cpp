#include "state_item_boundary.h"

namespace DataState {

StateItem_Boundary::StateItem_Boundary() {
}

StateItem_Boundary::StateItem_Boundary(const StateItem_Boundary &copyObj)
{
    this->boundaryVerts = copyObj.boundaryVerts;
}

} //end of namespace DataGenericItem
