#ifndef STATE_ITEM_BOUNDARY_H
#define STATE_ITEM_BOUNDARY_H

#include "data_generic_state_item/state_global_position.h"

#include <vector>

namespace DataState {

class StateItem_Boundary
{

public:
    StateItem_Boundary();

    StateItem_Boundary(const StateItem_Boundary &copyObj);

public:
    void operator = (const StateItem_Boundary &rhs)
    {
        this->boundaryVerts = rhs.boundaryVerts;
    }

    bool operator == (const StateItem_Boundary &rhs) {
        int counter = 0;
        for(auto vert : rhs.boundaryVerts) {
            if(vert != this->boundaryVerts[counter]) {
                return false;
            }
            counter++;
        }

        return true;
    }

    bool operator != (const StateItem_Boundary &rhs) {
        return !(*this == rhs);
    }

protected:
    std::vector<DataState::StateGlobalPosition> boundaryVerts;
};

} //end of namespace DataState

#endif // STATE_ITEM_BOUNDARY_H
