#ifndef STATE_LOCAL_POSITION_EX_H
#define STATE_LOCAL_POSITION_EX_H

#include "data_generic_state_item/state_local_position.h"

namespace DataState{

class StateLocalPositionEx : public StateLocalPosition
{
public:
    StateLocalPositionEx();

    StateLocalPositionEx(const StateLocalPosition &localPosition, const double &localHeading);

public:
    void operator = (const StateLocalPositionEx &rhs)
    {
        StateLocalPosition::operator =(rhs);
        this->heading = rhs.heading;
    }

    bool operator == (const StateLocalPositionEx &rhs) {
        if(!StateLocalPosition::operator ==(rhs)){
            return false;
        }
        if(this->heading != rhs.heading){
            return false;
        }
        return true;
    }

    bool operator != (const StateLocalPositionEx &rhs) {
        return !(*this == rhs);
    }

public:
    double heading;

};

} //end of namespace DataState

#endif // STATE_LOCAL_POSITION_EX_H
