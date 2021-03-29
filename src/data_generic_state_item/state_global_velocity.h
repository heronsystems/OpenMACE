#ifndef STATE_GLOBAL_VELOCITY_H
#define STATE_GLOBAL_VELOCITY_H

#include <iostream>

namespace DataState {

class StateGlobalVelocity
{
public:
    StateGlobalVelocity();

    void operator = (const StateGlobalVelocity &rhs)
    {
        this->x = rhs.x;
        this->y = rhs.y;
        this->z = rhs.z;
    }

    bool operator == (const StateGlobalVelocity &rhs) {
        if(this->x != rhs.x){
            return false;
        }
        if(this->y != rhs.y){
            return false;
        }
        if(this->z != rhs.z){
            return false;
        }
        return true;
    }

    bool operator != (const StateGlobalVelocity &rhs) {
        return !(*this == rhs);
    }

public:
    float x;
    float y;
    float z;
    float heading;
};

} //end of namespace DataState

#endif // STATE_GLOBAL_VELOCITY_H
