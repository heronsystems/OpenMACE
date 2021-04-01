#ifndef STATE_LOCAL_VELOCITY_H
#define STATE_LOCAL_VELOCITY_H

#include <iostream>

namespace DataState {

class StateLocalVelocity
{
public:
    StateLocalVelocity();

public:
    void operator = (const StateLocalVelocity &rhs)
    {
        this->x = rhs.x;
        this->y = rhs.y;
        this->z = rhs.z;
    }

    bool operator == (const StateLocalVelocity &rhs) {
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

    bool operator != (const StateLocalVelocity &rhs) {
        return !(*this == rhs);
    }

public:
    double x;
    double y;
    double z;
};

} //end of namespace DataState

#endif // STATE_LOCAL_VELOCITY_H
