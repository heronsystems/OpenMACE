#ifndef STATE_GENERIC_POSITION_H
#define STATE_GENERIC_POSITION_H

#include "data/coordinate_frame.h"



namespace DataState {
class StateGenericPosition
{
public:

    void setCoordinateFrame(const Data::CoordinateFrameType &coordinateFrame){
        this->m_CoordinateFrame = coordinateFrame;
    }

    Data::CoordinateFrameType getCoordinateFrame() const {
        return m_CoordinateFrame;
    }

public:
    void operator = (const StateGenericPosition &rhs)
    {
        this->m_CoordinateFrame = rhs.m_CoordinateFrame;
    }

    bool operator == (const StateGenericPosition &rhs) {
        if(this->m_CoordinateFrame != rhs.m_CoordinateFrame){
            return false;
        }
        return true;
    }

    bool operator != (const StateGenericPosition &rhs) {
        return !(*this == rhs);
    }
protected:
    Data::CoordinateFrameType m_CoordinateFrame;
};

}

#endif // STATE_GENERIC_POSITION_H
