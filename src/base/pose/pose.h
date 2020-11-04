#ifndef POSE_H
#define POSE_H

#include "data/environment_time.h"
#include "cartesian_position_3D.h"
#include "rotation_3D.h"

namespace mace{
namespace pose{

MACE_CLASS_FORWARD(Pose);

class Pose
{

public:
    Pose();

    Pose(const Pose &copyObj);

    void setTimeNow();

public:
    Pose& operator = (const Pose &rhs)
    {
        this->m_Position = rhs.m_Position;
        this->m_Rotation = rhs.m_Rotation;
        this->m_UpdateTime = rhs.m_UpdateTime;
        return *this;
    }

    bool operator == (const Pose &rhs) const
    {

        if(this->m_UpdateTime != rhs.m_UpdateTime){
            return false;
        }
        if(this->m_Position != rhs.m_Position){
            return false;
        }
        if(this->m_Rotation != rhs.m_Rotation){
            return false;
        }
        return true;
    }

    bool operator !=(const Pose &rhs) const
    {
        return !(*this == rhs);
    }


public:
    Data::EnvironmentTime m_UpdateTime;

    CartesianPosition_3D m_Position;
    Rotation_3D m_Rotation;

};

} //end of namespace pose
} //end of namespace mace
#endif // POSE_H
