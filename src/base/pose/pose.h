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

    void setTimeNow();

public:
    Data::EnvironmentTime m_UpdateTime;

    CartesianPosition_3D m_Position;
    Rotation_3D m_Rotation;
};

} //end of namespace pose
} //end of namespace mace
#endif // POSE_H
