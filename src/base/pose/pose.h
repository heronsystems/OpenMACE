#ifndef POSE_H
#define POSE_H

#include "abstract_position.h"
#include "abstract_rotation.h"

namespace mace{
namespace pose{

MACE_CLASS_FORWARD(Pose);

class Pose
{

public:
    Pose();

public:
    Position* m_Position;
    AbstractRotation* m_Rotation;
};

} //end of namespace pose
} //end of namespace mace
#endif // POSE_H
