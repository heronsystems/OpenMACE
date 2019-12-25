#ifndef TWIST_H
#define TWIST_H

#include "velocity_interface_rotational.h"
#include "velocity_helper.h"

namespace mace{
namespace pose{

MACE_CLASS_FORWARD(Twist);

class Twist
{

public:
    Twist();

    ~Twist();

public:
    Velocity_Cartesian3D m_TranslationalVelocity;
    Velocity_Rotation3D m_RotationalVelocity;
};

} //end of namespace pose
} //end of namespace mace

#endif // TWIST_H
