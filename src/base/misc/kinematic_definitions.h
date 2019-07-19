#ifndef KINEMATIC_STATES_H
#define KINEMATIC_STATES_H

#include <string>

enum class KinematicTypes: uint8_t{
    POSITION,
    VELOCITY,
    ACCELERATION,
    ORIENTATION,
    ORIENATION_RATE,
    ORIENTATION_ACCEL
};

enum class CoordinateSystemTypes : uint8_t{
    CARTESIAN = 0,
    GEODETIC = 1,
    UNKNOWN = 2
};

class Kinematic_BaseInterface
{
public:
    virtual ~Kinematic_BaseInterface() = default;

    virtual KinematicTypes getKinematicType() const = 0;

    virtual CoordinateSystemTypes getCoordinateSystemType() const = 0;
};

#endif // KINEMATIC_STATES_H
