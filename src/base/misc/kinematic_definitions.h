#ifndef KINEMATIC_STATES_H
#define KINEMATIC_STATES_H

#include <string>

enum class KinematicTypes: uint8_t{
    POSITION = 0,
    VELOCITY = 1,
    ACCELERATION = 2,
    ORIENTATION = 3,
    ORIENATION_RATE = 4,
    ORIENTATION_ACCEL = 5
};

enum class PositionTypes: uint8_t{
    TRANSLATIONAL = 0,
    ELEVATION = 1
};

enum class CoordinateSystemTypes : uint8_t{
    CARTESIAN = 0,
    GEODETIC = 1,
    NOT_IMPLIED = 2,
    UNKNOWN = 3
};

class Kinematic_BaseInterface
{
public:
    virtual ~Kinematic_BaseInterface() = default;

    virtual KinematicTypes getKinematicType() const = 0;

    virtual CoordinateSystemTypes getCoordinateSystemType() const = 0;

public:

    uint8_t getDimension() const
    {
        return this->dimension;
    }

    //!
    //! \brief is3D
    //! \return
    //!
    virtual bool isGreaterThan1D() const
    {
        return dimension > 1;
    }

    //!
    //! \brief is3D
    //! \return
    //!
    virtual bool is2D() const
    {
        return dimension == 2;
    }

    //!
    //! \brief is3D
    //! \return
    //!
    virtual bool isGreaterThan2D() const
    {
        return dimension > 2;
    }

    //!
    //! \brief is3D
    //! \return
    //!
    virtual bool is3D() const
    {
        return dimension == 3;
    }

public:
    uint8_t dimension = 0;
};

#endif // KINEMATIC_STATES_H
