#ifndef KINEMATIC_STATES_H
#define KINEMATIC_STATES_H

#include <string>

enum class KinematicTypes: uint8_t{
    POSITION = 0,
    VELOCITY = 1,
    ACCELERATION = 2,
    ORIENTATION = 3,
};

enum class PositionTypes: uint8_t{
    TRANSLATIONAL = 0,
    ELEVATION = 1
};

enum class VelocityTypes: uint8_t{
    TRANSLATIONAL = 0,
    ROTATIONAL = 1
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

    Kinematic_BaseInterface();

    Kinematic_BaseInterface(const Kinematic_BaseInterface &copy);

    virtual ~Kinematic_BaseInterface();

    virtual KinematicTypes getKinematicType() const = 0;

    virtual CoordinateSystemTypes getCoordinateSystemType() const = 0;

public:
    //!
    //! \brief setDimensionMask
    //! \param mask
    //!
    void setDimensionMask(const uint16_t &mask)
    {
        this->dimensionMask = mask;
    }

    //!
    //! \brief orDimensionMask
    //! \param orValue
    //!
    void orDimensionMask(const uint16_t &orValue)
    {
        this->dimensionMask = dimensionMask|orValue;
    }

    void xorDimensionMask(const uint16_t &xorValue)
    {
        this->dimensionMask = this->dimensionMask^xorValue;
    }

    void validateDimension(const uint16_t &validValue)
    {
        this->dimensionMask = this->dimensionMask & (~validValue);
    }
    //!
    //! \brief getDimensionMask
    //! \return
    //!
    uint16_t getDimensionMask() const
    {
        return this->dimensionMask;
    }
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

    uint16_t dimensionMask = 0;

};

#endif // KINEMATIC_STATES_H
