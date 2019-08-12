#ifndef VELOCITY_HELPER_H
#define VELOCITY_HELPER_H

#include <Eigen/Core>
#include <type_traits>

#include "velocity_interface.h"

namespace mace{
namespace pose{

template<const CoordinateSystemTypes coordType, typename CFDATA, class DATA>
class VelocityHelper: public VelocityInterface<coordType, CFDATA, DATA>
{

};

template<const CoordinateSystemTypes coordType, typename CFDATA>
class VelocityHelper : public VelocityInterface<coordType, CFDATA, Eigen::Vector2d>
{
    void setXVelocity(const double &value)
    {
        this->data(0) = value;
        this->validateDimension(IGNORE_X_DIMENSION);
    }

    void setYVelocity(const double &value)
    {
        this->data(1) = value;
        this->validateDimension(IGNORE_Y_DIMENSION);
    }

    double getXVelocity() const
    {
        return this->data(0);
    }

    double getYVelocity() const
    {
        return this->data(1);
    }

    bool hasXBeenSet() const
    {
        if((this->dimensionMask&IGNORE_X_DIMENSION) == 0)
            return true;
        return false;
    }

    bool hasYBeenSet() const
    {
        if((this->dimensionMask&IGNORE_Y_DIMENSION) == 0)
            return true;
        return false;
    }
};

template<const CoordinateSystemTypes coordType, typename CFDATA>
class VelocityHelper : public VelocityInterface<coordType, CFDATA, Eigen::Vector3d>
{
    void setXVelocity(const double &value)
    {
        this->data(0) = value;
        this->validateDimension(IGNORE_X_DIMENSION);
    }

    void setYVelocity(const double &value)
    {
        this->data(1) = value;
        this->validateDimension(IGNORE_Y_DIMENSION);
    }

    void setZVelocity(const double &value)
    {
        this->data(2) = value;
        this->validateDimension(IGNORE_Z_DIMENSION);
    }

    double getXVelocity() const
    {
        return this->data(0);
    }

    double getYVelocity() const
    {
        return this->data(1);
    }

    double getZVelocity() const
    {
        return this->data(1);
    }

    bool hasXBeenSet() const
    {
        if((this->dimensionMask&IGNORE_X_DIMENSION) == 0)
            return true;
        return false;
    }

    bool hasYBeenSet() const
    {
        if((this->dimensionMask&IGNORE_Y_DIMENSION) == 0)
            return true;
        return false;
    }

    bool hasZBeenSet() const
    {
        if((this->dimensionMask&IGNORE_Z_DIMENSION) == 0)
            return true;
        return false;
    }
};


typedef VelocityHelper<CoordinateSystemTypes::CARTESIAN, CartesianFrameTypes, Eigen::Vector2d> Cartesian_Velocity2D;
typedef VelocityHelper<CoordinateSystemTypes::CARTESIAN, CartesianFrameTypes, Eigen::Vector3d> Cartesian_Velocity3D;


} // end of namespace pose
} // end of namespace mace

#endif // VELOCITY_HELPER
