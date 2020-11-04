#ifndef VELOCITY_HELPER_H
#define VELOCITY_HELPER_H

#include <Eigen/Core>
#include <type_traits>

#include "velocity_interface_translational.h"

namespace mace{
namespace pose{

template<const CoordinateSystemTypes coordType, typename CFDATA, class DATA = Eigen::Vector2d>
class VelocityHelper: public VelocityInterface_Translational<coordType, CFDATA, DATA>
{

};

template<const CoordinateSystemTypes coordType, typename CFDATA>
class VelocityHelper<coordType, CFDATA, Eigen::Vector2d> : public VelocityInterface_Translational<coordType, CFDATA, Eigen::Vector2d>
{
public:
    VelocityHelper():
        VelocityInterface_Translational<coordType, CFDATA, Eigen::Vector2d>()
    {
        this->dimension = 2;
    }

    VelocityHelper(const CFDATA &frame):
        VelocityInterface_Translational<coordType, CFDATA, Eigen::Vector2d>(frame)
    {
        this->dimension = 2;
    }

    VelocityHelper(const VelocityHelper &copy):
        VelocityInterface_Translational<coordType, CFDATA, Eigen::Vector2d>(copy)
    {

    }

    ~VelocityHelper() = default;

public:
    void setXVelocity(const double &value)
    {
        this->data(0) = value;
        this->validateDimension(Velocity::IGNORE_X_DIMENSION);
    }

    void setYVelocity(const double &value)
    {
        this->data(1) = value;
        this->validateDimension(Velocity::IGNORE_Y_DIMENSION);
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
        if((this->dimensionMask&Velocity::IGNORE_X_DIMENSION) == 0)
            return true;
        return false;
    }

    bool hasYBeenSet() const
    {
        if((this->dimensionMask&Velocity::IGNORE_Y_DIMENSION) == 0)
            return true;
        return false;
    }
};

template<const CoordinateSystemTypes coordType, typename CFDATA>
class VelocityHelper<coordType, CFDATA, Eigen::Vector3d> : public VelocityInterface_Translational<coordType, CFDATA, Eigen::Vector3d>
{
public:
    VelocityHelper():
        VelocityInterface_Translational<coordType, CFDATA, Eigen::Vector3d>()
    {
        this->dimension = 3;
    }

    VelocityHelper(const CFDATA &frame):
        VelocityInterface_Translational<coordType, CFDATA, Eigen::Vector3d>(frame)
    {
        this->dimension = 3;
    }

    VelocityHelper(const VelocityHelper &copy):
        VelocityInterface_Translational<coordType, CFDATA, Eigen::Vector3d>(copy)
    {

    }

    ~VelocityHelper() = default;

public:
    void setXVelocity(const double &value)
    {
        this->data(0) = value;
        this->validateDimension(Velocity::IGNORE_X_DIMENSION);
    }

    void setYVelocity(const double &value)
    {
        this->data(1) = value;
        this->validateDimension(Velocity::IGNORE_Y_DIMENSION);
    }

    void setZVelocity(const double &value)
    {
        this->data(2) = value;
        this->validateDimension(Velocity::IGNORE_Z_DIMENSION);
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
        return this->data(2);
    }

    bool hasXBeenSet() const
    {
        if((this->dimensionMask&Velocity::IGNORE_X_DIMENSION) == 0)
            return true;
        return false;
    }

    bool hasYBeenSet() const
    {
        if((this->dimensionMask&Velocity::IGNORE_Y_DIMENSION) == 0)
            return true;
        return false;
    }

    bool hasZBeenSet() const
    {
        if((this->dimensionMask&Velocity::IGNORE_Z_DIMENSION) == 0)
            return true;
        return false;
    }

};


typedef VelocityHelper<CoordinateSystemTypes::CARTESIAN, CartesianFrameTypes, Eigen::Vector2d> Velocity_Cartesian2D;
typedef VelocityHelper<CoordinateSystemTypes::CARTESIAN, CartesianFrameTypes, Eigen::Vector3d> Velocity_Cartesian3D;


} // end of namespace pose
} // end of namespace mace

#endif // VELOCITY_HELPER
