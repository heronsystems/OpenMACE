#ifndef CARTESIAN_VELOCITY_3D_H
#define CARTESIAN_VELOCITY_3D_H

#include <Eigen/Core>

#include "base/state_space/state.h"
#include "base/misc/data_3d.h"

#include "abstract_velocity.h"

namespace mace {
namespace pose {

class CartesianVelocity_3D: public AbstractVelocity<CartesianVelocity_3D, misc::Data3D>, public state_space::State
{
public:
    CartesianVelocity_3D():
        AbstractVelocity(AbstractVelocity::VelocityType::CARTESIAN, CoordinateFrameTypes::CF_LOCAL_ENU)
    {

    }

    ~CartesianVelocity_3D() override = default;

    CartesianVelocity_3D(const CartesianVelocity_3D &copy):
        AbstractVelocity(copy), state_space::State(copy)
    {

    }

    CartesianVelocity_3D(const double x, const double &y, const double &z):
        AbstractVelocity(AbstractVelocity::VelocityType::CARTESIAN, CoordinateFrameTypes::CF_LOCAL_ENU)
    {
        this->data.setData_2D(x,y,z);
    }

    State* getStateClone() const override
    {
        return (new CartesianVelocity_3D(*this));
    }

    void getStateClone(State** state) const override
    {
        *state = new CartesianVelocity_3D(*this);
    }

public:
    void updateVelocity(const double &xVel, const double &yVel, const double &zVel)
    {
        this->data.setData_2D(xVel,yVel,zVel);
    }

    void setXVelocity(const double &xVel)
    {
        this->data.setX(xVel);
    }

    void setYVelocity(const double &yVel)
    {
        this->data.setY(yVel);
    }

    void setZVelocity(const double &zVel)
    {
        this->data.setZ(zVel);
    }

    double getXVelocity() const
    {
        return this->data.getX();
    }

    double getYVelocity() const
    {
        return this->data.getY();
    }

    double getZVelocity() const
    {
        return this->data.getZ();
    }

    Eigen::Vector3d getAsVector()
    {
        Eigen::Vector3d vec(this->data.getX(), this->data.getY(), this->data.getZ());
        return vec;
    }

    bool hasXVelocityBeenSet() const
    {
        return this->data.getDataXFlag();
    }

    bool hasYVelocityBeenSet() const
    {
        return this->data.getDataYFlag();
    }

    bool hasZVelocityBeenSet() const
    {
        return this->data.getDataZFlag();
    }

};

} //end of namespace pose
} //end of namespace mace

#endif // CARTESIAN_VELOCITY_3D_H
