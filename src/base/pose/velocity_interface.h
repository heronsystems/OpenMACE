#ifndef VELOCITY_INTERFACE_H
#define VELOCITY_INTERFACE_H

#include <Eigen/Core>
#include <type_traits>

#include "abstract_velocity.h"

namespace mace{
namespace pose{

template<const CoordinateSystemTypes coordType, typename CFDATA, class DATA>
class VelocityInterface: public Velocity
{
public:
    VelocityInterface(const CFDATA &frame):
        Velocity(), explicitFrame(frame)
    {
        explicitType = coordType;
    }

    VelocityInterface(const VelocityInterface &copy):
        Velocity(copy)
    {
        explicitType = copy.explicitType;
        explicitFrame = copy.explicitFrame;
        data = copy.data;
    }

    Eigen::VectorXd getDataVector() const override
    {
        return this->data;
    }
    void updateDataVector(const Eigen::VectorXd &vecObj) const override
    {
        this->data = vecObj;
    }


public:
    Velocity* getVelocityClone() const override
    {
        return (new VelocityInterface<coordType, CFDATA, DATA>(*this));
    }

    void getVelocityClone(Velocity** state) const override
    {
        *state = new VelocityInterface<coordType, CFDATA, DATA>(*this);
    }

    CoordinateSystemTypes getCoordinateSystemType() const override
    {
        return explicitType;
    }


public:
    CoordinateFrameTypes getExplicitCoordinateFrame() const
    {
        return getCoordinateFrame(explicitFrame);
    }

    void setExplicitCoordinateFrame(const CFDATA &frame)
    {
        explicitFrame = frame;
    }

private:
    CoordinateSystemTypes explicitType = CoordinateSystemTypes::UNKNOWN;
    CFDATA explicitFrame;

public:
    DATA data;
};


} // end of namespace pose
} // end of namespace mace

#endif // VELOCITY_INTERFACE_H
