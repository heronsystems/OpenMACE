#ifndef BASE_VELOCITY_H
#define BASE_VELOCITY_H

#include <Eigen/Core>
#include <type_traits>

#include "abstract_velocity.h"

namespace mace{
namespace pose{

template<typename CFDATA, class DATA>
class Base_Velocity: public Abstract_Velocity
{
public:
    Base_Velocity(const CFDATA &frame):
        Abstract_Velocity(), explicitFrame(frame)
    {
        if(typeid (CFDATA).name() == typeid (CartesianFrameTypes).name())
        {
            explicitType = CoordinateSystemTypes::CARTESIAN;
        }
    }

    Base_Velocity(const Base_Velocity &copy):
        Abstract_Velocity(copy)
    {
        explicitFrame = copy.explicitFrame;
    }

    Eigen::VectorXd getDataVector() const override
    {
        return this->data;
    }
    void updateDataVector(const Eigen::VectorXd &vecObj) const override
    {

    }


public:
    Abstract_Velocity* getVelocityClone() const override
    {
        return (new Base_Velocity<CFDATA,DATA>(*this));
    }

    void getVelocityClone(Abstract_Velocity** state) const override
    {
        *state = new Base_Velocity<CFDATA,DATA>(*this);
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

typedef Base_Velocity<CartesianFrameTypes, Eigen::Vector2d> Cartesian_Velocity2D;
typedef Base_Velocity<CartesianFrameTypes, Eigen::Vector3d> Cartesian_Velocity3D;


typedef Base_Velocity<GeodeticFrameTypes, Eigen::Vector2d> Geodetic_Velocity2D;
typedef Base_Velocity<GeodeticFrameTypes, Eigen::Vector3d> Geodetic_Velocity3D;

} // end of namespace pose
} // end of namespace mace

#endif // BASE_VELOCITY_H
