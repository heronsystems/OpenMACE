#ifndef BASE_VELOCITY_H
#define BASE_VELOCITY_H

#include <Eigen/Core>
#include <type_traits>

#include "abstract_velocity.h"

namespace mace{
namespace pose{

template<const CoordinateSystemTypes coordType, typename CFDATA, class DATA>
class Base_Velocity: public Velocity
{
public:
    Base_Velocity(const CFDATA &frame):
        Velocity(), explicitFrame(frame)
    {
        explicitType = coordType;
    }

    Base_Velocity(const Base_Velocity &copy):
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

    }


public:
    Velocity* getVelocityClone() const override
    {
        return (new Base_Velocity<coordType, CFDATA, DATA>(*this));
    }

    void getVelocityClone(Velocity** state) const override
    {
        *state = new Base_Velocity<coordType, CFDATA, DATA>(*this);
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

typedef Base_Velocity<CoordinateSystemTypes::CARTESIAN, CartesianFrameTypes, Eigen::Vector2d> Cartesian_Velocity2D;
typedef Base_Velocity<CoordinateSystemTypes::CARTESIAN, CartesianFrameTypes, Eigen::Vector3d> Cartesian_Velocity3D;


typedef Base_Velocity<CoordinateSystemTypes::GEODETIC, GeodeticFrameTypes, Eigen::Vector2d> Geodetic_Velocity2D;
typedef Base_Velocity<CoordinateSystemTypes::GEODETIC, GeodeticFrameTypes, Eigen::Vector3d> Geodetic_Velocity3D;

} // end of namespace pose
} // end of namespace mace

#endif // BASE_VELOCITY_H
