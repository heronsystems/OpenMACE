#ifndef BASE_VELOCITY_H
#define BASE_VELOCITY_H

#include <Eigen/Core>

#include "abstract_velocity.h"

namespace mace{
namespace pose{

template<typename CFDATA, class DATA>
class Base_Velocity: public Velocity
{
public:
    Base_Velocity(const CFDATA &frame):
        Velocity(), explicitFrame(frame)
    {

    }

    Base_Velocity(const Base_Velocity &copy):
        Velocity(copy)
    {
        explicitFrame = copy.explicitFrame;
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
