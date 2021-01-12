#ifndef POSE_EXPANDED_STATE_H
#define POSE_EXPANDED_STATE_H

#include <string>

#include "base/pose/cartesian_position_3D.h"
#include "base/pose/geodetic_position_3D.h"
#include "base/pose/rotation_3D.h"

namespace mace {
namespace pose {

template <typename POS, typename ROT>
class Pose_BasicState
{
public:
    Pose_BasicState() = default;

    Pose_BasicState(const Pose_BasicState &copy)
    {
        _position = copy._position;
        _rotation = copy._rotation;
        _speed = copy._speed;
    }

    Pose_BasicState(const POS &position, const ROT &rotation, const double &speed)
    {
        _position = position;
        _rotation = rotation;
        _speed = speed;
    }
public:
    void setPosition(const POS &position)
    {
        _position = position;
    }
    void setRotation(const ROT &rotation)
    {
        _rotation = rotation;
    }
    void setSpeed(const double &speed)
    {
        _speed = speed;
    }

public:
    POS getPosition() const
    {
        return _position;
    }

    ROT getRotation() const
    {
        return _rotation;
    }

    double getSpeed() const
    {
        return _speed;
    }

    void collectState(POS &position, ROT &rotation, double &speed)
    {
        position = _position;
        rotation = _rotation;
        speed = _speed;
    }

public:
    void operator = (const Pose_BasicState &rhs)
    {
        _position = rhs._position;
        _rotation = rhs._rotation;
        _speed = rhs._speed;
    }

    bool operator == (const Pose_BasicState &rhs) const{
        if(this->_position != rhs._position){
            return false;
        }
        if(this->_rotation != rhs._rotation){
            return false;
        }
        if(fabs(this->_speed - rhs._speed) > std::numeric_limits<double>::epsilon()){
            return false;
        }
        return true;
    }

    bool operator != (const Pose_BasicState &rhs) const{
        return !(*this == rhs);
    }

public:

    friend std::ostream &operator<<(std::ostream &out, const Pose_BasicState &obj)
    {
        UNUSED(obj);
        return out;
    }

private:
    POS _position;
    ROT _rotation;
    double _speed;
};

typedef Pose_BasicState<GeodeticPosition_3D, Rotation_3D> BasicGeoState3D;
typedef Pose_BasicState<CartesianPosition_3D, Rotation_3D> BasicCarState3D;

} //end of namespace pose
} //end of namespace mace


#endif // POSE_EXPANDED_STATE_H
