#ifndef DYNAMIC_TARGET_H
#define DYNAMIC_TARGET_H

#include <iostream>
#include <stdint.h>
#include <memory>
#include <list>
#include <map>

#include "base/pose/geodetic_position_3D.h"
#include "base/pose/cartesian_position_3D.h"
#include "base/pose/cartesian_velocity_3D.h"

using namespace mace::pose;

namespace TargetItem {

template <class POS, class VEL>
class DynamicTarget{
public:
    DynamicTarget() = default;

    ~DynamicTarget() = default;

    CoordinateFrame getPositionalCoordinateFrame() const
    {
        return this->position.getCoordinateFrame();
    }

    void setPosition(const POS &pos)
    {
        this->position = pos;
    }
    void setVelocity(const VEL &vel)
    {
        this->velocity = vel;
    }
    void setYaw(const double &yaw, const double &yawRate)
    {
        this->yaw = yaw;
        this->yawRate = yawRate;
    }

    POS getPosition() const
    {
        return this->position;
    }

    VEL getVelocity() const
    {
        return this->velocity;
    }

    double getYaw() const
    {
        return this->yaw;
    }

    double getYawRate() const
    {
        return this->yawRate;
    }
public:
    DynamicTarget& operator = (const DynamicTarget &rhs)
    {
        this->position = rhs.position;
        this->velocity = rhs.velocity;
        this->yaw = rhs.yaw;
        this->yawRate = rhs.yawRate;
        return *this;
    }

    bool operator == (const DynamicTarget &rhs) const{
        if(this->position != rhs.position){
            return false;
        }
        if(this->velocity != rhs.velocity){
            return false;
        }
        if(this->yaw != rhs.yaw){
            return false;
        }
        if(this->yawRate != rhs.yawRate){
            return false;
        }
        return true;
    }

    bool operator != (const DynamicTarget &rhs) const{
        return !(*this == rhs);
    }

private:
    POS position;
    VEL velocity;
    double yaw = 0.0;
    double yawRate = 0.0;
};

typedef DynamicTarget<CartesianPosition_3D,CartesianVelocity_3D> CartesianDynamicTarget;
typedef DynamicTarget<GeodeticPosition_3D,CartesianVelocity_3D> GeodeticDynamicTarget;

} //end of namespace TargetItem

#endif // DYNAMIC_TARGET_H
