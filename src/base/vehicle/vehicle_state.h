#ifndef VEHICLE_STATE_H
#define VEHICLE_STATE_H

#include "base/pose/cartesian_position_3D.h"
#include "base/pose/geodetic_position_3D.h"
#include "base/pose/rotation_3D.h"
#include "base/pose/velocity_helper.h"
#include "base/pose/velocity_interface_rotational.h"

#include "data/environment_time.h"
#include "common/class_forward.h"


template <class POSOBJ, class VELOBJ>
class VehicleState
{

public:
    VehicleState()
    {

    }

    VehicleState(const VehicleState &copyObj)
    {
        this->m_UpdateTime = copyObj.m_UpdateTime;
        this->m_Position = copyObj.m_Position;
        this->m_Velocity = copyObj.m_Velocity;
        this->m_Rotation = copyObj.m_Rotation;
    }

    ~VehicleState()
    {

    }

public:
    VehicleState &operator=(const VehicleState &rhs)
    {
        this->m_UpdateTime = rhs.m_UpdateTime;
        this->m_Position = rhs.m_Position;
        this->m_Velocity = rhs.m_Velocity;
        this->m_Rotation = rhs.m_Rotation;
        return *this;
    }

    bool operator==(const VehicleState &rhs) const
    {
        if (this->m_UpdateTime != rhs.m_UpdateTime)
        {
            return false;
        }
        if (this->m_Position != rhs.m_Position)
        {
            return false;
        }
        // if (this->m_Velocity != rhs.m_Velocity) //Ken Fix This Statement
        // {
        //     return false;
        // }
        if (this->m_Rotation != rhs.m_Rotation)
        {
            return false;
        }
        return true;
    }

    bool operator!=(const VehicleState &rhs) const
    {
        return !(*this == rhs);
    }

public:
    Data::EnvironmentTime m_UpdateTime;

    POSOBJ m_Position;
    VELOBJ m_Velocity;
    mace::pose::Rotation_3D m_Rotation;
    mace::pose::Velocity_Rotation3D m_RotationalVelocity;
};


typedef VehicleState<mace::pose::CartesianPosition_3D, mace::pose::Velocity_Cartesian3D> VehicleState_Cartesian3D;
typedef std::shared_ptr<VehicleState_Cartesian3D> VehicleState_Cartesian3DPtr;

typedef VehicleState<mace::pose::GeodeticPosition_3D, mace::pose::Velocity_Cartesian3D> VehicleState_Geodetic3D;
typedef std::shared_ptr<VehicleState_Geodetic3D> VehicleState_Geodetic3DPtr;


#endif //VehicleState
