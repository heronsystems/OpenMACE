#ifndef VELOCITY_INTERFACE_ROTATIONAL_H
#define VELOCITY_INTERFACE_ROTATIONAL_H

#include <Eigen/Core>
#include <type_traits>

#include "abstract_velocity.h"

namespace mace{
namespace pose{

template<class DATA>
class VelocityInterface_Rotational: public Velocity
{
public:
    VelocityInterface_Rotational():
        Velocity()
    {
        this->dimension = 3;
    }

    VelocityInterface_Rotational(const VelocityInterface_Rotational &copy):
        Velocity(copy)
    {
        data = copy.data;
    }

    Eigen::VectorXd getDataVector() const override
    {
        return data;
    }

    void updateDataVector(const Eigen::VectorXd &vecObj) override
    {
        data = vecObj.head(dimension);
    }


    /** Interface imposed via Velocity */
    VelocityTypes getVelocityType() const override
    {
        return VelocityTypes::ROTATIONAL;
    }

    CoordinateSystemTypes getCoordinateSystemType() const override
    {
        return CoordinateSystemTypes::NOT_IMPLIED;
    }

    CoordinateFrameTypes getExplicitCoordinateFrame() const override
    {
        return CoordinateFrameTypes::CF_NOT_RELEVANT;
    }

    /** End of interface imposed via Velocity */


public:
    Velocity* getVelocityClone() const override
    {
        return (new VelocityInterface_Rotational<DATA>(*this));
    }

    void getVelocityClone(Velocity** state) const override
    {
        *state = new VelocityInterface_Rotational<DATA>(*this);
    }

public:
    //!
    //! \brief operator =
    //! \param rhs
    //! \return
    //!
    VelocityInterface_Rotational& operator = (const VelocityInterface_Rotational &rhs)
    {
        Velocity::operator=(rhs);
        this->data = rhs.data;
        return *this;
    }

public:
    bool operator == (const VelocityInterface_Rotational &rhs) const
    {
        if(!Velocity::operator ==(rhs))
            return false;
        if(data != rhs.data){
            return false;
        }
        return true;
    }

    bool operator !=(const Velocity &rhs) const
    {
        return !(*this == rhs);
    }

public:
    DATA data;
};

typedef VelocityInterface_Rotational<Eigen::Vector3d> Velocity_Rotation3D;

} // end of namespace pose
} // end of namespace mace

#endif // VELOCITY_INTERFACE_ROTATIONAL_H
