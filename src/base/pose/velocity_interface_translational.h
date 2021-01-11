#ifndef VELOCITY_INTERFACE_TRANSLATIONAL_H
#define VELOCITY_INTERFACE_TRANSLATIONAL_H

#include <Eigen/Core>
#include <type_traits>

#include "abstract_velocity.h"

namespace mace{
namespace pose{

template<const CoordinateSystemTypes coordType, typename CFDATA, class DATA>
class VelocityInterface_Translational: public Velocity
{
public:
    VelocityInterface_Translational():
        Velocity(), explicitFrame()
    {

    }

    VelocityInterface_Translational(const CFDATA &frame):
        Velocity(), explicitFrame(frame)
    {
        explicitType = coordType;
    }

    VelocityInterface_Translational(const VelocityInterface_Translational &copy):
        Velocity(copy)
    {
        explicitType = copy.explicitType;
        explicitFrame = copy.explicitFrame;
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
        return VelocityTypes::TRANSLATIONAL;
    }

    /** End of interface imposed via Velocity */


public:
    Velocity* getVelocityClone() const override
    {
        return (new VelocityInterface_Translational<coordType, CFDATA, DATA>(*this));
    }

    void getVelocityClone(Velocity** state) const override
    {
        *state = new VelocityInterface_Translational<coordType, CFDATA, DATA>(*this);
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

public:

    VelocityInterface_Translational& operator = (const VelocityInterface_Translational &rhs)
    {
        Velocity::operator=(rhs);
        this->explicitType = rhs.explicitType;
        this->explicitFrame = rhs.explicitFrame;
        this->data = rhs.data;
        return *this;
    }

    bool operator == (const VelocityInterface_Translational &rhs) const
    {
        if(!Velocity::operator ==(rhs))
            return false;

        if(explicitType != rhs.explicitType){
            return false;
        }
        if(explicitFrame != rhs.explicitFrame){
            return false;
        }
        if(data != rhs.data){
            return false;
        }
        return true;
    }

    bool operator !=(const Velocity &rhs) const
    {
        return !(*this == rhs);
    }

private:
    CoordinateSystemTypes explicitType = CoordinateSystemTypes::UNKNOWN;
    CFDATA explicitFrame;

public:
    DATA data;
};


} // end of namespace pose
} // end of namespace mace

#endif // VELOCITY_INTERFACE_TRANSLATIONAL_H
