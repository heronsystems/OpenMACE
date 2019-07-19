#ifndef ABSTRACT_VELOCITY_H
#define ABSTRACT_VELOCITY_H

#include <Eigen/Core>

#include <iostream>
#include <exception>

#include "common/common.h"
#include "common/class_forward.h"
#include "base/math/math_components.h"

#include "base/misc/coordinate_frame_components.h"

namespace mace{
namespace pose{

class Abstract_Velocity : public Kinematic_BaseInterface
{
public:
    Abstract_Velocity(const std::string &velName = "Velocity Object");

    Abstract_Velocity(const Abstract_Velocity &copy);

    virtual ~Abstract_Velocity() override = default;

    /** Interface imposed via Kinemnatic_BaseInterace */
    KinematicTypes getKinematicType() const override
    {
        return KinematicTypes::VELOCITY;
    }

    /** End of interface imposed via Kinemnatic_BaseInterace */

public:
    virtual Eigen::VectorXd getDataVector() const = 0;

    virtual void updateDataVector(const Eigen::VectorXd &vecObj) const = 0;

public:

    void updateVelocityName(const std::string &nameString);

    std::string getName() const;

    virtual CoordinateFrameTypes getExplicitCoordinateFrame() const = 0;

public:
    /**
     *
     */
    template <class T>
    const T *velocityAs() const
    {
        //ensure that we are attempting to cast it to a type of state
        return static_cast<const T *>(this);
    }

    /**
     *
     */
    template <class T>
    T *velocityAs()
    {
        //ensure that we are attempting to cast it to a type of state
        return static_cast<T *>(this);
    }

    /**
     * @brief getClone
     * @return
     */
    virtual Abstract_Velocity* getVelocityClone() const = 0;

    /**
     * @brief getClone
     * @param state
     */
    virtual void getVelocityClone(Abstract_Velocity** vel) const = 0;


public:
    Abstract_Velocity& operator = (const Abstract_Velocity &rhs)
    {
        this->name = rhs.name;
        this->dimension = rhs.dimension;
        return *this;
    }

    bool operator == (const Abstract_Velocity &rhs) const
    {
        if(this->name != rhs.name){
            return false;
        }
        if(this->dimension != rhs.dimension){
            return false;
        }
        return true;
    }

    bool operator !=(const Abstract_Velocity &rhs) const
    {
        return !(*this == rhs);
    }

protected:
    std::string name;
};



} // end of namespace pose
} // end of namespace mace

#endif // ABSTRACT_VELOCITY_H
