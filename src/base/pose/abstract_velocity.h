#ifndef ABSTRACT_VELOCITY_H
#define ABSTRACT_VELOCITY_H

#include <Eigen/Core>

#include <iostream>
#include <exception>

#include "common/common.h"
#include "common/class_forward.h"
#include "../math/math_components.h"

#include "../misc/coordinate_frame_components.h"

namespace mace{
namespace pose{

MACE_CLASS_FORWARD(Velocity);

class Velocity : public Kinematic_BaseInterface
{
public:
    enum TYPEMASK_VELOCITY : uint16_t
    {
        VELOCITY_VALID = 0,
        IGNORE_X_DIMENSION = 8,
        IGNORE_Y_DIMENSION = 16,
        IGNORE_Z_DIMENSION = 32
    };

public:
    static const uint16_t ignoreAllVelocities = IGNORE_X_DIMENSION|IGNORE_Y_DIMENSION|IGNORE_Z_DIMENSION;

public:
    Velocity(const std::string &velName = "Velocity Object");

    Velocity(const Velocity &copy);

    virtual ~Velocity() override = default;

    /** Interface imposed via Kinemnatic_BaseInterace */
    virtual KinematicTypes getKinematicType() const override
    {
        return KinematicTypes::VELOCITY;
    }

    /** End of interface imposed via Kinemnatic_BaseInterace */

    virtual VelocityTypes getVelocityType() const = 0;

public:
    virtual Eigen::VectorXd getDataVector() const = 0;

    virtual void updateDataVector(const Eigen::VectorXd &vecObj) = 0;

    bool areAllVelocitiesValid() const
    {
        return (dimension&ignoreAllVelocities) > 0 ? false : true;
    }

    bool isAnyVelocityValid() const
    {
        return (dimensionMask^ignoreAllVelocities) > 0 ? true : false;
    }
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
    virtual Velocity* getVelocityClone() const = 0;

    /**
     * @brief getClone
     * @param state
     */
    virtual void getVelocityClone(Velocity** vel) const = 0;


public:
    Velocity& operator = (const Velocity &rhs)
    {
        this->name = rhs.name;
        this->dimension = rhs.dimension;
        this->dimensionMask = rhs.dimensionMask;
        return *this;
    }

    bool operator == (const Velocity &rhs) const
    {
        if(this->dimension != rhs.dimension){
            return false;
        }
        if(this->name != rhs.name){
            return false;
        }
        if(this->dimensionMask != rhs.dimensionMask){
            return false;
        }
        return true;
    }

    bool operator !=(const Velocity &rhs) const
    {
        return !(*this == rhs);
    }

protected:
    std::string name;
};



} // end of namespace pose
} // end of namespace mace

#endif // ABSTRACT_VELOCITY_H
