#ifndef ABSTRACT_VELOCITY_H
#define ABSTRACT_VELOCITY_H

#include <iostream>
#include <exception>

#include "common/common.h"
#include "common/class_forward.h"
#include "base/math/math_components.h"

#include "base/misc/coordinate_frame_components.h"

namespace mace{
namespace pose{

class Velocity : public Kinematic_BaseInterface
{
public:
    Velocity(const std::string &velName = "Velocity Object");

    Velocity(const Velocity &copy);

    virtual ~Velocity() override = default;

    /** Interface imposed via Kinemnatic_BaseInterace */
    KinematicTypes getKinematicType() const override
    {
        return KinematicTypes::VELOCITY;
    }

    /** End of interface imposed via Kinemnatic_BaseInterace */

public:

    void updateVelocityName(const std::string &nameString);

    std::string getName() const;

    virtual CoordinateFrameTypes getExplicitCoordinateFrame() const = 0;

public:

    //!
    //! \brief is3D
    //! \return
    //!
    virtual bool isGreaterThan1D() const
    {
        return dimension > 1;
    }

    //!
    //! \brief is3D
    //! \return
    //!
    virtual bool is2D() const
    {
        return dimension == 2;
    }

    //!
    //! \brief is3D
    //! \return
    //!
    virtual bool isGreaterThan2D() const
    {
        return dimension > 2;
    }

    //!
    //! \brief is3D
    //! \return
    //!
    virtual bool is3D() const
    {
        return dimension == 3;
    }

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
    virtual Velocity* getPositionalClone() const = 0;

    /**
     * @brief getClone
     * @param state
     */
    virtual void getPositionalClone(Velocity** vel) const = 0;


public:
    Velocity& operator = (const Velocity &rhs)
    {
        this->name = rhs.name;
        this->dimension = rhs.dimension;
        return *this;
    }

    bool operator == (const Velocity &rhs) const
    {
        if(this->name != rhs.name){
            return false;
        }
        if(this->dimension != rhs.dimension){
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

    uint8_t dimension = 0;
};



} // end of namespace pose
} // end of namespace mace

#endif // ABSTRACT_VELOCITY_H
