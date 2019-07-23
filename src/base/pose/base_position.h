#ifndef BASE_POSITION_H
#define BASE_POSITION_H

#include <Eigen/Core>

#include <iostream>
#include <exception>

#include "common/common.h"
#include "common/class_forward.h"
#include "base/math/math_components.h"

#include "../misc/coordinate_frame_components.h"

namespace mace{
namespace pose{

class Position : public Kinematic_BaseInterface
{
public:
    Position(const std::string &posName = "Position Object");

    Position(const Position &copy);

    virtual ~Position() override = default;

    /** Interface imposed via Kinemnatic_BaseInterace */
    KinematicTypes getKinematicType() const override
    {
        return KinematicTypes::POSITION;
    }

    /** End of interface imposed via Kinemnatic_BaseInterace */

public:
    virtual Eigen::VectorXd getDataVector() const = 0;

public:

    void setName(const std::string &nameString);

    std::string getName() const;

    virtual CoordinateFrameTypes getExplicitCoordinateFrame() const = 0;

public:
    /**
     *
     */
    template <class T>
    const T *positionAs() const
    {
        //ensure that we are attempting to cast it to a type of state
        return static_cast<const T *>(this);
    }

    /**
     *
     */
    template <class T>
    T *positionAs()
    {
        //ensure that we are attempting to cast it to a type of state
        return static_cast<T *>(this);
    }

    /**
     * @brief getClone
     * @return
     */
    virtual Position* getPositionalClone() const = 0;

    /**
     * @brief getClone
     * @param state
     */
    virtual void getPositionalClone(Position** position) const = 0;


public:
    Position& operator = (const Position &rhs)
    {
        this->name = rhs.name;
        this->dimension = rhs.dimension;
        return *this;
    }

    bool operator == (const Position &rhs) const
    {
        if(this->name != rhs.name){
            return false;
        }
        if(this->dimension != rhs.dimension){
            return false;
        }
        return true;
    }

    bool operator !=(const Position &rhs) const
    {
        return !(*this == rhs);
    }

protected:
    std::string name;
};



} // end of namespace pose
} // end of namespace mace

#endif // BASE_POSITION_H
