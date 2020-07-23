#ifndef ABSTRACT_CARTESIAN_POSITION_H
#define ABSTRACT_CARTESIAN_POSITION_H

#include <sstream>

#include <Eigen/Core>

#include "abstract_position.h"
#include "position_interface.h"

namespace mace {
namespace pose {

MACE_CLASS_FORWARD(Abstract_CartesianPosition);
MACE_CLASS_FORWARD(CartesianPosition_2D);
MACE_CLASS_FORWARD(CartesianPosition_3D);

class Abstract_CartesianPosition : public Position, public PositionInterface<Abstract_CartesianPosition>
{
public:
    Abstract_CartesianPosition(const CartesianFrameTypes &explicitFrame, const std::string &posName = "Position Object");

    Abstract_CartesianPosition(const Abstract_CartesianPosition &copy);

    ~Abstract_CartesianPosition() override = default;

    virtual Abstract_CartesianPosition* getCartesianClone() const = 0;

    virtual void getCartesianClone(Abstract_CartesianPosition** state) const = 0;

    /** Interface imposed via Position*/
    PositionTypes getPositionalType() const
    {
        return PositionTypes::TRANSLATIONAL;
    }

    /** End of interface imposed via Position */

public:
    CoordinateSystemTypes getCoordinateSystemType() const override;

    void setCoordinateFrame(const CartesianFrameTypes &explicitFrame);

    CartesianFrameTypes getCartesianCoordinateFrame() const;

    CoordinateFrameTypes getExplicitCoordinateFrame() const override;

    bool areEquivalentCartesianFrames(const Abstract_CartesianPosition &obj) const;

    /** Assignment Operators */
public:
    Abstract_CartesianPosition& operator = (const Abstract_CartesianPosition &rhs)
    {
        Position::operator =(rhs);
        this->cartesianFrameType = rhs.cartesianFrameType;
        return *this;
    }

    /** Relational Operators */
public:
    //!
    //! \brief operator ==
    //! \param rhs
    //! \return
    //!
    bool operator == (const Abstract_CartesianPosition &rhs) const
    {
        if(!Position::operator ==(rhs))
            return false;

        if(this->cartesianFrameType != rhs.cartesianFrameType){
            return false;
        }
        return true;
    }

    //!
    //! \brief operator !=
    //! \param rhs
    //! \return
    //!
    bool operator != (const Abstract_CartesianPosition &rhs) const {
        return !(*this == rhs);
    }

protected:
    CartesianFrameTypes cartesianFrameType = CartesianFrameTypes::CF_LOCAL_UNKNOWN;
};

} //end of namespace pose
} //end of namespace mace

#endif // ABSTRACT_CARTESIAN_POSITION_H
