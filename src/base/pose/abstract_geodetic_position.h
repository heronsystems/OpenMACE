#ifndef ABSTRACT_GEODETIC_POSITION_H
#define ABSTRACT_GEODETIC_POSITION_H

#include <sstream>

#include <Eigen/Core>

#include "base_position.h"
#include "position_interface.h"

namespace mace {
namespace pose {

MACE_CLASS_FORWARD(Abstract_GeodeticPosition);
MACE_CLASS_FORWARD(GeodeticPosition_2D);
MACE_CLASS_FORWARD(GeodeticPosition_2D);

class Abstract_GeodeticPosition : public Position, public PositionInterface<Abstract_GeodeticPosition, misc::Data2D>
{
public:
    Abstract_GeodeticPosition(const GeodeticFrameTypes &explicitFrame, const std::string &posName = "Position Object");

    Abstract_GeodeticPosition(const GeodeticFrameTypes &explicitFrame, const double &latitude, const double &longitude, const std::string &posName = "Position Object");

    Abstract_GeodeticPosition(const Abstract_GeodeticPosition &copy);

    ~Abstract_GeodeticPosition() override = default;

public:
    PositionType getPositionType() const override;

    CoordinateFrameTypes getExplicitCoordinateFrame() const override;

    /** Assignment Operators */
public:
    void operator = (const Abstract_GeodeticPosition &rhs)
    {
        Position::operator =(rhs);
        this->geodeticFrameType = rhs.geodeticFrameType;
    }

    /** Relational Operators */
public:
    //!
    //! \brief operator ==
    //! \param rhs
    //! \return
    //!
    bool operator == (const Abstract_GeodeticPosition &rhs) const
    {
        if(!Position::operator ==(rhs))
            return false;

        if(this->geodeticFrameType != rhs.geodeticFrameType){
            return false;
        }
        return true;
    }

    //!
    //! \brief operator !=
    //! \param rhs
    //! \return
    //!
    bool operator != (const Abstract_GeodeticPosition &rhs) const {
        return !(*this == rhs);
    }

protected:
    GeodeticFrameTypes geodeticFrameType = CartesianFrameTypes::CF_LOCAL_UNKNOWN;
};

} //end of namespace pose
} //end of namespace mace

#endif // ABSTRACT_CARTESIAN_POSITION_H
