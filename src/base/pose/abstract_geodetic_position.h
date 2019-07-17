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
MACE_CLASS_FORWARD(GeodeticPosition_3D);

class Abstract_GeodeticPosition : public Position, public PositionInterface<Abstract_GeodeticPosition, misc::Data2D>
{
public:
    Abstract_GeodeticPosition(const GeodeticFrameTypes &explicitFrame, const std::string &posName = "Position Object");

    Abstract_GeodeticPosition(const GeodeticFrameTypes &explicitFrame, const double &latitude, const double &longitude, const std::string &posName = "Position Object");

    Abstract_GeodeticPosition(const Abstract_GeodeticPosition &copy);

    ~Abstract_GeodeticPosition() override = default;

    bool areEquivalentGeodeticFrames(const Abstract_GeodeticPosition &obj) const;

public:
    //!
    //! \brief setLatitude
    //! \param latitude
    //!
    void setLatitude(const double &latitude)
    {
        this->setY(latitude);
    }

    //!
    //! \brief setLongitude
    //! \param longitude
    //!
    void setLongitude(const double &longitude)
    {
        this->setX(longitude);
    }

    //!
    //! \brief getLatitude
    //! \return
    //!
    double getLatitude() const
    {
        return this->getY();
    }

    //!
    //! \brief getLongitude
    //! \return
    //!
    double getLongitude() const
    {
        return this->getX();
    }

    //!
    //! \brief hasLatitudeBeenSet
    //! \return
    //!
    bool hasLatitudeBeenSet() const
    {
        return this->getDataYFlag();
    }

    //!
    //! \brief hasLongitudeBeenSet
    //! \return
    //!
    bool hasLongitudeBeenSet() const
    {
        return this->getDataXFlag();
    }

    //!
    //! \brief deltaLatitude
    //! \param that
    //! \return
    //!
    double deltaLatitude(const Abstract_GeodeticPosition* that) const
    {
        return this->getLatitude() - that->getLatitude();
    }

    //!
    //! \brief deltaLongitude
    //! \param that
    //! \return
    //!
    double deltaLongitude(const Abstract_GeodeticPosition* that) const
    {
        return this->getLongitude() - that->getLongitude();
    }

public:
    PositionType getPositionType() const override;

    void setCoordinateFrame(const GeodeticFrameTypes &explicitFrame);

    GeodeticFrameTypes getGeodeticCoordinateFrame() const;

    CoordinateFrameTypes getExplicitCoordinateFrame() const override;

    /** Assignment Operators */
public:
    Abstract_GeodeticPosition& operator = (const Abstract_GeodeticPosition &rhs)
    {
        Position::operator =(rhs);
        this->geodeticFrameType = rhs.geodeticFrameType;
        return *this;
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
    GeodeticFrameTypes geodeticFrameType = GeodeticFrameTypes::CF_GLOBAL_UNKNOWN;
};

} //end of namespace pose
} //end of namespace mace

#endif // ABSTRACT_CARTESIAN_POSITION_H
