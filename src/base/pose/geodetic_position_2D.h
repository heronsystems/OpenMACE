#ifndef GEODETIC_POSITION_2D_H
#define GEODETIC_POSITION_2D_H

#include "base/state_space/state.h"
#include "abstract_geodetic_position.h"

namespace mace {
namespace pose {

class GeodeticPosition_2D : public Abstract_GeodeticPosition,
        public state_space::State
{

public:
    //!
    //! \brief GeodeticPosition_2D
    //! \param pointName
    //! \param latitude
    //! \param longitude
    //!
    GeodeticPosition_2D(const GeodeticFrameTypes &frameType = GeodeticFrameTypes::CF_GLOBAL_RELATIVE_ALT,
                        const double &latitude = 0.0, const double &longitude = 0.0,
                        const std::string &pointName = "Position Point");

    GeodeticPosition_2D(const std::string &pointName,
                        const double &latitude, const double &longitude);

    GeodeticPosition_2D(const double &latitude, const double &longitude);

    //!
    //! \brief GeodeticPosition_2D
    //! \param copy
    //!
    GeodeticPosition_2D(const GeodeticPosition_2D &copy);

    GeodeticPosition_2D(const GeodeticPosition_3D &copy);

    ~GeodeticPosition_2D() override = default;

    bool areEquivalentFrames(const GeodeticPosition_2D &obj) const;


    //!
    //! \brief printInfo
    //! \return
    //!
    std::string printInfo() const override
    {
        std::string rtn = "Geodetic Position 2D: " + std::to_string(getLatitude()) + ", " + std::to_string(getLongitude()) + ".";
        return rtn;
    }

public:
    //!
    //! \brief updatePosition
    //! \param latitude
    //! \param longitude
    //!
    void updatePosition(const double &latitude, const double &longitude)
    {
        this->setData_2D(latitude,longitude);
    }

    //!
    //! \brief getAsVector
    //! \return
    //!
    Eigen::Vector2d getAsVector()
    {
        Eigen::Vector2d vec(this->getX(), this->getY());
        return vec;
    }


    /** Interface imposed via state_space::State */
public:
    //!
    //! \brief getStateClone
    //! \return
    //!
    State* getStateClone() const override
    {
        return (new GeodeticPosition_2D(*this));
    }

    //!
    //! \brief getStateClone
    //! \param state
    //!
    void getStateClone(State** state) const override
    {
        *state = new GeodeticPosition_2D(*this);
    }


    /** Interface imposed via Abstract_CartesianPosition */

public:
    //!
    //! \brief getPositionalClone
    //! \return
    //!
    Position* getPositionalClone() const override
    {
        return (new GeodeticPosition_2D(*this));
    }

    //!
    //! \brief getPositionalClone
    //! \param state
    //!
    void getPositionalClone(Position** state) const override
    {
        *state = new GeodeticPosition_2D(*this);
    }

public:
    //!
    //! \brief hasBeenSet
    //! \return
    //!
    bool hasBeenSet() const override
    {
        return hasLatitudeBeenSet() || hasLongitudeBeenSet();
    }

    //!
    //! \brief distanceFromOrigin
    //! \return
    //!
    double distanceFromOrigin() const override;

    //!
    //! \brief polarBearingFromOrigin
    //! \return
    //!
    double polarBearingFromOrigin() const override;

    //!
    //! \brief distanceBetween2D
    //! \param position
    //! \return
    //!
    double distanceBetween2D(const Abstract_GeodeticPosition* pos) const override;

    //!
    //! \brief distanceTo
    //! \param position
    //! \return
    //!
    double distanceTo(const Abstract_GeodeticPosition* pos) const override;

    //!
    //! \brief polarBearingTo
    //! \param position
    //! \return
    //!
    double polarBearingTo(const Abstract_GeodeticPosition* pos) const override;

    //!
    //! \brief polarBearingTo
    //! \param position
    //! \return
    //!
    double compassBearingTo(const Abstract_GeodeticPosition* pos) const override;

    //!
    //! \brief newPositionFromPolar
    //! \param distance
    //! \param compassBearing
    //! \return
    //!
    virtual void newPositionFromPolar(Abstract_GeodeticPosition* newObject, const double &distance, const double &bearing) const override;

    //!
    //! \brief newPositionFromPolar
    //! \param distance
    //! \param compassBearing
    //! \return
    //!
    virtual void newPositionFromCompass(Abstract_GeodeticPosition *newObject, const double &distance, const double &bearing) const override;


    //!
    //! \brief applyPositionalShiftFromPolar
    //! \param distance
    //! \param bearing
    //!
    void applyPositionalShiftFromPolar(const double &distance, const double &bearing) override;

    //!
    //! \brief applyPositionalShiftFromCompass
    //! \param distance
    //! \param bearing
    //!
    void applyPositionalShiftFromCompass(const double &distance, const double &bearing) override;

    /** Assignment Operators */
public:
    GeodeticPosition_2D& operator = (const GeodeticPosition_2D &rhs)
    {
        Abstract_GeodeticPosition::operator =(rhs);
        return *this;
    }

    /** Arithmetic Operators */
public:

    //!
    //! \brief operator +
    //! \param that
    //! \return
    //!
    GeodeticPosition_2D operator + (const GeodeticPosition_2D &rhs) const
    {
        GeodeticPosition_2D newPoint(*this);

        if(this->areEquivalentFrames(rhs))
        {
            newPoint.x = newPoint.x + rhs.x;
            newPoint.y = newPoint.y + rhs.y;
        }
        else
        {
            throw std::logic_error("Tried to perform a + operation between 3DGeodetic of differnet coordinate frames.");
        }

        return newPoint;
    }

    //!
    //! \brief operator -
    //! \param that
    //! \return
    //!
    GeodeticPosition_2D operator - (const GeodeticPosition_2D &rhs) const
    {
        GeodeticPosition_2D newPoint(*this);

        if(this->areEquivalentFrames(rhs))
        {
            newPoint.x = newPoint.x - rhs.x;
            newPoint.y = newPoint.y - rhs.y;
        }
        else
        {
            throw std::logic_error("Tried to perform a - operation between 3DGeodetic of differnet coordinate frames.");
        }

        return newPoint;
    }
    /** Relational Operators */
public:
    //!
    //! \brief operator ==
    //! \param rhs
    //! \return
    //!
    bool operator == (const GeodeticPosition_2D &rhs) const
    {
        if(!Abstract_GeodeticPosition::operator ==(rhs))
            return false;
        return true;
    }

    //!
    //! \brief operator !=
    //! \param rhs
    //! \return
    //!
    bool operator != (const GeodeticPosition_2D &rhs) const {
        return !(*this == rhs);
    }
};

} //end of namespace pose
} //end of namespace mace

#endif // GEODETIC_POSITION_2D_H
