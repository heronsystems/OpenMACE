#ifndef GEODETIC_POSITION_2D_H
#define GEODETIC_POSITION_2D_H

#include "base_position.h"
#include "base/state_space/state.h"

using namespace mace::math;

namespace mace {
namespace pose {

class GeodeticPosition_2D : public AbstractPosition<GeodeticPosition_2D, misc::Data2D>, public GeodeticPosition,
        public state_space::State
{
public:
    GeodeticPosition_2D():
        AbstractPosition(AbstractPosition::PositionType::GEODETIC, CoordinateFrame::CF_GLOBAL_RELATIVE_ALT)
    {

    }

    GeodeticPosition_2D(const GeodeticPosition_2D &copy):
        AbstractPosition(copy), state_space::State(copy)
    {

    }

    GeodeticPosition_2D(const double latitude, const double &longitude):
        AbstractPosition(AbstractPosition::PositionType::CARTESIAN, CoordinateFrame::CF_GLOBAL_RELATIVE_ALT)
    {
        this->data.setData(latitude,longitude);
    }

    State* getClone() const override
    {
        return (new GeodeticPosition_2D(*this));
    }

    void getClone(State** state) const override
    {
        *state = new GeodeticPosition_2D(*this);
    }

public:
    void updatePosition(const double &latitude, const double &longitude)
    {
        this->data.setData(latitude,longitude);
    }

    void setLatitude(const double &latitude)
    {
        this->data.setX(latitude);
    }

    void setLongitude(const double &longitude)
    {
        this->data.setY(longitude);
    }

    double getLatitude() const
    {
        return this->data.getX();
    }

    double getLongitude() const
    {
        return this->data.getY();
    }

    Eigen::Vector2d getAsVector()
    {
        Eigen::Vector2d vec(this->data.getX(), this->data.getY());
        return vec;
    }

    bool hasLatitudeBeenSet() const
    {
        return this->data.getDataXFlag();
    }

    bool hasLongitudeBeenSet() const
    {
        return this->data.getDataYFlag();
    }
public:
    double deltaLatitude(const GeodeticPosition_2D &that) const;
    double deltaLongitude(const GeodeticPosition_2D &that) const;
public:
    void setCoordinateFrame(const GlobalFrameType &desiredFrame)
    {
        this->frame = mace::pose::getCoordinateFrame(desiredFrame);
    }

    /** Arithmetic Operators */
public:

    //!
    //! \brief operator +
    //! \param that
    //! \return
    //!
    GeodeticPosition_2D operator + (const GeodeticPosition_2D &that) const
    {
        GeodeticPosition_2D newPoint(*this);
        newPoint.data = this->data + that.data;
        return newPoint;
    }

    //!
    //! \brief operator -
    //! \param that
    //! \return
    //!
    GeodeticPosition_2D operator - (const GeodeticPosition_2D &that) const
    {
        GeodeticPosition_2D newPoint(*this);
        newPoint.data = this->data - that.data;
        return newPoint;
    }


public:

    bool hasBeenSet() const override
    {
        return hasLatitudeBeenSet() || hasLongitudeBeenSet();
    }

    double distanceFromOrigin() const override;

    double polarBearingFromOrigin() const override;

    //!
    //! \brief distanceBetween2D
    //! \param position
    //! \return
    //!
    double distanceBetween2D(const GeodeticPosition_2D &position) const override;

    //!
    //! \brief distanceTo
    //! \param position
    //! \return
    //!
    double distanceTo(const GeodeticPosition_2D &pos) const override;

    //!
    //! \brief polarBearingTo
    //! \param position
    //! \return
    //!
    double polarBearingTo(const GeodeticPosition_2D &pos) const override;

    //!
    //! \brief polarBearingTo
    //! \param position
    //! \return
    //!
    double compassBearingTo(const GeodeticPosition_2D &pos) const override;

    //!
    //! \brief newPositionFromPolar
    //! \param distance
    //! \param compassBearing
    //! \return
    //!
    GeodeticPosition_2D newPositionFromPolar(const double &distance, const double &bearing) const override;

    //!
    //! \brief newPositionFromPolar
    //! \param distance
    //! \param compassBearing
    //! \return
    //!
    GeodeticPosition_2D newPositionFromCompass(const double &distance, const double &bearing) const override;

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
};

} //end of namespace pose
} //end of namespace mace

#endif // GEODETIC_POSITION_2D_H
