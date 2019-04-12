#ifndef GEODETIC_POSITION_3D_H
#define GEODETIC_POSITION_3D_H

#include "base_position.h"
#include "base/state_space/state.h"

using namespace mace::math;

namespace mace {
namespace pose {

class GeodeticPosition_3D : public AbstractPosition<GeodeticPosition_3D, misc::Data3D>, public GeodeticPosition,
        public state_space::State
{
public:
    GeodeticPosition_3D():
        AbstractPosition(AbstractPosition::PositionType::GEODETIC, CoordinateFrame::CF_GLOBAL_RELATIVE_ALT)
    {

    }

    GeodeticPosition_3D(const GeodeticPosition_3D &copy):
        AbstractPosition(copy), state_space::State(copy)
    {

    }

    GeodeticPosition_3D(const double latitude, const double &longitude, const double &altitude):
        AbstractPosition(AbstractPosition::PositionType::CARTESIAN, CoordinateFrame::CF_GLOBAL_RELATIVE_ALT)
    {
        this->data.setData(latitude,longitude,altitude);
    }

    State* getClone() const override
    {
        return (new GeodeticPosition_3D(*this));
    }

    void getClone(State** state) const override
    {
        *state = new GeodeticPosition_3D(*this);
    }

public:
    void updatePosition(const double &latitude, const double &longitude)
    {
        this->data.setData(latitude,longitude,this->getAltitude());
    }

    void updatePosition(const double &latitude, const double &longitude, const double &altitude)
    {
        this->data.setData(latitude,longitude,altitude);
    }

    void setLatitude(const double &latitude)
    {
        this->data.setX(latitude);
    }

    void setLongitude(const double &longitude)
    {
        this->data.setY(longitude);
    }

    void setAltitude(const double &altitude)
    {
        this->data.setZ(altitude);
    }

    double getLatitude() const
    {
        return this->data.getX();
    }

    double getLongitude() const
    {
        return this->data.getY();
    }

    double getAltitude() const
    {
        return this->data.getZ();
    }

    Eigen::Vector3d getAsVector()
    {
        Eigen::Vector3d vec(this->data.getX(), this->data.getY(), this->data.getZ());
        return vec;
    }

    bool hasLatitudeBeenSet() const
    {
        return this->data.getDataXFlag();
    }

    bool hasLongitudeBeenSet() const
    {
        return this->data.getDataXFlag();
    }

    bool hasAltitudeBeenSet() const
    {
        return this->data.getDataZFlag();
    }

public:
    double deltaLatitude(const GeodeticPosition_3D &that) const;
    double deltaLongitude(const GeodeticPosition_3D &that) const;
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
    GeodeticPosition_3D operator + (const GeodeticPosition_3D &that) const
    {
        GeodeticPosition_3D newPoint(*this);
        newPoint.data = this->data + that.data;
        return newPoint;
    }

    //!
    //! \brief operator -
    //! \param that
    //! \return
    //!
    GeodeticPosition_3D operator - (const GeodeticPosition_3D &that) const
    {
        GeodeticPosition_3D newPoint(*this);
        newPoint.data = this->data - that.data;
        return newPoint;
    }


public:

    bool hasBeenSet() const override
    {
        return hasLatitudeBeenSet() || hasLongitudeBeenSet() || hasAltitudeBeenSet();
    }
    //!
    //! \brief deltaAltitude
    //! \param position
    //! \return
    //!
    double deltaAltitude(const GeodeticPosition_3D &position) const;

    //!
    //! \brief distanceBetween2D
    //! \param position
    //! \return
    //!
    double distanceBetween2D(const GeodeticPosition_3D &pos) const override;

    //!
    //! \brief distanceBetween3D
    //! \param position
    //! \return
    //!
    double distanceBetween3D(const GeodeticPosition_3D &position) const;

    //!
    //! \brief distanceTo
    //! \param position
    //! \return
    //!
    double distanceTo(const GeodeticPosition_3D &pos) const override;

    double distanceFromOrigin() const override;

    double polarBearingFromOrigin() const override;

    //!
    //! \brief polarBearingTo
    //! \param position
    //! \return
    //!
    double polarBearingTo(const GeodeticPosition_3D &pos) const override;

    //!
    //! \brief polarBearingTo
    //! \param position
    //! \return
    //!
    double compassBearingTo(const GeodeticPosition_3D &pos) const override;

    //!
    //! \brief polarAzimuthTo
    //! \param position
    //! \return
    //!
    double elevationAngleTo(const GeodeticPosition_3D &position) const;

    //!
    //! \brief newPositionFromPolar
    //! \param distance
    //! \param compassBearing
    //! \return
    //!
    GeodeticPosition_3D newPositionFromPolar(const double &distance, const double &bearing) const override;

    GeodeticPosition_3D newPositionFromPolar(const double &distance, const double &bearing, const double &elevation) const;

    //!
    //! \brief newPositionFromPolar
    //! \param distance
    //! \param compassBearing
    //! \return
    //!
    GeodeticPosition_3D newPositionFromCompass(const double &distance, const double &bearing) const override;

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

#endif // GEODETIC_POSITION_3D_H
