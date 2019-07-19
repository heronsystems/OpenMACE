#ifndef GEODETIC_POSITION_3D_H
#define GEODETIC_POSITION_3D_H

#include "base/state_space/state.h"
#include "abstract_geodetic_position.h"
#include "abstract_altitude.h"

namespace mace {
namespace pose {

class GeodeticPosition_3D : public Abstract_GeodeticPosition, public Abstract_Altitude, public state_space::State
{
public:

    GeodeticPosition_3D(const GeodeticFrameTypes &frameType = GeodeticFrameTypes::CF_GLOBAL_RELATIVE_ALT,
                        const double &latitude = 0.0, const double &longitude = 0.0,
                        const AltitudeReferenceTypes &altitudeType = AltitudeReferenceTypes::REF_ALT_UNKNOWN, const double &altitude = 0.0,
                        const std::string &pointName = "Geodetic Point");

    GeodeticPosition_3D(const double &latitude, const double &longitude, const double &altitude, const std::string &pointName = "Geodetic Point");

    //!
    //! \brief GeodeticPosition_2D
    //! \param copy
    //!
    GeodeticPosition_3D(const GeodeticPosition_3D &copy);

    GeodeticPosition_3D(const GeodeticPosition_2D &copy);

    ~GeodeticPosition_3D() override = default;

    //!
    //! \brief printInfo
    //! \return
    //!
    std::string printInfo() const override
    {
        std::string rtn = "Geodetic Position 3D: " + std::to_string((getLatitude())) + ", " + std::to_string(getLongitude()) + ", " + std::to_string(getAltitude()) + ".";
        return rtn;
    }

    bool areEquivalentFrames(const GeodeticPosition_3D &obj) const;

    //!
    //! \brief updatePosition
    //! \param latitude
    //! \param longitude
    //!
    void updatePosition(const double &latitude, const double &longitude, const double &altitude)
    {
        this->updateTranslationalComponents(latitude,longitude);
        this->setAltitude(altitude);
    }

    //!
    //! \brief updatePosition
    //! \param latitude
    //! \param longitude
    //!
    void updateTranslationalComponents(const double &latitude, const double &longitude)
    {
        this->setLongitude(longitude);
        this->setLatitude(latitude);
    }

    //!
    //! \brief setLatitude
    //! \param latitude
    //!
    void setLatitude(const double &latitude) override
    {
        this->data(1) = latitude;
    }

    //!
    //! \brief setLongitude
    //! \param longitude
    //!
    void setLongitude(const double &longitude) override
    {
        this->data(0) = longitude;
    }

    //!
    //! \brief setAltitude
    //! \param altitude
    //!
    void setAltitude(const double &altitude) override
    {
        this->data(2) = altitude;
    }

    //!
    //! \brief getLatitude
    //! \return
    //!
    double getLatitude() const override
    {
        return  this->data(1);
    }

    //!
    //! \brief getLongitude
    //! \return
    //!
    double getLongitude() const override
    {
        return this->data(0);
    }

    //!
    //! \brief getAltitude
    //! \return
    //!
    double getAltitude() const override
    {
        return this->data(2);
    }

public:

    double distanceBetween3D(const GeodeticPosition_3D &position) const;

    double elevationAngleTo(const GeodeticPosition_3D &position) const;


    /** Interface imposed via state_space::State */
public:
    //!
    //! \brief getStateClone
    //! \return
    //!
    State* getStateClone() const override
    {
        return (new GeodeticPosition_3D(*this));
    }

    //!
    //! \brief getStateClone
    //! \param state
    //!
    void getStateClone(State** state) const override
    {
        *state = new GeodeticPosition_3D(*this);
    }


    /** Interface imposed via Abstract_CartesianPosition */

public:
    //!
    //! \brief getPositionalClone
    //! \return
    //!
    Position* getPositionalClone() const override
    {
        return (new GeodeticPosition_3D(*this));
    }

    //!
    //! \brief getPositionalClone
    //! \param state
    //!
    void getPositionalClone(Position** state) const override
    {
        *state = new GeodeticPosition_3D(*this);
    }

public:
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
    void newPositionFromPolar(Abstract_GeodeticPosition* newObject, const double &distance, const double &bearing) const override;

    //!
    //! \brief newPositionFromPolar
    //! \param distance
    //! \param bearing
    //! \param elevation
    //! \return
    //!
    GeodeticPosition_3D newPositionFromPolar(const double &distance, const double &bearing, const double &elevation) const;

    //!
    //! \brief newPositionFromPolar
    //! \param distance
    //! \param compassBearing
    //! \return
    //!
    void newPositionFromCompass(Abstract_GeodeticPosition *newObject, const double &distance, const double &bearing) const override;


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
    GeodeticPosition_3D& operator = (const GeodeticPosition_3D &rhs)
    {
        Abstract_GeodeticPosition::operator =(rhs);
        Abstract_Altitude::operator =(rhs);
        this->data = rhs.data;
        return *this;
    }

    /** Relational Operators */
public:
    //!
    //! \brief operator ==
    //! \param rhs
    //! \return
    //!
    bool operator == (const GeodeticPosition_3D &rhs) const
    {
        if(!Abstract_GeodeticPosition::operator ==(rhs))
            return false;
        if(!Abstract_Altitude::operator ==(rhs))
            return false;
        if(!this->data.isApprox(rhs.data, std::numeric_limits<double>::epsilon()))
            return false;
        return true;
    }

    //!
    //! \brief operator !=
    //! \param rhs
    //! \return
    //!
    bool operator != (const GeodeticPosition_3D &rhs) const {
        return !(*this == rhs);
    }


public:
    Eigen::Vector3d data;
};

/*!
 * @brief Overloaded plus operator for Vectors.
 */
BASESHARED_EXPORT GeodeticPosition_3D operator +(const GeodeticPosition_3D &a, const GeodeticPosition_3D &b ) = delete;


/*!
 * @brief Overloaded minus operator for Vectors.
 */
BASESHARED_EXPORT GeodeticPosition_3D operator -(const GeodeticPosition_3D &a, const GeodeticPosition_3D &b ) = delete;

} //end of namespace pose
} //end of namespace mace

#endif // GEODETIC_POSITION_3D_H
