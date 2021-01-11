#ifndef GEODETIC_POSITION_3D_H
#define GEODETIC_POSITION_3D_H

#include "../state_space/state.h"
#include "abstract_geodetic_position.h"
#include "abstract_altitude.h"

namespace mace {
namespace pose {

class GeodeticPosition_3D : public Abstract_GeodeticPosition, public Abstract_Altitude, public state_space::State
{
public:

    GeodeticPosition_3D();

    GeodeticPosition_3D(const GeodeticFrameTypes &frameType,
                        const double &latitude, const double &longitude,
                        const AltitudeReferenceTypes &altitudeType, const double &altitude,
                        const std::string &pointName);

    GeodeticPosition_3D(const double &latitude, const double &longitude, const double &altitude, const std::string &pointName = "Geodetic Point");

    GeodeticPosition_3D(const mavlink_global_position_int_t &pos);

    void updateFromPosition(const GeodeticPosition_3D &copy);

    //!
    //! \brief GeodeticPosition_2D
    //! \param copy
    //!
    GeodeticPosition_3D(const GeodeticPosition_3D &copy);

    GeodeticPosition_3D(const GeodeticPosition_2D &copy);

    ~GeodeticPosition_3D() override = default;

    Abstract_GeodeticPosition* getGeodeticClone() const override
    {
        return (new GeodeticPosition_3D(*this));
    }

    void getGeodeticClone(Abstract_GeodeticPosition** state) const override
    {
        *state = new GeodeticPosition_3D(*this);
    }
public:
    mavlink_global_position_int_t getMACE_GlobalPositionInt() const;

    void fromMACEMsg(const mavlink_global_position_int_t &msg);

    mavlink_message_t getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const override;

public:
    Eigen::VectorXd getDataVector() const override
    {
        return this->data;
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
        this->validateDimension(IGNORE_Y_DIMENSION);
    }

    //!
    //! \brief setLongitude
    //! \param longitude
    //!
    void setLongitude(const double &longitude) override
    {
        this->data(0) = longitude;
        this->validateDimension(IGNORE_X_DIMENSION);
    }

    //!
    //! \brief setAltitude
    //! \param altitude
    //!
    void setAltitude(const double &altitude) override
    {
        this->data(2) = altitude;
        this->validateDimension(IGNORE_Z_DIMENSION);
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
    bool hasLatitudeBeenSet() const;

    bool hasLongitudeBeenSet() const;

    bool hasAltitudeBeenSet() const;

    bool hasTranslationalComponentBeenSet() const;

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
    void updateQJSONObject(QJsonObject &obj) const override;

public:
    //!
    //! \brief distanceFromOrigin
    //! \return
    //!
    double distanceFromOrigin() const override;

    //!
    //! \brief translationalDistanceFromOrigin
    //! \return
    //!
    double translationalDistanceFromOrigin() const override;

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
    std::string printPositionalInfo() const override
    {
        std::stringstream stream;
        stream.precision(6);
        stream << std::fixed << "Lat:" << this->getLatitude() << ", Lng:"<< this->getLongitude() << ", Alt: "<<this->getAltitude() <<".";
        return stream.str();
    }

    friend std::ostream& operator<<(std::ostream& os, const GeodeticPosition_3D& t)
    {
        std::stringstream newStream;
        t.printPositionLog(newStream);
        os << newStream.str();
        return os;
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
