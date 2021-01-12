#ifndef GEODETIC_POSITION_2D_H
#define GEODETIC_POSITION_2D_H

#include "abstract_geodetic_position.h"
#include "../state_space/state.h"

namespace mace {
namespace pose {

class GeodeticPosition_2D : public Abstract_GeodeticPosition, public state_space::State
{

public:

    //!
    //! \brief GeodeticPosition_2D
    //!
    GeodeticPosition_2D();

    //!
    //! \brief GeodeticPosition_2D
    //! \param pointName
    //! \param latitude
    //! \param longitude
    //!
    GeodeticPosition_2D(const GeodeticFrameTypes &frameType,
                        const double &latitude, const double &longitude,
                        const std::string &pointName);

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


    Abstract_GeodeticPosition* getGeodeticClone() const override
    {
        return (new GeodeticPosition_2D(*this));
    }

    void getGeodeticClone(Abstract_GeodeticPosition** state) const override
    {
        *state = new GeodeticPosition_2D(*this);
    }

    bool areEquivalentFrames(const GeodeticPosition_2D &obj) const;

    Eigen::VectorXd getDataVector() const override
    {
        return this->data;
    }

public:
    void updateQJSONObject(QJsonObject &obj) const override;

public:

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

public:
    bool hasLatitudeBeenSet() const;

    bool hasLongitudeBeenSet() const;

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

public:
    mavlink_global_position_int_t getMACE_GlobalPositionInt() const;

    mavlink_message_t getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const override;

    /** Assignment Operators */
public:
    GeodeticPosition_2D& operator = (const GeodeticPosition_2D &rhs)
    {
        Abstract_GeodeticPosition::operator =(rhs);
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
    bool operator == (const GeodeticPosition_2D &rhs) const
    {
        if(!Abstract_GeodeticPosition::operator ==(rhs))
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
    bool operator != (const GeodeticPosition_2D &rhs) const {
        return !(*this == rhs);
    }

public:
    std::string printPositionalInfo() const override
    {
        std::stringstream stream;
        stream.precision(6);
        stream << std::fixed << "Geodetic Position 2D: Lat:" << this->getLatitude() << ", Lng:"<< this->getLongitude() << ".";
        return stream.str();
    }

    friend std::ostream& operator<<(std::ostream& os, const GeodeticPosition_2D& t)
    {
        std::stringstream newStream;
        t.printPositionLog(newStream);
        os << newStream.str();
        return os;
    }

public:
    
    Eigen::Vector2d data;
};

/*!
 * @brief Overloaded plus operator for Vectors.
 */
BASESHARED_EXPORT GeodeticPosition_2D operator +(const GeodeticPosition_2D &a, const GeodeticPosition_2D &b ) = delete;


/*!
 * @brief Overloaded minus operator for Vectors.
 */
BASESHARED_EXPORT GeodeticPosition_2D operator -(const GeodeticPosition_2D &a, const GeodeticPosition_2D &b ) = delete;

} //end of namespace pose
} //end of namespace mace

#endif // GEODETIC_POSITION_2D_H
