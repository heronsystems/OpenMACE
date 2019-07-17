#ifndef GEODETIC_POSITION_3D_H
#define GEODETIC_POSITION_3D_H

#include "base/state_space/state.h"
#include "abstract_geodetic_position.h"
#include "abstract_altitude.h"

namespace mace {
namespace pose {

class GeodeticPosition_3D : public Abstract_GeodeticPosition, public Abstract_Altitude,
    public state_space::State
{
public:

    GeodeticPosition_3D(const GeodeticFrameTypes &frameType = GeodeticFrameTypes::CF_GLOBAL_RELATIVE_ALT,
                        const double &latitude = 0.0, const double &longitude = 0.0,
                        const AltitudeReferenceTypes &altitudeType = AltitudeReferenceTypes::REF_ALT_UNKNOWN, const double &altitude = 0.0,
                        const std::string &pointName = "Geodetic Point");

    GeodeticPosition_3D(const std::string &pointName,
                        const double &latitude, const double &longitude, const double &altitude);

    GeodeticPosition_3D(const double &latitude, const double &longitude, const double &altitude);

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
        this->updateTranslationalComponent(latitude,longitude);
        this->updateElevationComponent(altitude);
    }

    void updateTranslationalComponent(const double &latitude, const double &longitude)
    {
        this->setData_2D(latitude,longitude);
    }

    void updateElevationComponent(const double &altitude)
    {
        this->setData_1D(altitude);
    }

    //!
    //! \brief hasBeenSet
    //! \return
    //!
    bool hasBeenSet() const override
    {
        return hasLatitudeBeenSet() || hasLongitudeBeenSet() || hasAltitudeBeenSet();
    }

public:

    //!
    //! \brief getAsVector
    //! \return
    //!
    Eigen::Vector3d getAsVector()
    {
        Eigen::Vector3d vec(this->getLatitude(), this->getLongitude(), this->getAltitude());
        return vec;
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
        return *this;
    }

    /** Arithmetic Operators */
public:

    //!
    //! \brief operator +
    //! \param that
    //! \return
    //!
    GeodeticPosition_3D operator + (const GeodeticPosition_3D &rhs) const
    {
        GeodeticPosition_3D newPoint(*this);

        if(this->areEquivalentFrames(rhs))
        {
            newPoint.x = newPoint.x + rhs.x;
            newPoint.y = newPoint.y + rhs.y;
            newPoint.z = newPoint.z + rhs.z;
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
    GeodeticPosition_3D operator - (const GeodeticPosition_3D &rhs) const
    {
        GeodeticPosition_3D newPoint(*this);

        if(this->areEquivalentFrames(rhs))
        {
            newPoint.x = newPoint.x - rhs.x;
            newPoint.y = newPoint.y - rhs.y;
            newPoint.z = newPoint.z - rhs.z;
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
    bool operator == (const GeodeticPosition_3D &rhs) const
    {
        if(!Abstract_GeodeticPosition::operator ==(rhs))
            return false;
        if(!Abstract_Altitude::operator ==(rhs))
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
};

} //end of namespace pose
} //end of namespace mace

#endif // GEODETIC_POSITION_3D_H
