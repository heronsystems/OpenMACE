#ifndef CARTESIAN_POSITION_3D_H
#define CARTESIAN_POSITION_3D_H

#include "abstract_altitude.h"
#include "abstract_cartesian_position.h"
#include "../state_space/state.h"

using namespace mace::math;

namespace mace {
namespace pose {

class CartesianPosition_3D : public Abstract_CartesianPosition, public Abstract_Altitude, public state_space::State
{
public:

    CartesianPosition_3D(const CartesianFrameTypes &frameType = CartesianFrameTypes::CF_LOCAL_UNKNOWN,
                        const double &x = 0.0, const double &y = 0.0,
                        const AltitudeReferenceTypes &altitudeType = AltitudeReferenceTypes::REF_ALT_UNKNOWN, const double &z = 0.0,
                        const std::string &pointName = "Cartesian Point");

    CartesianPosition_3D(const double &x, const double &y, const double &z, const std::string &pointName = "Cartesian Point");

    CartesianPosition_3D(const CartesianPosition_3D &copy);

    CartesianPosition_3D(const CartesianPosition_2D &copy);


    ~CartesianPosition_3D() override = default;

    Abstract_CartesianPosition* getCartesianClone() const override
    {
        return (new CartesianPosition_3D(*this));
    }

    void getCartesianClone(Abstract_CartesianPosition** state) const override
    {
        *state = new CartesianPosition_3D(*this);
    }

    std::string printInfo() const override
    {
        std::string rtn = "Cartesian Position 3D: " + std::to_string(getXPosition()) + ", " + std::to_string(getYPosition()) + ", " + std::to_string(getZPosition()) +  ".";
        return rtn;
    }

    bool areEquivalentFrames(const CartesianPosition_3D &obj) const;

    Eigen::VectorXd getDataVector() const override
    {
        return this->data;
    }

public:
    void updatePosition(const double &x, const double &y, const double &z)
    {
        this->setTranslationalPosition(x,y);
        this->setAltitude(z);
    }

    void setTranslationalPosition(const double &x, const double &y)
    {
        this->setXPosition(x);
        this->setYPosition(y);
    }
    void setXPosition(const double &x)
    {
        this->data(0) = x;
    }

    void setYPosition(const double &y)
    {
        this->data(1) = y;
    }

    void setZPosition(const double &z)
    {
        this->data(2) = z;
    }

    double getXPosition() const
    {
        return this->data(0);
    }

    double getYPosition() const
    {
        return this->data(1);
    }

    double getZPosition() const
    {
        return this->data(2);
    }

public:
    double deltaX(const CartesianPosition_3D &that) const;
    double deltaY(const CartesianPosition_3D &that) const;
    double deltaZ(const CartesianPosition_3D &that) const;

    /** Interface imposed via AltitudeInterface */

    //!
    //! \brief setAltitude
    //! \param altitude
    //!
    void setAltitude(const double &altitude) override;

    //!
    //! \brief getAltitude
    //! \return
    //!
    double getAltitude() const override;

    //!
    //! \brief elevationAngleFromOrigin
    //! \return
    //!
    double elevationAngleFromOrigin() const override;

    //!
    //! \brief elevationAngleTo
    //! \param position
    //! \return
    //!
    double elevationAngleTo(const Abstract_CartesianPosition* position) const;



    /** Interface imposed via state_space::State */
public:
    State* getStateClone() const override
    {
        return (new CartesianPosition_3D(*this));
    }

    void getStateClone(State** state) const override
    {
        *state = new CartesianPosition_3D(*this);
    }


    /** Interface imposed via Abstract_CartesianPosition */

public:
    Position* getPositionalClone() const override
    {
        return (new CartesianPosition_3D(*this));
    }

    void getPositionalClone(Position** state) const override
    {
        *state = new CartesianPosition_3D(*this);
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
    double distanceBetween2D(const Abstract_CartesianPosition* pos) const override;

    double distanceBetween3D(const Abstract_CartesianPosition* pos) const;

    //!
    //! \brief distanceTo
    //! \param position
    //! \return
    //!
    double distanceTo(const Abstract_CartesianPosition* pos) const override;

    //!
    //! \brief polarBearingTo
    //! \param position
    //! \return
    //!
    double polarBearingTo(const Abstract_CartesianPosition* pos) const override;

    //!
    //! \brief polarBearingTo
    //! \param position
    //! \return
    //!
    double compassBearingTo(const Abstract_CartesianPosition* pos) const override;

    //!
    //! \brief newPositionFromPolar
    //! \param distance
    //! \param compassBearing
    //! \return
    //!
    void newPositionFromPolar(Abstract_CartesianPosition* newObject, const double &distance, const double &bearing) const override;

    //!
    //! \brief newPositionFromPolar
    //! \param distance
    //! \param compassBearing
    //! \return
    //!
    void newPositionFromCompass(Abstract_CartesianPosition *newObject, const double &distance, const double &bearing) const override;

    void applyPositionalShiftFromPolar(const double &distance, const double &bearing) override;

    void applyPositionalShiftFromPolar(const double &distance, const double &bearing, const double &elevation);

    void applyPositionalShiftFromCompass(const double &distance, const double &bearing) override;

    void applyPositionalShiftFromCompass(const double &distance, const double &bearing, const double &elevation);


    /** Assignment Operators */
public:
    CartesianPosition_3D& operator = (const CartesianPosition_3D &rhs)
    {
        Abstract_CartesianPosition::operator =(rhs);
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
    bool operator == (const CartesianPosition_3D &rhs) const
    {
        if(!Abstract_CartesianPosition::operator ==(rhs))
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
    bool operator != (const CartesianPosition_3D &rhs) const {
        return !(*this == rhs);
    }
public:
    friend std::ostream& operator<<(std::ostream& os, const CartesianPosition_3D& t);

    CartesianPosition_3D& operator+= (const CartesianPosition_3D &rhs)
    {

        if(this->areEquivalentFrames(rhs))
        {
            this->data += rhs.data;
            return *this;
        }
        else
            throw std::logic_error("Tried to perform a + operation between 2DCartesians of differnet coordinate frames.");
    }

public:
    Eigen::Vector3d data;
};


/*!
 * @brief Overloaded plus operator for Vectors.
 */
BASESHARED_EXPORT CartesianPosition_3D operator +(const CartesianPosition_3D &a, const CartesianPosition_3D &b );

/*!
 * @brief Overloaded minus operator for Vectors.
 */
BASESHARED_EXPORT CartesianPosition_3D operator -(const CartesianPosition_3D &a, const CartesianPosition_3D &b );

} //end of namespace pose
} //end of namespace mace

#endif // CARTESIAN_POSITION_3D_H
