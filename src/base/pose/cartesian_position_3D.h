#ifndef CARTESIAN_POSITION_3D_H
#define CARTESIAN_POSITION_3D_H

#include "base/state_space/state.h"
#include "abstract_cartesian_position.h"
#include "abstract_altitude.h"

using namespace mace::math;

namespace mace {
namespace pose {

class CartesianPosition_3D : public Abstract_CartesianPosition, public Abstract_Altitude,
        public state_space::State
{
public:
    CartesianPosition_3D(const std::string &pointName);

    CartesianPosition_3D(const double &x, const double &y, const double &z);

    CartesianPosition_3D(const std::string &pointName = "Position Point", const double &x = 0.0, const double &y = 0.0, const double &z = 0.0);

    CartesianPosition_3D(const CartesianPosition_3D &copy);

    CartesianPosition_3D(const CartesianPosition_2D &copy);


    ~CartesianPosition_3D() override = default;

    std::string printInfo() const override
    {
        std::string rtn = "Cartesian Position 3D: " + std::to_string(getXPosition()) + ", " + std::to_string(getYPosition()) + ", " + std::to_string(getZPosition()) +  ".";
        return rtn;
    }

    bool areEquivalentFrames(const CartesianPosition_3D &obj) const;

public:
    void normalize();

    void scale(const double &value);

public:
    void updatePosition(const double &x, const double &y, const double &z)
    {
        this->setData_2D(x,y);
        this->setData_1D(z);
    }

    void setXPosition(const double &x)
    {
        this->setX(x);
    }

    void setYPosition(const double &y)
    {
        this->setY(y);
    }

    void setZPosition(const double &z)
    {
        this->setZ(z);
    }

    double getXPosition() const
    {
        return this->getX();
    }

    double getYPosition() const
    {
        return this->getY();
    }

    double getZPosition() const
    {
        return this->getZ();
    }

    Eigen::Vector3d getAsVector()
    {
        Eigen::Vector3d vec(this->getX(), this->getY(), this->getZ());
        return vec;
    }

    bool hasXBeenSet() const
    {
        return this->getDataXFlag();
    }

    bool hasYBeenSet() const
    {
        return this->getDataYFlag();
    }

    bool hasZBeenSet() const
    {
        return this->getDataZFlag();
    }

public:
    double deltaX(const CartesianPosition_3D &that) const;
    double deltaY(const CartesianPosition_3D &that) const;
    double deltaZ(const CartesianPosition_3D &that) const;

    /** Interface imposed via AltitudeInterface */

    //!
    //! \brief hasAltitudeBeenSet
    //! \return
    //!
    bool hasAltitudeBeenSet() const override;


    //!
    //! \brief elevationFromOrigin
    //! \return
    //!
    double elevationFromOrigin() const override;


    //!
    //! \brief deltaAltitude
    //! \param pos
    //! \return
    //!
    double deltaAltitude(const Abstract_Altitude* pos) const override;


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
    bool hasBeenSet() const override
    {
        return hasXBeenSet() || hasYBeenSet() || hasZBeenSet();
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
        return *this;
    }

    /** Arithmetic Operators */
public:
    CartesianPosition_3D operator+ (const CartesianPosition_3D &rhs)
    {
        CartesianPosition_3D newPoint(*this);

        if(this->areEquivalentFrames(rhs))
        {
            newPoint.x = newPoint.x + rhs.x;
            newPoint.y = newPoint.y + rhs.y;
            newPoint.z = newPoint.z + rhs.z;
        }
        else
        {
            throw std::logic_error("Tried to perform a + operation between 3DCartesians of differnet coordinate frames.");
        }

        return newPoint;
    }

    CartesianPosition_3D& operator += (const CartesianPosition_3D &rhs)
    {
        if(this->areEquivalentFrames(rhs))
        {
            this->x += rhs.x;
            this->y += rhs.y;
            this->z += rhs.z;
        }
        else
            throw std::logic_error("Tried to perform a + operation between 3DCartesians of differnet coordinate frames.");

        return *this;
    }

    CartesianPosition_3D operator- (const CartesianPosition_3D &rhs)
    {
        CartesianPosition_3D newPoint(*this);

        if(this->areEquivalentFrames(rhs))
        {
            newPoint.x = newPoint.x - rhs.x;
            newPoint.y = newPoint.y - rhs.y;
            newPoint.z = newPoint.z - rhs.z;
        }
        else
            throw std::logic_error("Tried to perform a - operation between 3DCartesians of differnet coordinate frames.");
        return newPoint;
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
};


} //end of namespace pose
} //end of namespace mace

#endif // CARTESIAN_POSITION_3D_H
