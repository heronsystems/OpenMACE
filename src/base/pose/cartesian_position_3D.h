#ifndef CARTESIAN_POSITION_3D_H
#define CARTESIAN_POSITION_3D_H

#include "base_position.h"
#include "cartesian_position_2D.h"
#include "base/state_space/state.h"

using namespace mace::math;

namespace mace {
namespace pose {

class CartesianPosition_3D : public Abstract_CartesianPosition, public misc::Data3D,
        public state_space::State
{
public:
    CartesianPosition_3D(const std::string &pointName):
        Abstract_CartesianPosition(CartesianFrameTypes::CF_LOCAL_ENU, pointName), Data3D(), State()
    {

    }

    CartesianPosition_3D(const std::string &pointName = "Position Point", const double &x = 0.0, const double &y = 0.0, const double &z = 0.0):
        Abstract_CartesianPosition(CartesianFrameTypes::CF_LOCAL_ENU, pointName), Data3D(x,y,z), State()
    {

    }

    CartesianPosition_3D(const CartesianPosition_3D &copy):
        Abstract_CartesianPosition(copy), Data3D(copy), state_space::State(copy)
    {

    }

    CartesianPosition_3D(const CartesianPosition_2D &copy):
        Abstract_CartesianPosition(copy), Data3D(copy), state_space::State(copy)
    {

    }

    ~CartesianPosition_3D() override = default;

    std::string printInfo() const override
    {
        std::string rtn = "Cartesian Position 3D: " + std::to_string(getXPosition()) + ", " + std::to_string(getYPosition()) + ", " + std::to_string(getZPosition()) +  ".";
        return rtn;
    }

public:
    void normalize();

    void scale(const double &value);

public:
    void updatePosition(const double &x, const double &y, const double &z)
    {
        this->setData(x,y,z);
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
        return hasXBeenSet() || hasYBeenSet();
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

    void applyPositionalShiftFromCompass(const double &distance, const double &bearing) override;


    /** Arithmetic Operators */
public:

    //!
    //! \brief operator +
    //! \param that
    //! \return
    //!
    CartesianPosition_3D operator + (const CartesianPosition_3D &that) const
    {
        CartesianPosition_3D newPoint(*this);
        newPoint.x = this->x + that.x;
        newPoint.y = this->y + that.y;
        newPoint.z = this->y + that.z;
        return newPoint;
    }


    //!
    //! \brief operator -
    //! \param that
    //! \return
    //!
    CartesianPosition_3D operator - (const CartesianPosition_3D &that) const
    {
        CartesianPosition_3D newPoint(*this);

        newPoint.x = this->x - that.x;
        newPoint.y = this->y - that.y;
        newPoint.z = this->z - that.z;
        return newPoint;
    }


public:
    friend std::ostream& operator<<(std::ostream& os, const CartesianPosition_3D& t);
};

} //end of namespace pose
} //end of namespace mace

#endif // CARTESIAN_POSITION_3D_H
