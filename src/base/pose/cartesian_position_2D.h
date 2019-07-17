#ifndef CARTESIAN_POSITION_2D_H
#define CARTESIAN_POSITION_2D_H

#include "abstract_cartesian_position.h"
#include "base/state_space/state.h"

namespace mace{
namespace pose {

class CartesianPosition_2D : public Abstract_CartesianPosition,
        public state_space::State
{
public:
    CartesianPosition_2D(const std::string &pointName):
        Abstract_CartesianPosition(CartesianFrameTypes::CF_LOCAL_ENU, pointName), State()
    {
        this->dimension = 2;
    }

    CartesianPosition_2D(const std::string &pointName = "Position Point", const double &x = 0.0, const double &y = 0.0):
        Abstract_CartesianPosition(CartesianFrameTypes::CF_LOCAL_ENU, pointName), State()
    {
        this->dimension = 2;
    }

    ~CartesianPosition_2D() override = default;

    CartesianPosition_2D(const CartesianPosition_2D &copy):
        Abstract_CartesianPosition(copy), state_space::State(copy)
    {

    }

//    CartesianPosition_2D(const CartesianPosition_3D &copy):
//        Abstract_CartesianPosition(copy), Data2D(copy), state_space::State(copy)
//    {

//    }

    std::string printInfo() const override
    {
        std::string rtn = "Cartesian Position 2D: " + std::to_string(getXPosition()) + ", " + std::to_string(getYPosition()) + ".";
        return rtn;
    }

public:

    void normalize();

    void scale(const double &value);

public:
    void updatePosition(const double &x, const double &y)
    {
        this->setData_2D(x,y);
    }

    void setXPosition(const double &x)
    {
        this->setX(x);
    }

    void setYPosition(const double &y)
    {
        this->setY(y);
    }

    double getXPosition() const
    {
        return this->getX();
    }

    double getYPosition() const
    {
        return this->getY();
    }

    Eigen::Vector2d getAsVector()
    {
        Eigen::Vector2d vec(this->getX(), this->getY());
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
public:
    double deltaX(const CartesianPosition_2D &that) const;
    double deltaY(const CartesianPosition_2D &that) const;



    /** Interface imposed via state_space::State */
public:
    State* getStateClone() const override
    {
        return (new CartesianPosition_2D(*this));
    }

    void getStateClone(State** state) const override
    {
        *state = new CartesianPosition_2D(*this);
    }


    /** Interface imposed via Abstract_CartesianPosition */

public:
    Position* getPositionalClone() const override
    {
        return (new CartesianPosition_2D(*this));
    }

    void getPositionalClone(Position** state) const override
    {
        *state = new CartesianPosition_2D(*this);
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
    CartesianPosition_2D operator + (const CartesianPosition_2D &that) const
    {
        CartesianPosition_2D newPoint(*this);
        newPoint.x = this->x + that.x;
        newPoint.y = this->y + that.y;
        return newPoint;
    }

    //!
    //! \brief operator -
    //! \param that
    //! \return
    //!
    CartesianPosition_2D operator - (const CartesianPosition_2D &that) const
    {
        CartesianPosition_2D newPoint(*this);

        newPoint.x = this->x - that.x;
        newPoint.y = this->y - that.y;

        return newPoint;
    }

public:
    friend std::ostream& operator<<(std::ostream& os, const CartesianPosition_2D& t);

};

} //end of namespace pose
} //end of namespace mace

#endif // CARTESIAN_POSITION_2D_H
