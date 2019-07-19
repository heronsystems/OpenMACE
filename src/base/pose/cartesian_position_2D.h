#ifndef CARTESIAN_POSITION_2D_H
#define CARTESIAN_POSITION_2D_H

#include "abstract_cartesian_position.h"
#include "base/state_space/state.h"

namespace mace{
namespace pose {

class CartesianPosition_2D : public Abstract_CartesianPosition, public state_space::State
{

public:

    explicit CartesianPosition_2D(const CartesianFrameTypes &frameType = CartesianFrameTypes::CF_LOCAL_UNKNOWN,
                        const double &x = 0.0, const double &y = 0.0,
                        const std::string &pointName = "Cartesian Point");

    explicit CartesianPosition_2D(const std::string &pointName,
                        const double &x, const double &y);

    explicit CartesianPosition_2D(const double &x, const double &y);

    CartesianPosition_2D(const CartesianPosition_2D &copy);

    CartesianPosition_2D(const CartesianPosition_3D &copy);

    ~CartesianPosition_2D() override = default;

    std::string printInfo() const override
    {
        std::string rtn = "Cartesian Position 2D: " + std::to_string(getXPosition()) + ", " + std::to_string(getYPosition()) + ".";
        return rtn;
    }

    bool areEquivalentFrames(const CartesianPosition_2D &obj) const;

public:
    void updatePosition(const double &x, const double &y)
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

    double getXPosition() const
    {
        return this->data(0);
    }

    double getYPosition() const
    {
        return this->data(1);
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


    double distanceBetween2D(const CartesianPosition_2D &pos) const;

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

    /** Assignment Operators */
public:
    CartesianPosition_2D& operator = (const CartesianPosition_2D &rhs)
    {
        Abstract_CartesianPosition::operator =(rhs);
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
    bool operator == (const CartesianPosition_2D &rhs) const
    {
        if(!Abstract_CartesianPosition::operator ==(rhs))
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
    bool operator != (const CartesianPosition_2D &rhs) const {
        return !(*this == rhs);
    }

public:
    friend std::ostream& operator<<(std::ostream& os, const CartesianPosition_2D& t);

    CartesianPosition_2D& operator+= (const CartesianPosition_2D &rhs)
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
    Eigen::Vector2d data;
};

/*!
 * @brief Overloaded plus operator for Vectors.
 */
BASESHARED_EXPORT CartesianPosition_2D operator +(const CartesianPosition_2D &a, const CartesianPosition_2D &b );


/*!
 * @brief Overloaded minus operator for Vectors.
 */
BASESHARED_EXPORT CartesianPosition_2D operator -(const CartesianPosition_2D &a, const CartesianPosition_2D &b );


} //end of namespace pose
} //end of namespace mace

#endif // CARTESIAN_POSITION_2D_H
