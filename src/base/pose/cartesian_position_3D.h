#ifndef CARTESIAN_POSITION_3D_H
#define CARTESIAN_POSITION_3D_H

#include "base_position.h"
#include "cartesian_position_2D.h"
#include "base/state_space/state.h"

using namespace mace::math;

namespace mace {
namespace pose {

class CartesianPosition_3D: public AbstractPosition<CartesianPosition_3D, misc::Data3D>, public CartesianPosition,
        public state_space::State
{
public:
    CartesianPosition_3D():
        AbstractPosition(AbstractPosition::PositionType::CARTESIAN, CoordinateFrame::CF_LOCAL_ENU)
    {

    }

    ~CartesianPosition_3D() = default;

    CartesianPosition_3D(const CartesianPosition_3D &copy):
        AbstractPosition(copy), state_space::State(copy)
    {

    }

    CartesianPosition_3D(const double x, const double &y, const double &z):
        AbstractPosition(AbstractPosition::PositionType::CARTESIAN, CoordinateFrame::CF_LOCAL_ENU)
    {
        this->data.setData(x,y,z);
    }

    State* getClone() const override
    {
        return (new CartesianPosition_3D(*this));
    }

    void getClone(State** state) const override
    {
        *state = new CartesianPosition_3D(*this);
    }

public:
    void updatePosition(const double &x, const double &y, const double &z)
    {
        this->data.setData(x,y,z);
    }

    void setXPosition(const double &x)
    {
        this->data.setX(x);
    }

    void setYPosition(const double &y)
    {
        this->data.setY(y);
    }

    void setZPosition(const double &z)
    {
        this->data.setZ(z);
    }

    double getXPosition() const
    {
        return this->data.getX();
    }

    double getYPosition() const
    {
        return this->data.getY();
    }

    double getZPosition() const
    {
        return this->data.getZ();
    }

    Eigen::Vector3d getAsVector()
    {
        Eigen::Vector3d vec(this->data.getX(), this->data.getY(), this->data.getZ());
        return vec;
    }

    bool hasXBeenSet() const
    {
        return this->data.getDataXFlag();
    }

    bool hasYBeenSet() const
    {
        return this->data.getDataYFlag();
    }

    bool hasZBeenSet() const
    {
        return this->data.getDataZFlag();
    }

public:
    double deltaX(const CartesianPosition_3D &that) const;
    double deltaY(const CartesianPosition_3D &that) const;
    double deltaZ(const CartesianPosition_3D &that) const;

public:
    void setCoordinateFrame(const LocalFrameType &desiredFrame)
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
    CartesianPosition_3D operator + (const CartesianPosition_3D &that) const
    {
        CartesianPosition_3D newPoint(*this);
        newPoint.data = this->data + that.data;
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
        newPoint.data = this->data - that.data;
        return newPoint;
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
    //! \brief elevationFromOrigin
    //! \return
    //!
    double elevationFromOrigin() const override;

    //!
    //! \brief polarAzimuthTo
    //! \param position
    //! \return
    //!
    double elevationAngleTo(const CartesianPosition_3D &position) const;

    //!
    //! \brief deltaAltitude
    //! \param position
    //! \return
    //!
    double deltaAltitude(const CartesianPosition_3D &position) const;


    //!
    //! \brief distanceBetween2D
    //! \param position
    //! \return
    //!
    virtual double distanceBetween2D(const CartesianPosition_3D &pos) const override;

    //!
    //! \brief distanceBetween3D
    //! \param position
    //! \return
    //!
    double distanceBetween3D(const CartesianPosition_3D &position) const;

    //!
    //! \brief distanceTo
    //! \param position
    //! \return
    //!
    double distanceTo(const CartesianPosition_3D &pos) const override;

    //!
    //! \brief polarBearingTo
    //! \param position
    //! \return
    //!
    double polarBearingTo(const CartesianPosition_3D &pos) const override;

    //!
    //! \brief polarBearingTo
    //! \param position
    //! \return
    //!
    double compassBearingTo(const CartesianPosition_3D &pos) const override;

    //!
    //! \brief newPositionFromPolar
    //! \param distance
    //! \param compassBearing
    //! \return
    //!
    CartesianPosition_3D newPositionFromPolar(const double &distance, const double &bearing) const override;

    //!
    //! \brief newPositionFromPolar
    //! \param distance
    //! \param compassBearing
    //! \return
    //!
    CartesianPosition_3D newPositionFromCompass(const double &distance, const double &bearing) const override;

    void applyPositionalShiftFromPolar(const double &distance, const double &bearing) override;

    void applyPositionalShiftFromPolar(const double &distance, const double &bearing, const double &elevation);

    void applyPositionalShiftFromCompass(const double &distance, const double &bearing) override;

    void applyPositionalShiftFromCompass(const double &distance, const double &bearing, const double &elevation);

};

} //end of namespace pose
} //end of namespace mace

#endif // CARTESIAN_POSITION_3D_H
