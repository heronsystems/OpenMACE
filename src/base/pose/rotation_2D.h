#ifndef ROTATION_2D_H
#define ROTATION_2D_H

#include <cmath>

#include <Eigen/Geometry>
#include "abstract_orientation.h"

namespace mace {
namespace pose {

/** A class used to the store the heading anlge (phi) rotation as a functional
 * rotation matrix. This is only to be used when the dimension of the space
 * is 2. The storage of this value is often referred to as a special orthogonal
 * matrix SO(2).
 */

class Rotation_2D : public AbstractOrientation, public Eigen::Rotation2D<double>
{
public:
    //!
    //! \brief Orientation_2D
    //!
    Rotation_2D(const std::string &name = "Orientation 2D");

    ~Rotation_2D();

    //!
    //! \brief Orientation_2D
    //! \param copy
    //!
    Rotation_2D(const Rotation_2D &copy);

    //!
    //! \brief Orientation_2D
    //! \param angle
    //!
    Rotation_2D(const double &angle);

public:
    //!
    //! \brief setPhi
    //! \param angle
    //!
    void setPhi(const double &angle);

    //!
    //! \brief getPhi
    //! \return
    //!
    double getPhi() const;


    /** Arithmetic Operators */
public:

    //!
    //! \brief operator +
    //! \param that
    //! \return
    //!
    Rotation_2D operator + (const Rotation_2D &that) const
    {
        Rotation_2D newObj(*this);
        newObj.angle() = newObj.angle() + that.angle();
        return newObj;
    }

    //!
    //! \brief operator -
    //! \param that
    //! \return
    //!
    Rotation_2D operator - (const Rotation_2D &that) const
    {
        Rotation_2D newObj(*this);
        newObj.angle() = newObj.angle() - that.angle();
        return newObj;
    }

    /** Relational Operators */
public:

    //!
    //! \brief operator <
    //! \param rhs
    //! \return
    //!
    bool operator < (const Rotation_2D &rhs) const
    {
        if(this->angle() >= rhs.angle())
            return false;
        return true;
    }

    //!
    //! \brief operator >=
    //! \param rhs
    //! \return
    //!
    bool operator >= (const Rotation_2D &rhs) const
    {
        return !(*this < rhs);
    }

    //!
    //! \brief operator >
    //! \param rhs
    //! \return
    //!
    bool operator > (const Rotation_2D &rhs) const
    {
        if(this->angle() <= rhs.angle())
            return false;
        return true;
    }

    //!
    //! \brief operator <=
    //! \param rhs
    //! \return
    //!
    bool operator <= (const Rotation_2D &rhs) const
    {
        return !(this->angle() > rhs.angle());
    }

    //!
    //! \brief operator ==
    //! \param rhs
    //! \return
    //!
    bool operator == (const Rotation_2D &rhs) const
    {
        if(!Eigen::Rotation2D<double>::isApprox(rhs,std::numeric_limits<double>::epsilon()))
            return false;
        return true;
    }

    //!
    //! \brief operator !=
    //! \param rhs
    //! \return
    //!
    bool operator != (const Rotation_2D &rhs) {
        return !(*this == rhs);
    }

    /** Assignment Operators */
public:

    //!
    //! \brief operator =
    //! \param rhs
    //! \return
    //!
    Rotation_2D& operator = (const Rotation_2D &rhs)
    {
        AbstractOrientation::operator=(rhs);
        Eigen::Rotation2D<double>::operator=(rhs);
        return *this;
    }

    //!
    //! \brief operator +=
    //! \param that
    //! \return
    //!
    Rotation_2D& operator += (const Rotation_2D &rhs)
    {
        Eigen::Rotation2D<double>::operator=(rhs);

        return *this;
    }

    //!
    //! \brief operator -=
    //! \param that
    //! \return
    //!
    Rotation_2D& operator -= (const Rotation_2D &rhs)
    {
        this->angle() -= rhs.angle();
        return *this;
    }
};

} //end of namespace pose
} //end of namespace mace

#endif // ROTATION_2D_H
