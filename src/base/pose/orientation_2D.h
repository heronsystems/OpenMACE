#ifndef ORIENTATION_2D_H
#define ORIENTATION_2D_H

#include <cmath>

#include "Eigen/Geometry"

namespace mace {
namespace pose {

/** A class used to the store the heading anlge (phi) rotation as a functional
 * rotation matrix. This is only to be used when the dimension of the space
 * is 2. The storage of this value is often referred to as a special orthogonal
 * matrix SO(2).
 */

class Orientation_2D : public Eigen::Rotation2D<double>
{
public:
    //!
    //! \brief Orientation_2D
    //!
    Orientation_2D(const std::string &name = "Orientation 2D");

    ~Orientation_2D();

    //!
    //! \brief Orientation_2D
    //! \param copy
    //!
    Orientation_2D(const Orientation_2D &copy);

    //!
    //! \brief Orientation_2D
    //! \param angle
    //!
    Orientation_2D(const double &angle);

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
    Orientation_2D operator + (const Orientation_2D &that) const
    {
        Orientation_2D newObj(*this);
        newObj.angle() = newObj.angle() + that.angle();
        return newObj;
    }

    //!
    //! \brief operator -
    //! \param that
    //! \return
    //!
    Orientation_2D operator - (const Orientation_2D &that) const
    {
        Orientation_2D newObj(*this);
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
    bool operator < (const Orientation_2D &rhs) const
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
    bool operator >= (const Orientation_2D &rhs) const
    {
        return !(*this < rhs);
    }

    //!
    //! \brief operator >
    //! \param rhs
    //! \return
    //!
    bool operator > (const Orientation_2D &rhs) const
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
    bool operator <= (const Orientation_2D &rhs) const
    {
        return !(this->angle() > rhs.angle());
    }

    //!
    //! \brief operator ==
    //! \param rhs
    //! \return
    //!
    bool operator == (const Orientation_2D &rhs) const
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
    bool operator != (const Orientation_2D &rhs) {
        return !(*this == rhs);
    }

    /** Assignment Operators */
public:

    //!
    //! \brief operator =
    //! \param rhs
    //! \return
    //!
    Orientation_2D& operator = (const Orientation_2D &rhs)
    {
        Eigen::Rotation2D<double>::operator=(rhs);
        return *this;
    }

    //!
    //! \brief operator +=
    //! \param that
    //! \return
    //!
    Orientation_2D& operator += (const Orientation_2D &rhs)
    {
        Eigen::Rotation2D<double>::operator=(rhs);

        return *this;
    }

    //!
    //! \brief operator -=
    //! \param that
    //! \return
    //!
    Orientation_2D& operator -= (const Orientation_2D &rhs)
    {
        this->angle() -= rhs.angle();
        return *this;
    }


    /** Protected Members */
protected:
    std::string name = "";
};

} //end of namespace pose
} //end of namespace mace

#endif // ORIENTATION_2D_H
