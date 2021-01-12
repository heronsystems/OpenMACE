#ifndef ROTATION_2D_H
#define ROTATION_2D_H

#include <cmath>

#include <Eigen/Geometry>
#include "abstract_rotation.h"

namespace mace {
namespace pose {

/** A class used to the store the heading anlge (phi) rotation as a functional
 * rotation matrix. This is only to be used when the dimension of the space
 * is 2. The storage of this value is often referred to as a special orthogonal
 * matrix SO(2).
 */

class Rotation_2D : public AbstractRotation, public Eigen::Rotation2D<double>
{
public:


public:
    enum TYPEMASK_YAW
    {
        YAW_DIMENSION_VALID = 0,
        IGNORE_YAW_DIMENSION = 1024,
        IGNORE_YAW_RATE_DIMENSION = 2048
    };

public:
    static const uint16_t ignoreAllPositions = IGNORE_YAW_DIMENSION|IGNORE_YAW_RATE_DIMENSION;

public:
    //!
    //! \brief Orientation_2D
    //!
    Rotation_2D(const std::string &name = "Orientation 2D");

    virtual ~Rotation_2D() override = default;

    //!
    //! \brief Orientation_2D
    //! \param copy
    //!
    Rotation_2D(const Rotation_2D &copy);

    //!
    //! \brief Rotation_2D
    //! \param copy
    //!
    Rotation_2D(const Rotation_3D &copy);

    //!
    //! \brief Orientation_2D
    //! \param angle
    //!
    Rotation_2D(const double &angle);

public:
    AbstractRotation* getRotationalClone() const override
    {
        return (new Rotation_2D(*this));
    }

    void getRotationalClone(AbstractRotation** state) const override
    {
         *state = new Rotation_2D(*this);
    }

public:
    mavlink_attitude_quaternion_t getMACEQuaternion() const override;

    mavlink_attitude_t getMACEEuler() const override;

    mavlink_message_t getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const override;

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


    //!
    //! \brief isYawDimensionSet
    //! \return
    //!
    bool isYawDimensionSet() const;

public:
    void setQuaternion(const Eigen::Quaterniond &rot) override;

    Eigen::Quaterniond getQuaternion() const override;

public:
    //!
    //! \brief setDimensionMask
    //! \param mask
    //!
    void setDimensionMask(const uint16_t &mask)
    {
        this->dimensionMask = mask;
    }

    //!
    //! \brief orDimensionMask
    //! \param orValue
    //!
    void orDimensionMask(const uint16_t &orValue)
    {
        this->dimensionMask = dimensionMask|orValue;
    }

    //!
    //! \brief getDimensionMask
    //! \return
    //!
    uint16_t getDimensionMask() const
    {
        return this->dimensionMask;
    }

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
        if(this->dimensionMask != rhs.dimensionMask)
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
        AbstractRotation::operator=(rhs);
        Eigen::Rotation2D<double>::operator=(rhs);
        this->dimensionMask = rhs.dimensionMask;
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


public:
    static const uint8_t rotationalDOF = 2;

private:
    uint16_t dimensionMask = 0;

};

} //end of namespace pose
} //end of namespace mace

#endif // ROTATION_2D_H
