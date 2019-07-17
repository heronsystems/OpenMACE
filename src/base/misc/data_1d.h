#ifndef DATA_1D_H
#define DATA_1D_H

#include <iostream>
#include <cmath>
#include <limits>

#include "common/class_forward.h"

namespace mace {
namespace misc {

MACE_CLASS_FORWARD(Data1D);

class Data1D
{
public:
    //!
    //! \brief Data1D
    //!
    Data1D();

    //!
    virtual ~Data1D();

    //!
    //! \brief Data1D
    //! \param copy
    //!
    Data1D(const Data1D &copy);

    //!
    //! \brief Data1D
    //! \param x
    //! \param y
    //!
    Data1D(const double &z);


    Data1D norm() const
    {
        double length = sqrt(z*z);
        if(fabs(length) < std::numeric_limits<double>::epsilon())
            return Data1D();
        else
            return Data1D(z/length);
    }

    //!
    //! \brief getDataYFlag
    //! \return
    //!
    bool getDataZFlag() const
    {
        return this->dataZFlag;
    }

    /** Common among all point classes */
public:

    //!
    //! \brief setData
    //! \param data1D
    //!
    void setData_1D(const Data1D &data1D);

    //!
    //! \brief setData
    //! \param x
    //! \param y
    //!
    void setData_1D(const double &z);

    //!
    //! \brief setX
    //! \param posX
    //!
    void setZ(const double &posX)
    {
        this->z = posX;
        this->dataZFlag = true;
    }

    //!
    //! \brief getX
    //! \return
    //!
    double getZ() const
    {
        return this->z;
    }

    /** Arithmetic Operators */
public:

    //!
    //! \brief operator +
    //! \param that
    //! \return
    //!
    Data1D operator + (const Data1D &that) const
    {
        double newZ = this->z + that.z;
        Data1D newPoint(newZ);
        return newPoint;
    }

    //!
    //! \brief operator -
    //! \param that
    //! \return
    //!
    Data1D operator - (const Data1D &that) const
    {
        double newZ = this->z - that.z;
        Data1D newPoint(newZ);
        return newPoint;
    }

    Data1D operator * (const double &value) const
    {
        Data1D newPoint(z*value);
        return newPoint;
    }

    Data1D operator / (const double &value) const
    {
        Data1D newPoint(z/value);
        return newPoint;
    }

    double dot(const Data1D &that) const
    {
        return z * that.z;
    }

    /** Relational Operators */
public:

    //!
    //! \brief operator <
    //! \param rhs
    //! \return
    //!
    bool operator < (const Data1D &rhs) const
    {
        if(this->z >= rhs.z)
            return false;
        return true;
    }

    //!
    //! \brief operator >=
    //! \param rhs
    //! \return
    //!
    bool operator >= (const Data1D &rhs) const
    {
        return !(*this < rhs);
    }

    //!
    //! \brief operator >
    //! \param rhs
    //! \return
    //!
    bool operator > (const Data1D &rhs) const
    {
        if(this->z <= rhs.z)
            return false;
        return true;
    }

    //!
    //! \brief operator <=
    //! \param rhs
    //! \return
    //!
    bool operator <= (const Data1D &rhs) const
    {
        return !(*this > rhs);
    }

    //!
    //! \brief operator ==
    //! \param rhs
    //! \return
    //!
    bool operator == (const Data1D &rhs) const
    {
        if(fabs(this->z - rhs.z) > std::numeric_limits<double>::epsilon()){
            return false;
        }
        if(this->dataZFlag != rhs.dataZFlag){
            return false;
        }
        return true;
    }

    //!
    //! \brief operator !=
    //! \param rhs
    //! \return
    //!
    bool operator != (const Data1D &rhs) const{
        return !(*this == rhs);
    }

    /** Assignment Operators */
public:

    //!
    //! \brief operator =
    //! \param rhs
    //! \return
    //!
    Data1D& operator = (const Data1D &rhs)
    {
        this->z = rhs.z;
        this->dataZFlag = rhs.dataZFlag;
        return *this;
    }

    //!
    //! \brief operator +=
    //! \param that
    //! \return
    //!
    Data1D& operator += (const Data1D &rhs)
    {
        this->z += rhs.z;
        return *this;
    }

    //!
    //! \brief operator -=
    //! \param that
    //! \return
    //!
    Data1D& operator -= (const Data1D &rhs)
    {
        this->z -= rhs.z;
        return *this;
    }


    /** Protected Members */
protected:
    //!
    //! \brief z
    //!
    double z = 0.0;

    //!
    //! \brief dataZFlag
    //!
    bool dataZFlag = false;

};

} //end of namespace misc
} //end of namespace mace

#endif // POINT_2D_H
