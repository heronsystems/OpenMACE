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
    Data1D(const double &x);


    Data1D norm() const
    {
        double length = sqrt(x*x);
        if(fabs(length) < std::numeric_limits<double>::epsilon())
            return Data1D();
        else
            return Data1D(x/length);
    }

    /** Implied through inheritance of AbstractPoint */
public:
    virtual bool is3D() const
    {
        return false;
    }

    virtual bool is2D() const
    {
        return false;
    }

    virtual bool is1D() const
    {
        return true;
    }

    //!
    //! \brief getDataYFlag
    //! \return
    //!
    bool getDataXFlag() const
    {
        return this->dataXFlag;
    }

    /** Common among all point classes */
public:

    //!
    //! \brief setData
    //! \param data1D
    //!
    void setData(const Data1D &data1D);

    //!
    //! \brief setData
    //! \param x
    //! \param y
    //!
    void setData(const double &x);

    //!
    //! \brief setX
    //! \param posX
    //!
    void setX(const double &posX)
    {
        this->x = posX;
        this->dataXFlag = true;
    }

    //!
    //! \brief getX
    //! \return
    //!
    double getZ() const
    {
        return this->x;
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
        double newZ = this->x + that.x;
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
        double newZ = this->x - that.x;
        Data1D newPoint(newZ);
        return newPoint;
    }

    Data1D operator * (const double &value) const
    {
        Data1D newPoint(x*value);
        return newPoint;
    }

    Data1D operator / (const double &value) const
    {
        Data1D newPoint(x/value);
        return newPoint;
    }

    double dot(const Data1D &that) const
    {
        return x * that.x;
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
        if(this->x >= rhs.x)
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
        if(this->x <= rhs.x)
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
        if(fabs(this->x - rhs.x) > std::numeric_limits<double>::epsilon()){
            return false;
        }
        if(this->dataXFlag != rhs.dataXFlag){
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
        this->x = rhs.x;
        this->dataXFlag = rhs.dataXFlag;
        return *this;
    }

    //!
    //! \brief operator +=
    //! \param that
    //! \return
    //!
    Data1D& operator += (const Data1D &rhs)
    {
        this->x += rhs.x;
        return *this;
    }

    //!
    //! \brief operator -=
    //! \param that
    //! \return
    //!
    Data1D& operator -= (const Data1D &rhs)
    {
        this->x -= rhs.x;
        return *this;
    }


    /** Protected Members */
protected:
    //!
    //! \brief x
    //!
    double x = 0.0;

    //!
    //! \brief dataXFlag
    //!
    bool dataXFlag = false;

};

} //end of namespace misc
} //end of namespace mace

#endif // POINT_2D_H
