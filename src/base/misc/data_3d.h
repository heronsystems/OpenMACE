#ifndef DATA_3D_H
#define DATA_3D_H

#include "data_2d.h"

namespace mace {
namespace misc {

MACE_CLASS_FORWARD(Data3D);

class Data3D : public Data2D
{

public:
    //!
    //! \brief Data3D
    //!
    Data3D();

    virtual ~Data3D();

    //!
    //! \brief Data3D
    //! \param copy
    //!
    Data3D(const Data3D &copy);

    //!
    //! \brief Data3D
    //! \param copy
    //!
    Data3D(const Data2D &copy);

    //!
    //! \brief Data3D
    //! \param copy
    //!
    Data3D(const Data2D &copy, const double &z);

    //!
    //! \brief Data3D
    //! \param compX
    //! \param compY
    //! \param compZ
    //!
    Data3D(const double &compX, const double &compY, const double &compZ);


    //!
    //! \brief get2DData
    //! \return
    //!
    Data2D get2DData() const;

private:
    double vectorLength() const
    {
        return sqrt(x*x + y*y + z*z);
    }

public:
    Data3D norm() const
    {
        double length = vectorLength();
        if(fabs(length) < std::numeric_limits<double>::epsilon())
            return Data3D();
        else
            return Data3D(x / length, y / length, z / length);
    }

    double L2Norm() const
    {
        return vectorLength();
    }

    /** Common functions among all Point3D objects */
public:

    void setData_2D(const double &compX, const double &compY, const double &compZ)
    {
        this->setX(compX);
        this->setY(compY);
        this->setZ(compZ);
    }

    //!
    //! \brief setZ
    //! \param posZ
    //!
    void setZ(const double &dataZ)
    {
        this->z = dataZ;
        this->dataZFlag = true;
    }

    //!
    //! \brief getZ
    //! \return
    //!
    double getZ() const
    {
        return this->z;
    }

    //!
    //! \brief getDataYFlag
    //! \return
    //!
    bool getDataZFlag() const
    {
        return this->dataZFlag;
    }

    /** Arithmetic Operators */
public:

    //!
    //! \brief operator +
    //! \param that
    //! \return
    //!
    Data3D operator + (const Data3D &that) const
    {
        Data2D new2D = Data2D::operator +(that);
        double newZ = this->z + that.z;
        Data3D newPoint(new2D, newZ);
        return newPoint;
    }

    friend Data3D operator + (const Data2D &lhs, const Data3D &rhs);

    //!
    //! \brief operator -
    //! \param that
    //! \return
    //!
    Data3D operator - (const Data3D &that) const
    {
        Data2D new2D = Data2D::operator -(that);
        double newZ = this->z - that.z;
        Data3D newPoint(new2D, newZ);
        return newPoint;
    }

    friend Data3D operator - (const Data2D &lhs, const Data3D &rhs);

    Data3D operator * (const double &value) const
    {
        Data3D newPoint(x*value, y*value, z * value);
        return newPoint;
    }

    Data3D operator / (const double &value) const
    {
        Data3D newPoint(x/value, y/value, z/value);
        return newPoint;
    }

    double dot(const Data3D &that) const
    {
        return x * that.x + y * that.y + z * that.z;
    }


    /** Relational Operators */
public:

    //!
    //! \brief operator <
    //! \param rhs
    //! \return
    //!
    bool operator < (const Data3D &rhs) const
    {
        if(!Data2D::operator <(rhs))
            return false;
        if(this->z >= rhs.z)
            return false;
        return true;
    }

    friend bool operator < (const Data3D &lhs, const Data2D &rhs);


    //!
    //! \brief operator >=
    //! \param rhs
    //! \return
    //!
    bool operator >= (const Data3D &rhs) const
    {
        return !(*this < rhs);
    }

    friend bool operator >= (const Data3D &lhs, const Data2D &rhs);


    //!
    //! \brief operator >
    //! \param rhs
    //! \return
    //!
    bool operator > (const Data3D &rhs) const
    {
        if(!Data2D::operator >(rhs))
            return false;
        if(this->z <= rhs.z)
            return false;
        return true;
    }

    friend bool operator > (const Data3D &lhs, const Data2D &rhs);

    //!
    //! \brief operator <=
    //! \param rhs
    //! \return
    //!
    bool operator <= (const Data3D &rhs) const
    {
        return !(*this > rhs);
    }

    friend bool operator <= (const Data3D &lhs, const Data2D &rhs);

    //!
    //! \brief operator ==
    //! \param rhs
    //! \return
    //!
    bool operator == (const Data3D &rhs) const
    {
        if(!Data2D::operator ==(rhs))
            return false;
        if(fabs(this->z - rhs.z) > std::numeric_limits<double>::epsilon()){
            return false;
        }
        if(this->dataZFlag != rhs.dataZFlag){
            return false;
        }
        return true;
    }

    friend bool operator == (const Data3D &lhs, const Data2D &rhs);

    //!
    //! \brief operator !=
    //! \param rhs
    //! \return
    //!
    bool operator != (const Data3D &rhs) const {
        return !(*this == rhs);
    }

    friend bool operator != (const Data3D &lhs, const Data2D &rhs);

    /** Assignment Operators */
public:

    //!
    //! \brief operator =
    //! \param rhs
    //! \return
    //!
    Data3D& operator = (const Data3D &rhs)
    {
        Data2D::operator =(rhs);
        this->z = rhs.z;
        this->dataZFlag = rhs.dataZFlag;
        return *this;
    }

    //!
    //! \brief operator +=
    //! \param that
    //! \return
    //!
    Data3D& operator += (const Data3D &rhs)
    {
        Data2D::operator +=(rhs);
        this->z += rhs.z;
        return *this;
    }

    //!
    //! \brief operator -=
    //! \param that
    //! \return
    //!
    Data3D& operator -= (const Data3D &rhs)
    {
        Data2D::operator -=(rhs);
        this->z -= rhs.z;
        return *this;
    }

    /** Protected Members */
protected:
    //!
    //! \brief x
    //!
    double z = 0.0;

    //!
    //! \brief posYFlag
    //!
    bool dataZFlag = 0.0;

};


} //end of namespace misc
} //end of namespace base

#endif // DATA_3D_H
