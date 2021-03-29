#ifndef BASE_3D_POSITION_H
#define BASE_3D_POSITION_H

#include "base_2d_position.h"
#include "iomanip"

namespace DataState {

class Base3DPosition : public Base2DPosition
{

public:
    //!
    //! \brief Base3DPosition
    //!
    Base3DPosition():
        Base2DPosition()
    {
        this->z = 0.0;
        this->posZFlag =false;
    }

    Base3DPosition(const Base3DPosition &copy):
        Base2DPosition(copy)
    {
        this->z = copy.z;
        this->posZFlag = copy.posZFlag;
    }

    //!
    //! \brief Base3DPosition
    //! \param coordinateFrame
    //!
    Base3DPosition(const Data::CoordinateFrameType &coordinateFrame) :
        Base2DPosition(coordinateFrame)
    {
        this->z = 0.0;
        this->posZFlag =false;
    }

    //!
    //! \brief StateGenericPosition
    //! \param posX
    //! \param posY
    //! \param posZ
    //!
    Base3DPosition(const double &posX, const double &posY, const double &posZ):
        Base2DPosition(posX, posY)
    {
        this->setPosition3D(posX,posY,posZ);
    }

    //!
    //! \brief StateGenericPosition
    //! \param coordinateFrame
    //! \param posX
    //! \param posY
    //! \param posZ
    //!
    Base3DPosition(const Data::CoordinateFrameType &coordinateFrame, const double &posX, const double &posY, const double &posZ):
        Base2DPosition(coordinateFrame,posX,posY)
    {
        this->setPosition3D(posX,posY,posZ);
    }

public:

    //!
    //! \brief setPosition3D
    //! \param posX
    //! \param posY
    //! \param posZ
    //!
    void setPosition3D(const double &posX, const double &posY, const double &posZ)
    {
        this->setX(posX);
        this->setY(posY);
        this->setZ(posZ);
    }

    //!
    //! \brief setZ
    //! \param posZ
    //!
    void setZ(const double &posZ)
    {
        this->z = posZ;
        this->posZFlag = true;
    }

    //!
    //! \brief getZ
    //! \return
    //!
    double getZ() const
    {
        return this->z;
    }

public:
    //!
    //! \brief operator =
    //! \param rhs
    //!
    Base3DPosition& operator = (const Base3DPosition &rhs)
    {
        Base2DPosition::operator =(rhs);
        this->z = rhs.z;
        this->posZFlag = rhs.posZFlag;
        return *this;
    }

    //!
    //! \brief operator ==
    //! \param rhs
    //! \return
    //!
    bool operator == (const Base3DPosition &rhs) {

        if(!Base2DPosition::operator ==(rhs)){
            return false;
        }
        if(this->z != rhs.z){
            return false;
        }
        if(this->posZFlag != rhs.posZFlag){
            return false;
        }
        return true;
    }

    //!
    //! \brief operator !=
    //! \param rhs
    //! \return
    //!
    bool operator != (const Base3DPosition &rhs) {
        return !(*this == rhs);
    }

public:
    //!
    //! \brief getPosZFlag
    //! \return
    //!
    bool getPosZFlag() const
    {
        return this->posZFlag;
    }

    //!
    //! \brief has3DPositionSet
    //! \return
    //!
    bool has3DPositionSet() const
    {
        return this->posXFlag && this->posYFlag && this->posZFlag;
    }

protected:
    //!
    //! \brief z
    //!
    double z;

    //!
    //! \brief posZFlag
    //!
    bool posZFlag;
};

}

#endif // BASE_3D_POSITION_H
