#ifndef ABSTRACT_VELOCITY_H
#define ABSTRACT_VELOCITY_H

#include "base/misc/data_components.h"
#include "coordinate_frame.h"

namespace mace{
namespace pose{

template <class T, class DIM>
class AbstractVelocity
{
public:
    enum class VelocityType{
        CARTESIAN = 0,
        GEODETIC = 1,
        UNKNOWN = 2
    };

public:
    virtual ~AbstractVelocity() = default;

    AbstractVelocity(const VelocityType &desiredType, const CoordinateFrameTypes &desiredFrame)
    {
        this->type = desiredType;
        this->frame = desiredFrame;
    }

    AbstractVelocity(const AbstractVelocity &copy)
    {
        this->type = copy.type;
        this->frame = copy.frame;
        this->data = copy.data;
    }

    AbstractVelocity* getAbstractClone() const
    {
        return new AbstractVelocity(*this);
    }

    AbstractVelocity& operator = (const AbstractVelocity &copy)
    {
        this->type = copy.type;
        this->frame = copy.frame;
        this->data = copy.data;
        return *this;
    }

public:

    //!
    //! \brief getCoordinateFrame
    //! \return
    //!
    CoordinateFrameTypes getCoordinateFrame() const
    {
        return frame;
    }

    //!
    //! \brief getVelocityType
    //! \return
    //!
    VelocityType getVelocityType() const
    {
        return type;
    }

    //!
    //! \brief is3D
    //! \return
    //!
    bool is3D() const
    {
        if(mace::misc::details::PositionTypeHelper<T,DIM>::static_size > 2)
            return true;
        return false;
    }

protected:
    //!
    //! \brief setCoordinateFrame
    //! \param desiredFrame
    //!
    void setCoordinateFrame(const CoordinateFrameTypes &desiredFrame)
    {
        this->frame = desiredFrame;
    }

    //!
    //! \brief setVelocityType
    //! \param desiredType
    //!
    void setVelocityType(const VelocityType &desiredType)
    {
        this->type = desiredType;
    }

    /** Relational Operators */
public:

    //!
    //! \brief operator <
    //! \param rhs
    //! \return
    //!
    bool operator < (const AbstractVelocity &rhs) const
    {
        if(this->data >= rhs.data)
            return false;
        return true;
    }

    //!
    //! \brief operator >=
    //! \param rhs
    //! \return
    //!
    bool operator >= (const AbstractVelocity &rhs) const
    {
        return !(*this < rhs);
    }

    //!
    //! \brief operator >
    //! \param rhs
    //! \return
    //!
    bool operator > (const AbstractVelocity &rhs) const
    {
        if(this->data <= rhs.data)
            return false;
        return true;
    }

    //!
    //! \brief operator <=
    //! \param rhs
    //! \return
    //!
    bool operator <= (const AbstractVelocity &rhs) const
    {
        return !(*this > rhs);
    }

    //!
    //! \brief operator ==
    //! \param rhs
    //! \return
    //!
    bool operator == (const AbstractVelocity &rhs) const
    {
        if(this->data != rhs.data){
            return false;
        }
        if(this->frame != rhs.frame){
            return false;
        }
        if(this->type != rhs.type){
            return false;
        }
        return true;
    }

    //!
    //! \brief operator !=
    //! \param rhs
    //! \return
    //!
    bool operator != (const AbstractVelocity &rhs) const {
        return !(*this == rhs);
    }

    /** Assignment Operators */
public:

    //!
    //! \brief operator +=
    //! \param that
    //! \return
    //!
    AbstractVelocity& operator += (const AbstractVelocity &rhs)
    {
        this->data += rhs.data;
        return *this;
    }

    //!
    //! \brief operator -=
    //! \param that
    //! \return
    //!
    AbstractVelocity& operator -= (const AbstractVelocity &rhs)
    {
        this->data -= rhs.data;
        return *this;
    }

protected:
    CoordinateFrameTypes frame = CoordinateFrameTypes::CF_UNKNOWN;
    VelocityType type = VelocityType::UNKNOWN;

    DIM data;
};

} //end of namespace pose
} //end of namespace mace

#endif // ABSTRACT_VELOCITY_H
