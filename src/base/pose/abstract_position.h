#ifndef ABSTRACT_POSITION_H
#define ABSTRACT_POSITION_H

#include "base/misc/abstract_data.h"
#include "coordinate_frame.h"

using namespace mace::misc::details;

namespace mace{
namespace pose{

template <class T, class DIM>
class AbstractPosition: public PositionTypeHelper<T,DIM>
{
public:
    enum class PositionType{
        CARTESIAN = 0,
        GEODETIC = 1,
        UNKNOWN = 2
    };

public:
    virtual ~AbstractPosition() = default;

    AbstractPosition(const PositionType &desiredType, const CoordinateFrame &desiredFrame)
    {
        this->type = desiredType;
        this->frame = desiredFrame;
    }

    AbstractPosition(const AbstractPosition &copy)
    {
        this->type = copy.type;
        this->frame = copy.frame;
        this->data = copy.data;
    }

    AbstractPosition* getAbstractClone() const
    {
        return new AbstractPosition(*this);
    }

    AbstractPosition& operator = (const AbstractPosition &copy)
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
    CoordinateFrame getCoordinateFrame() const
    {
        return frame;
    }

    //!
    //! \brief getPositionType
    //! \return
    //!
    PositionType getPositionType() const
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

    virtual bool hasBeenSet() const = 0;

    //!
    //! \brief distanceFromOrigin
    //! \return
    //!
    virtual double distanceFromOrigin() const = 0;

    //!
    //! \brief polarBearingFromOrigin
    //! \return
    //!
    virtual double polarBearingFromOrigin() const = 0;

    //!
    //! \brief elevationFromOrigin
    //! \return
    //!
    virtual double elevationFromOrigin() const
    {
        return 0.0;
    }

    //!
    //! \brief distanceBetween2D
    //! \param position
    //! \return
    //!
    virtual double distanceBetween2D(const T &position) const = 0;

    //!
    //! \brief distanceTo
    //! \param position
    //! \return
    //!
    virtual double distanceTo(const T &position) const = 0;

    //!
    //! \brief bearingTo
    //! \param position
    //! \return
    //!
    virtual double polarBearingTo(const T &position) const = 0;

    //!
    //! \brief compassBearingTo
    //! \param position
    //! \return
    //!
    virtual double compassBearingTo(const T &position) const = 0;

    //!
    //! \brief newPosition
    //! \param distance
    //! \param bearing
    //! \return
    //!
    virtual T newPositionFromPolar(const double &distance, const double &bearing) const = 0;

    //!
    //! \brief newPosition
    //! \param distance
    //! \param bearing
    //! \return
    //!
    virtual T newPositionFromCompass(const double &distance, const double &bearing) const = 0;

    virtual void applyPositionalShiftFromPolar(const double &distance, const double &bearing)  = 0;

    virtual void applyPositionalShiftFromCompass(const double &distance, const double &bearing)  = 0;


protected:
    //!
    //! \brief setCoordinateFrame
    //! \param desiredFrame
    //!
    void setCoordinateFrame(const CoordinateFrame &desiredFrame)
    {
        this->frame = desiredFrame;
    }

    //!
    //! \brief setPositionalType
    //! \param desiredType
    //!
    void setPositionalType(const PositionType &desiredType)
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
    bool operator < (const AbstractPosition &rhs) const
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
    bool operator >= (const AbstractPosition &rhs) const
    {
        return !(*this < rhs);
    }

    //!
    //! \brief operator >
    //! \param rhs
    //! \return
    //!
    bool operator > (const AbstractPosition &rhs) const
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
    bool operator <= (const AbstractPosition &rhs) const
    {
        return !(*this > rhs);
    }

    //!
    //! \brief operator ==
    //! \param rhs
    //! \return
    //!
    bool operator == (const AbstractPosition &rhs) const
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
    bool operator != (const AbstractPosition &rhs) const {
        return !(*this == rhs);
    }

    /** Assignment Operators */
public:

    //!
    //! \brief operator +=
    //! \param that
    //! \return
    //!
    AbstractPosition& operator += (const AbstractPosition &rhs)
    {
        this->data += rhs.data;
        return *this;
    }

    //!
    //! \brief operator -=
    //! \param that
    //! \return
    //!
    AbstractPosition& operator -= (const AbstractPosition &rhs)
    {
        this->data -= rhs.data;
        return *this;
    }

protected:
    CoordinateFrame frame = CoordinateFrame::CF_UNKNOWN;
    PositionType type = PositionType::UNKNOWN;

    DIM data;
};



} //end of namespace pose
} //end of namespace mace

#endif // ABSTRACT_POSITION_H
