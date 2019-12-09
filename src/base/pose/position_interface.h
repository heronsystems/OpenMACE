#ifndef POSITION_INTERFACE_H
#define POSITION_INTERFACE_H

#include "abstract_position.h"

namespace mace{
namespace pose{

template <typename T>
class PositionInterface
{
    //This class cannot possess a static assert as it is a recursive construction of the object
public:
    //!
    //! \brief distanceFromOrigin
    //! \return
    //!
    virtual double distanceFromOrigin() const = 0;

    //!
    //! \brief translationalDistanceFromOrigin
    //! \return
    //!
    virtual double translationalDistanceFromOrigin() const = 0;

    //!
    //! \brief polarBearingFromOrigin
    //! \return
    //!
    virtual double polarBearingFromOrigin() const = 0;

    //!
    //! \brief distanceBetween2D
    //! \param position
    //! \return
    //!
    virtual double distanceBetween2D(const T* position) const = 0;

    //!
    //! \brief distanceTo
    //! \param position
    //! \return
    //!
    virtual double distanceTo(const T* position) const = 0;

    //!
    //! \brief bearingTo
    //! \param position
    //! \return
    //!
    virtual double polarBearingTo(const T* position) const = 0;

    //!
    //! \brief compassBearingTo
    //! \param position
    //! \return
    //!
    virtual double compassBearingTo(const T* position) const = 0;

    //!
    //! \brief newPosition
    //! \param distance
    //! \param bearing
    //! \return
    //!
    virtual void newPositionFromPolar(T*, const double &distance, const double &bearing) const = 0;

    //!
    //! \brief newPosition
    //! \param distance
    //! \param bearing
    //! \return
    //!
    virtual void newPositionFromCompass(T*, const double &distance, const double &bearing) const = 0;

    //!
    //! \brief applyPositionalShiftFromPolar
    //! \param distance
    //! \param bearing
    //!
    virtual void applyPositionalShiftFromPolar(const double &distance, const double &bearing)  = 0;

    //!
    //! \brief applyPositionalShiftFromCompass
    //! \param distance
    //! \param bearing
    //!
    virtual void applyPositionalShiftFromCompass(const double &distance, const double &bearing)  = 0;

};



} //end of namespace pose
} //end of namespace mace

#endif // POSITION_INTERFACE_H
