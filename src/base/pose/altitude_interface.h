#ifndef ALTITUDE_INTERFACE_H
#define ALTITUDE_INTERFACE_H

#include "base_position.h"
#include "coordinate_frame.h"

namespace mace{
namespace pose{

template <typename T, class DATA>
class AltitudeInterface : public DATA
{
public:

    AltitudeInterface() = default;

    virtual ~AltitudeInterface() = default;

    template <typename NEWDATA>
    AltitudeInterface(const AltitudeInterface<T, NEWDATA> &ref):
        DATA(ref)
    {

    }

    template<typename ... Arg>
    AltitudeInterface(const Arg ... arg):
        DATA(arg ...)
    {

    }

public:

    //!
    //! \brief hasAltitudeBeenSet
    //! \return
    //!
    virtual bool hasAltitudeBeenSet() const = 0;


    //!
    //! \brief elevationFromOrigin
    //! \return
    //!
    virtual double elevationFromOrigin() const = 0;


    //!
    //! \brief deltaAltitude
    //! \param pos
    //! \return
    //!
    virtual double deltaAltitude(const T* pos) const = 0;



public:
    AltitudeInterface& operator = (const AltitudeInterface &rhs)
    {
        DATA::operator=(rhs);
        return *this;
    }

    bool operator == (const AltitudeInterface &rhs) const
    {
        if(!DATA::operator ==(rhs))
            return false;
        return true;
    }

    bool operator !=(const AltitudeInterface &rhs) const
    {
        return !(*this == rhs);
    }

};



} //end of namespace pose
} //end of namespace mace

#endif // ALTITUDE_INTERFACE_H
