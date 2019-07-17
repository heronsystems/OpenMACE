#ifndef ALTITUDE_INTERFACE_H
#define ALTITUDE_INTERFACE_H

#include "base_position.h"
#include "coordinate_frame.h"

namespace mace{
namespace pose{

template <typename T>
class AltitudeInterface
{
public:

    AltitudeInterface() = default;

    virtual ~AltitudeInterface() = default;

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

};



} //end of namespace pose
} //end of namespace mace

#endif // ALTITUDE_INTERFACE_H
