#ifndef ALTITUDE_INTERFACE_H
#define ALTITUDE_INTERFACE_H

#include "common/common.h"
#include "common/class_forward.h"

namespace mace{
namespace pose{

template <typename T>
class AltitudeInterface
{
public:
    //!
    //! \brief setAltitude
    //! \param altitude
    //!
    virtual void setAltitude(const double &altitude) = 0;

    //!
    //! \brief getAltitude
    //! \return
    //!
    virtual double getAltitude() const = 0;

    //!
    //! \brief deltaAltitude
    //! \param pos
    //! \return
    //!
    virtual double deltaAltitude(const T* pos) const = 0;

    //!
    //! \brief elevationAngleFromOrigin
    //! \return
    //!
    virtual double elevationAngleFromOrigin() const
    {
        return 0.0;
    }

};



} //end of namespace pose
} //end of namespace mace

#endif // ALTITUDE_INTERFACE_H
