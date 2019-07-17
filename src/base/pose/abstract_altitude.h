#ifndef ABSTRACT_ALTITUDE_H
#define ABSTRACT_ALTITUDE_H

#include "altitude_interface.h"

namespace mace {
namespace pose {

class Abstract_Altitude : public AltitudeInterface<Abstract_Altitude, misc::Data1D>
{
public:
    Abstract_Altitude(const AltitudeReferenceTypes &explicitFrame = AltitudeReferenceTypes::REF_ALT_UNKNOWN);

    Abstract_Altitude(const AltitudeReferenceTypes &explicitFrame, const double &z);

    Abstract_Altitude(const Abstract_Altitude &copy);

    ~Abstract_Altitude() override = default;

public:
    AltitudeReferenceTypes getAltitudeReferenceFrame() const;

    /** Assignment Operators */
public:
    void operator = (const Abstract_Altitude &rhs)
    {
        this->altitudeFrameType = rhs.altitudeFrameType;
    }

    /** Relational Operators */
public:
    //!
    //! \brief operator ==
    //! \param rhs
    //! \return
    //!
    bool operator == (const Abstract_Altitude &rhs) const
    {
        if(!AltitudeInterface::operator ==(rhs))
            return false;

        if(this->altitudeFrameType != rhs.altitudeFrameType){
            return false;
        }
        return true;
    }

    //!
    //! \brief operator !=
    //! \param rhs
    //! \return
    //!
    bool operator != (const Abstract_Altitude &rhs) const {
        return !(*this == rhs);
    }

protected:
    AltitudeReferenceTypes altitudeFrameType = AltitudeReferenceTypes::REF_ALT_UNKNOWN;
};

} //end of namespace pose
} //end of namespace mace


#endif // ABSTRACT_ALTITUDE_H
