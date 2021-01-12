#ifndef ABSTRACT_ALTITUDE_H
#define ABSTRACT_ALTITUDE_H

#include "../misc/coordinate_frame_components.h"
#include "abstract_position.h"
#include "altitude_interface.h"

namespace mace {
namespace pose {

MACE_CLASS_FORWARD(Abstract_Altitude);

class Abstract_Altitude : public AltitudeInterface<Abstract_Altitude>
{
public:
    Abstract_Altitude(const AltitudeReferenceTypes &explicitFrame = AltitudeReferenceTypes::REF_ALT_RELATIVE);

    Abstract_Altitude(const Abstract_Altitude &copy);

    virtual ~Abstract_Altitude() = default;

public:
    AltitudeReferenceTypes getAltitudeReferenceFrame() const;

    void setAltitudeReferenceFrame(const AltitudeReferenceTypes &explicitType);

    bool areEquivalentAltitudeFrames(const Abstract_Altitude *obj) const;

    double deltaAltitude(const Abstract_Altitude* pos) const override;


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
