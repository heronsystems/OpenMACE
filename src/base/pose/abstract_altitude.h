#ifndef ABSTRACT_ALTITUDE_H
#define ABSTRACT_ALTITUDE_H

#include "altitude_interface.h"

namespace mace {
namespace pose {

MACE_CLASS_FORWARD(Abstract_Altitude);

class Abstract_Altitude : protected AltitudeInterface<Abstract_Altitude>, protected misc::Data1D
{
public:
    Abstract_Altitude(const AltitudeReferenceTypes &explicitFrame = AltitudeReferenceTypes::REF_ALT_UNKNOWN);

    Abstract_Altitude(const AltitudeReferenceTypes &explicitFrame, const double &z);

    Abstract_Altitude(const Abstract_Altitude &copy);

    ~Abstract_Altitude() override = default;

public:
    void setAltitude(const double &altitude);

    double getAltitude() const;

    double deltaAltitude(const Abstract_Altitude* pos) const override;

    bool hasAltitudeBeenSet() const override;

    virtual double elevationFromOrigin() const override;

public:
    AltitudeReferenceTypes getAltitudeReferenceFrame() const;

    void setAltitudeReferenceFrame(const AltitudeReferenceTypes &explicitType);

    bool areEquivalentAltitudeFrames(const Abstract_Altitude &obj) const;

    /** Assignment Operators */
public:
    void operator = (const Abstract_Altitude &rhs)
    {
        Data1D::operator=(rhs);
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
        if(!Data1D::operator ==(rhs))
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
