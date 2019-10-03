#ifndef DYNAMIC_TARGET_STATE_H
#define DYNAMIC_TARGET_STATE_H

#include "dynamic_target_kinematic.h"

namespace command_target {

class DynamicTargetState : public DynamicTarget_Kinematic
{
public:
    enum class TARGETSTATE : uint8_t{
        COMPLETE,
        ACTIVE,
        INCOMPLETE
    };

    //!
    //! \brief DynamicTargetState
    //!
    DynamicTargetState();

    //!
    //! \brief DynamicTargetState
    //! \param target
    //! \param state
    //!
    DynamicTargetState(const DynamicTarget_Kinematic &target, const TARGETSTATE &state = TARGETSTATE::INCOMPLETE);

    //!
    //! \brief DynamicTargetState
    //! \param copy
    //!
    DynamicTargetState(const DynamicTargetState &copy);

    ~DynamicTargetState() = default;
public:

    //!
    //! \brief setTargetState
    //! \param state
    //!
    void setTargetState(const TARGETSTATE &state);

    //!
    //! \brief getTargetState
    //! \return
    //!
    TARGETSTATE getTargetState() const;

    /** Assignment Operators */
public:
    DynamicTargetState& operator = (const DynamicTargetState &rhs)
    {
        DynamicTarget_Kinematic::operator =(rhs);
        this->currentState = rhs.currentState;
        return *this;
    }

    /** Relational Operators */
public:
    //!
    //! \brief operator ==
    //! \param rhs
    //! \return
    //!
    bool operator == (const DynamicTargetState &rhs) const
    {
        if(!DynamicTarget_Kinematic::operator ==(rhs))
            return false;
        if(this->currentState != rhs.currentState)
            return false;
        return true;
    }

    //!
    //! \brief operator !=
    //! \param rhs
    //! \return
    //!
    bool operator != (const DynamicTargetState &rhs) const {
        return !(*this == rhs);
    }

protected:
    TARGETSTATE currentState;
};

} //end of namespace command_target

#endif // DYNAMIC_TARGET_STATE
