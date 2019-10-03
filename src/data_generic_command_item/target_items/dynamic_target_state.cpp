#include "dynamic_target_state.h"

namespace command_target {

//!
//! \brief DynamicTargetState
//!
DynamicTargetState::DynamicTargetState()
{

}

//!
//! \brief DynamicTargetState
//! \param target
//! \param state
//!
DynamicTargetState::DynamicTargetState(const DynamicTarget_Kinematic &target, const TARGETSTATE &state):
    DynamicTarget_Kinematic(target)
{
    this->currentState = state;
}

//!
//! \brief DynamicTargetState
//! \param copy
//!
DynamicTargetState::DynamicTargetState(const DynamicTargetState &copy):
    DynamicTarget_Kinematic(copy)
{
    this->currentState = copy.currentState;
}


//!
//! \brief setTargetState
//! \param state
//!
void DynamicTargetState::setTargetState(const TARGETSTATE &state)
{
    this->currentState = state;
}

//!
//! \brief getTargetState
//! \return
//!
DynamicTargetState::TARGETSTATE DynamicTargetState::getTargetState() const
{
    return this->currentState;
}

} //end of namespace command_target
