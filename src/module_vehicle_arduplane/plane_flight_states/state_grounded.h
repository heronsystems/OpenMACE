#ifndef AP_STATE_GROUNDED_H
#define AP_STATE_GROUNDED_H

#include "module_vehicle_ardupilot/flight_states/abstract_root_state.h"

namespace ardupilot {
namespace state {

class AP_State_GroundedIdle;
class AP_State_GroundedArming;
class AP_State_GroundedArmed;
class AP_State_GroundedDisarming;
class AP_State_GroundedDisarmed;

class AP_State_Takeoff;

class AP_State_Flight;

class AP_State_Grounded : public AbstractRootState
{
public:
    AP_State_Grounded();

public:
    AbstractStateArdupilot* getClone() const override;

    void getClone(AbstractStateArdupilot** state) const override;

public:
    hsm::Transition GetTransition() override;

public:
    bool handleCommand(const std::shared_ptr<AbstractCommandItem> command) override;

    void Update() override;

    void OnEnter() override;

    void OnEnter(const std::shared_ptr<AbstractCommandItem> command) override;
};

} //end of namespace ardupilot
} //end of namespace state

#endif // AP_STATE_GROUNDED_H
