#ifndef AP_STATE_TAKEOFF_H
#define AP_STATE_TAKEOFF_H

#include <mavlink.h>

#include "module_vehicle_ardupilot/flight_states/abstract_root_state.h"

#include "module_vehicle_ardupilot/ardupilot_target_progess.h"


namespace ardupilot {
namespace state{

class AP_State_Grounded;
class AP_State_TakeoffClimbing;
class AP_State_TakeoffTransitioning;
class AP_State_TakeoffComplete;
class AP_State_Flight;

class AP_State_Takeoff : public AbstractRootState
{
public:
    AP_State_Takeoff();

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

private:

};

} //end of namespace ardupilot
} //end of namespace state

#endif // AP_STATE_TAKEOFF_H
