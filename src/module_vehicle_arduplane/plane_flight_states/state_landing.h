#ifndef AP_STATE_LANDING_H
#define AP_STATE_LANDING_H

#include <mavlink.h>

#include "module_vehicle_ardupilot/flight_states/abstract_root_state.h"

namespace ardupilot{
namespace state{

class AP_State_LandingTransitioning;
class AP_State_LandingDescent;
class AP_State_LandingComplete;
class AP_State_Grounded;
class AP_State_Flight;

class AP_State_Landing : public AbstractRootState
{
public:
    AP_State_Landing();

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

#endif // AP_STATE_LANDING_H
