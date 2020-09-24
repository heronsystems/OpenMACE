#ifndef AP_STATE_UNKNOWN_H
#define AP_STATE_UNKNOWN_H


#include "module_vehicle_ardupilot/flight_states/abstract_state_ardupilot.h"

namespace ardupilot {
namespace state{

class AP_State_Grounded;
class AP_State_Takeoff;
class AP_State_Flight;
class AP_State_Landing;

class AP_State_Unknown : public AbstractStateArdupilot
{
public:
    AP_State_Unknown();

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


#endif // AP_STATE_UNKNOWN_H
