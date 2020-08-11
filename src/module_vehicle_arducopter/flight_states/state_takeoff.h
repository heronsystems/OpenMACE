#ifndef STATE_TAKEOFF_H
#define STATE_TAKEOFF_H

#include <mavlink.h>

#include "module_vehicle_ardupilot/flight_states/abstract_root_state.h"

#include "module_vehicle_ardupilot/ardupilot_target_progess.h"


namespace ardupilot {
namespace state{

class State_Grounded;
class State_TakeoffClimbing;
class State_TakeoffTransitioning;
class State_TakeoffComplete;
class State_Flight;

class State_Takeoff : public AbstractRootState
{
public:
    State_Takeoff();

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

#endif // STATE_TAKEOFF_H
