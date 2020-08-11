#ifndef STATE_FLIGHT_H
#define STATE_FLIGHT_H

#include <mavlink.h>

#include "module_vehicle_ardupilot/flight_states/abstract_root_state.h"

#include "module_vehicle_MAVLINK/controllers/commands/command_rtl.h"

namespace ardupilot{
namespace state{

class State_FlightAuto;
class State_FlightBrake;
class State_FlightGuided;
class State_FlightLand;
class State_FlightLoiter;
class State_FlightManual;
class State_FlightRTL;
class State_FlightUnknown;

class State_Landing;
class State_Grounded;

class State_Flight : public AbstractRootState
{
public:
    State_Flight();

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

    void OnExit() override;

private:
    void checkTransitionFromMode(const std::string &mode);

};

} //end of namespace ardupilot
} //end of namespace state

#endif // STATE_FLIGHT_H
