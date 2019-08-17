#ifndef STATE_FLIGHT_GUIDED_H
#define STATE_FLIGHT_GUIDED_H

#include <iostream>

#include "data/timer.h"

#include "abstract_state_ardupilot.h"

#include "../ardupilot_target_progess.h"
#include "../guided_timeout_controller.h"

#include "module_vehicle_MAVLINK/controllers/controller_guided_target_item_local.h"
#include "module_vehicle_MAVLINK/controllers/controller_guided_target_item_global.h"
#include "module_vehicle_MAVLINK/controllers/controller_guided_mission_item.h"

#include "data_generic_command_item/command_item_components.h"

#include "data_generic_mission_item_topic/mission_item_reached_topic.h"


namespace ardupilot{

namespace state{

class State_FlightGuided_Idle;
class State_FlightGuided_Queue;
class State_FlightGuided_Target;
class State_FlightGuided_MissionItem;

class State_FlightGuided : public AbstractStateArdupilot
{
public:
    State_FlightGuided();

    void OnExit() override;

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

} //end of namespace state
} //end of namespace arudpilot


#endif // STATE_FLIGHT_GUIDED_H
