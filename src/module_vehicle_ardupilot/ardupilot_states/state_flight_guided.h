#ifndef STATE_FLIGHT_GUIDED_H
#define STATE_FLIGHT_GUIDED_H

#include <mavlink.h>

#include <iostream>

#include "data/timer.h"

#include "abstract_state_ardupilot.h"

#include "../ardupilot_target_progess.h"
#include "../guided_timeout_controller.h"

#include "data_generic_command_item/command_item_components.h"

#include "data_generic_mission_item_topic/mission_item_reached_topic.h"


namespace ardupilot{

namespace state{

class State_FlightGuided_Idle;
class State_FlightGuided_Queue;
class State_FlightGuided_SpatialItem;
class State_FlightGuided_AttTarget;
class State_FlightGuided_CarTarget;
class State_FlightGuided_GeoTarget;

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
