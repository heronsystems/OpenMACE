#ifndef AP_STATE_FLIGHT_GUIDED_H
#define AP_STATE_FLIGHT_GUIDED_H

#include <mavlink.h>

#include <iostream>

#include "data/timer.h"


#include "module_vehicle_ardupilot/flight_states/abstract_state_ardupilot.h"

#include "module_vehicle_ardupilot/ardupilot_target_progess.h"

#include "data_generic_command_item/command_item_components.h"

#include "data_generic_mission_item_topic/mission_item_reached_topic.h"


namespace ardupilot {

namespace state{

class AP_State_FlightGuided_Idle;
class AP_State_FlightGuided_Queue;
class AP_State_FlightGuided_SpatialItem;
class AP_State_FlightGuided_AttTarget;
class AP_State_FlightGuided_CarTarget;
class AP_State_FlightGuided_GeoTarget;

class AP_State_FlightGuided : public AbstractStateArdupilot
{
public:
    AP_State_FlightGuided();

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
} //end of namespace arudcopter


#endif // AP_STATE_FLIGHT_GUIDED_H
