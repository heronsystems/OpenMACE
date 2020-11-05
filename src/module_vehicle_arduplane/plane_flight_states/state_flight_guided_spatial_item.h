#ifndef AP_STATE_FLIGHT_GUIDED_SPATIAL_ITEM_H
#define AP_STATE_FLIGHT_GUIDED_SPATIAL_ITEM_H

#include <iostream>
#include <mavlink.h>

#include "data/timer.h"


#include "module_vehicle_ardupilot/flight_states/abstract_state_ardupilot.h"

#include "module_vehicle_ardupilot/ardupilot_target_progess.h"

#include "module_vehicle_MAVLINK/controllers/controller_guided_target_item_local.h"
#include "module_vehicle_MAVLINK/controllers/controller_guided_target_item_global.h"
#include "module_vehicle_MAVLINK/controllers/controller_guided_mission_item.h"

#include "data_generic_command_item/command_item_components.h"

#include "data_generic_mission_item_topic/mission_item_reached_topic.h"


namespace ardupilot {
namespace state{

class AP_State_FlightGuided_Idle;

class AP_State_FlightGuided_SpatialItem : public AbstractStateArdupilot
{
public:
    AP_State_FlightGuided_SpatialItem();

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
    void processSpatialWaypoint();

private:
    bool commandAccepted = false;
    ArdupilotTargetProgess guidedProgress;
};

} //end of namespace state
} //end of namespace arudcopter

#endif // AP_STATE_FLIGHT_GUIDED_SPATIAL_ITEM_H
