#ifndef ARDUPILOT_TAKEOFF_CONTROLLER_H
#define ARDUPILOT_TAKEOFF_CONTROLLER_H

#include <chrono>
#include <thread>
#include <string>
#include <iostream>
#include <mavlink.h>

#include "data/controller_state.h"

#include "data_interface_MAVLINK/vehicle_object_mavlink.h"

#include "ardupilot_general_controller.h"
#include "ardupilot_target_progess.h"


class Ardupilot_TakeoffController : public Ardupilot_GeneralController
{
public:

    Ardupilot_TakeoffController(std::shared_ptr<DataInterface_MAVLINK::VehicleObject_MAVLINK> vehicleData);

    ~Ardupilot_TakeoffController();

    void initializeTakeoffSequence(const command_item::SpatialTakeoff &takeoff);

    double distanceToTarget();
    void controlSequence();
    void generateControl(const Data::ControllerState &currentState);
    void updateCommandACK(const mavlink_command_ack_t &cmdACK);

    void run();

private:
    enum stateLogic{
        DISARMED,
        ARMED_WRONG_MODE,
        ARMED_RIGHT_MODE,
        ALTITUDE_TRANSITION,
        HORIZONTAL_TRANSITION
    };

    stateLogic currentStateLogic;

private:
    command_item::SpatialTakeoff missionItem_Takeoff;
};

#endif // ARDUPILOT_TAKEOFF_CONTROLLER_H
