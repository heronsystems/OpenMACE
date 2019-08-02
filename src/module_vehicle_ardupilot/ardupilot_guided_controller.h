#ifndef ARDUPILOT_GUIDED_CONTROLLER_H
#define ARDUPILOT_GUIDED_CONTROLLER_H

#include <chrono>
#include <thread>
#include <string>
#include <iostream>
#include <mavlink.h>

#include "data/controller_state.h"

#include "data_interface_MAVLINK/vehicle_object_mavlink.h"

#include "ardupilot_general_controller.h"
#include "ardupilot_target_progess.h"

class Ardupilot_GuidedController : public Ardupilot_GeneralController
{
public:

    MissionItem::MissionList getDummyMissionList() const{
        return m_CurrentMission;
    }

    Ardupilot_GuidedController(std::shared_ptr<DataInterface_MAVLINK::VehicleObject_MAVLINK> vehicleData);

    ~Ardupilot_GuidedController() {
        std::cout << "Destructor on guidance controller" << std::endl;
    }

    void updatedMission(const MissionItem::MissionList &updatedMission);

    double distanceToTarget();
    void generateControl(const Data::ControllerState &currentState);
    void updateCommandACK(const mavlink_command_ack_t &cmdACK);
    void run();

private:
    //FLAGS for the thread:
    bool executionState;
    bool missionUpdated;

    MissionItem::MissionList m_CurrentMission;
    command_item::SpatialHome m_VehicleHome;

};


#endif // ARDUPILOT_GUIDED_CONTROLLER_H
