#ifndef ARDUPILOT_GENERAL_CONTROLLER_H
#define ARDUPILOT_GENERAL_CONTROLLER_H

#include <chrono>
#include <thread>
#include <string>
#include <iostream>
#include <mavlink.h>

#include "data/controller_state.h"

#include "data_interface_MAVLINK/vehicle_object_mavlink.h"

#include "data_generic_state_item/state_item_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

#include "data_generic_command_item/command_item_components.h"
#include "data_generic_command_item_topic/command_item_topic_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

#include "comms/comms_marshaler.h"

#include "common/thread_manager.h"
#include "ardupilot_target_progess.h"

#include <list>

typedef void(*CallbackFunctionPtr_VehicleTarget)(void*, MissionTopic::VehicleTargetTopic&);

class Ardupilot_GeneralController : public Thread
{
public:
    enum controllerTypes
    {
        CONTROLLER_GUIDED,
        CONTROLLER_TAKEOFF
    };

public:
    Ardupilot_GeneralController(std::shared_ptr<DataInterface_MAVLINK::VehicleObject_MAVLINK> vehicleData);

    ~Ardupilot_GeneralController() {
        std::cout << "Destructor on general controller" << std::endl;
        mToExit = true;
    }

    void terminateObject();

    virtual void updateCommandACK(const mavlink_command_ack_t &cmdACK) = 0;

    void connectTargetCallback(CallbackFunctionPtr_VehicleTarget cb, void *p)
    {
        m_CBTarget = cb;
        m_FunctionTarget = p;
    }

protected:
    CallbackFunctionPtr_VehicleTarget m_CBTarget;
    void *m_FunctionTarget;

protected:
    controllerTypes controllerType;

protected:
    std::shared_ptr<DataInterface_MAVLINK::VehicleObject_MAVLINK> vehicleDataObject;

protected:
    //FLAGS for the thread:
    bool mToExit;

    //Methods for determining state of the vehicle
    ArdupilotTargetProgess vehicleMissionState;

protected:
    std::list<std::function<void()>> m_LambdasToRun;

    void RunPendingTasks() {
        while(m_LambdasToRun.size() > 0) {
            auto lambda = m_LambdasToRun.front();
            m_LambdasToRun.pop_front();
            lambda();
        }
    }

};

#endif // ARDUPILOT_GENERAL_CONTROLLER_H
