#ifndef STATE_FLIGHT_GUIDED_H
#define STATE_FLIGHT_GUIDED_H

#include <iostream>

#include "data/timer.h"

#include "abstract_state_ardupilot.h"

#include "../ardupilot_target_progess.h"

#include "module_vehicle_MAVLINK/controllers/controller_guided_target_item_local.h"
#include "module_vehicle_MAVLINK/controllers/controller_guided_target_item_global.h"
#include "module_vehicle_MAVLINK/controllers/controller_guided_mission_item.h"

#include "data_generic_command_item/command_item_components.h"

#include "data_generic_mission_item_topic/mission_item_reached_topic.h"


namespace ardupilot{

namespace state{

class State_FlightGuided_GoTo;
class State_FlightGuided_Idle;
class State_FlightGuided_Queue;

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
    void initializeNewTargetList();

    void handleGuidedState(const CartesianPosition_3D currentPosition, const unsigned int currentTargetIndex, const Data::ControllerState &state, const double targetDistance);

    void announceTargetState(const TargetItem::CartesianDynamicTarget &target, const double &targetDistance);

private:

    TargetItem::DynamicMissionQueue* currentQueue;

    ArdupilotTargetProgess guidedProgress;

public:
    void cbiArdupilotTimeout_TargetLocal(const TargetItem::CartesianDynamicTarget &target) override
    {
        Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> *collection = Owner().ControllersCollection();
        auto ptr = static_cast<MAVLINKVehicleControllers::ControllerGuidedTargetItem_Local<MAVLINKVehicleControllers::TargetControllerStructLocal>*>(collection->At("localGuidedController"));
        if(ptr != nullptr)
        {
            MavlinkEntityKey targetID = Owner().getMAVLINKID();
            MavlinkEntityKey sender = 255;

            MAVLINKVehicleControllers::TargetControllerStructLocal action;
            action.targetID = targetID;
            action.target = target;

            ptr->Broadcast(action, sender);
        }
    }

    void cbiArdupilotTimeout_TargetGlobal(const TargetItem::GeodeticDynamicTarget &target) override
    {

        //        Controllers::ControllerCollection<mavlink_message_t> *collection = Owner().ControllersCollection();
        //        auto ptr = static_cast<MAVLINKVehicleControllers::ControllerGuidedTargetItem_Global<MAVLINKVehicleControllers::TargetControllerStructGlobal>*>(collection->At("globalGuidedController"));
        //        if(ptr != nullptr)
        //        {
        //            MaceCore::ModuleCharacteristic targetCharacter;
        //            targetCharacter.ID = Owner().getMAVLINKID();
        //            targetCharacter.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;
        //            MaceCore::ModuleCharacteristic sender;
        //            sender.ID = 255;
        //            sender.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

        //            MAVLINKVehicleControllers::TargetControllerStructGlobal action;
        //            action.targetID = targetCharacter.ID;
        //            action.target = target;

        //            ptr->Send(action, sender, targetCharacter);
        //        }
        //    }

    }
};

} //end of namespace state
} //end of namespace arudpilot


#endif // STATE_FLIGHT_GUIDED_H
