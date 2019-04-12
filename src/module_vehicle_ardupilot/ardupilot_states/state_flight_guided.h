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

MACE_CLASS_FORWARD(GuidedTimeoutController);

class ArdupilotTimeout_Interface
{
public:
    virtual void cbiArdupilotTimeout_TargetLocal(const TargetItem::CartesianDynamicTarget &target) = 0;
    virtual void cbiArdupilotTimeout_TargetGlobal(const TargetItem::GeodeticDynamicTarget &target) = 0;
};

namespace state{

class State_FlightGuided_GoTo;
class State_FlightGuided_Idle;
class State_FlightGuided_Queue;

class State_FlightGuided : public AbstractStateArdupilot, public ArdupilotTimeout_Interface
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

    GuidedTimeoutController* guidedTimeout;

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

#include "common/thread_manager.h"

namespace ardupilot{

class GuidedTimeoutController : public Thread
{
public:
    GuidedTimeoutController(ArdupilotTimeout_Interface* callback, const unsigned int &timeout):
        currentTarget(nullptr)
    {
        this->m_CB = callback;
        this->timeout = timeout;
    }

    ~GuidedTimeoutController() {
        std::cout << "Destructor on guided timeout controller" << std::endl;
        mToExit = true;
    }

    void start() override
    {
        this->m_Timeout.start();
        Thread::start();
    }

    void run()
    {
        while(true)
        {
            if(mToExit == true) {
                clearPendingTasks();
                m_Timeout.stop();
                break;
            }

            this->RunPendingTasks();

            //The current state we can find out how much time has passed.
            //If one of the lambda expressions has fired the clock should
            //be reset right at the end, thus making this value small and
            //improbable the next function will fire
            double timeElapsed = m_Timeout.elapsedMilliseconds();

            if(timeElapsed >= timeout)
            {
                //For the moment we are removing the callback timeout
                if((currentTarget != nullptr) && (m_CB != nullptr))
                    m_CB->cbiArdupilotTimeout_TargetLocal(*currentTarget);
                m_Timeout.reset();
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(timeout/2));
        }
    }

    void updateTarget(const TargetItem::CartesianDynamicTarget &target)
    {
        m_LambdasToRun.push_back([this, target]{
            currentTarget = new TargetItem::CartesianDynamicTarget(target);
            //reset the timeout and send this new command
        });
    }

    void clearTarget()
    {
        delete currentTarget;
        currentTarget = nullptr;
    }

    void setCallbackFunction(ArdupilotTimeout_Interface* callback)
    {
        m_CB = callback;
    }

protected:
    ArdupilotTimeout_Interface* m_CB;

private:
    Timer m_Timeout;
    unsigned int timeout;

protected:
    TargetItem::CartesianDynamicTarget* currentTarget;

protected:
    std::list<std::function<void()>> m_LambdasToRun;

    void clearPendingTasks()
    {
        m_LambdasToRun.clear();
    }

    void RunPendingTasks() {
        while(m_LambdasToRun.size() > 0) {
            auto lambda = m_LambdasToRun.front();
            m_LambdasToRun.pop_front();
            lambda();
        }
    }

};

} //end of namespace ardupilot


#endif // STATE_FLIGHT_GUIDED_H
