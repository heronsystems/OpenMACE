#ifndef GUIDED_TIMEOUT_CONTROLLER_H
#define GUIDED_TIMEOUT_CONTROLLER_H


#include <iostream>

#include "data/timer.h"

#include "common/thread_manager.h"
#include "common/class_forward.h"

#include "ardupilot_target_progess.h"

#include "module_vehicle_MAVLINK/controllers/controller_guided_target_item_local.h"
#include "module_vehicle_MAVLINK/controllers/controller_guided_target_item_global.h"
#include "module_vehicle_MAVLINK/controllers/controller_guided_mission_item.h"

#include "data_generic_command_item/command_item_components.h"

#include "data_generic_mission_item_topic/mission_item_reached_topic.h"

namespace ardupilot_vehicle{

MACE_CLASS_FORWARD(GuidedTimeoutController);

typedef void(*CallbackFunctionPtr_DynamicTarget)(void*, command_item::Action_DynamicTarget&);

class GuidedTimeoutController : public Thread
{
public:
    GuidedTimeoutController(const unsigned int &timeout);

    ~GuidedTimeoutController() override;

    void start() override;

    void run() override;

    void registerCurrentTarget(const command_item::Action_DynamicTarget &commandTarget);

    void clearTarget();

public:
    void connectTargetCallback(CallbackFunctionPtr_DynamicTarget cb, void *p)
    {
        m_CBTarget = cb;
        m_FunctionTarget = p;
    }

    void callTargetCallback(command_item::Action_DynamicTarget &target)
    {
        if(m_CBTarget != nullptr)
            m_CBTarget(m_FunctionTarget,target);
    }

private:
    CallbackFunctionPtr_DynamicTarget m_CBTarget;
    void *m_FunctionTarget;

private:
    command_item::Action_DynamicTarget m_CurrentTarget;

    Timer m_Timeout;

    unsigned int timeout;

private:
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

} //end of namespace ardupilot_vehicle


#endif // STATE_FLIGHT_GUIDED_TIMEOUT_CONTROLLER_H
