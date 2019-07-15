#ifndef GUIDED_TIMEOUT_CONTROLLER_H
#define GUIDED_TIMEOUT_CONTROLLER_H


#include <iostream>

#include "data/timer.h"

#include "common/thread_manager.h"
#include "common/class_forward.h"

#include "../ardupilot_target_progess.h"

#include "module_vehicle_MAVLINK/controllers/controller_guided_target_item_local.h"
#include "module_vehicle_MAVLINK/controllers/controller_guided_target_item_global.h"
#include "module_vehicle_MAVLINK/controllers/controller_guided_mission_item.h"

#include "data_generic_command_item/command_item_components.h"

#include "data_generic_mission_item_topic/mission_item_reached_topic.h"


class ArdupilotTimeout_Interface
{
public:
    virtual void cbiArdupilotTimeout_TargetLocal(const TargetItem::CartesianDynamicTarget &target) = 0;
    virtual void cbiArdupilotTimeout_TargetGlobal(const TargetItem::GeodeticDynamicTarget &target) = 0;
};

MACE_CLASS_FORWARD(GuidedTimeoutController);

namespace mavlink{

class GuidedTimeoutController : public Thread
{
public:
    GuidedTimeoutController(ArdupilotTimeout_Interface* callback, const unsigned int &timeout);

    ~GuidedTimeoutController();

    void start() override;

    void run() override;

    void updateTarget(const TargetItem::CartesianDynamicTarget &target);

    void clearTarget();

    void setCallbackFunction(ArdupilotTimeout_Interface* callback);

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

} //end of namespace mavlink


#endif // STATE_FLIGHT_GUIDED_TIMEOUT_CONTROLLER_H
