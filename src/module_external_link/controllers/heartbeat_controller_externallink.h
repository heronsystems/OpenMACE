#ifndef HEARTBEAT_CONTROLLER_EXTERNALLINK_H
#define HEARTBEAT_CONTROLLER_EXTERNALLINK_H

#include <iostream>
#include <QDate>

#include <mavlink.h>

#include "common/thread_manager.h"
#include "data/timer.h"

#include "data/autopilot_types.h"
#include "data/comms_protocol.h"
#include "data/system_type.h"

#include "data_generic_command_item/command_item_components.h"

namespace ExternalLink {

class HeartbeatController_Interface
{
public:
    virtual void cbiHeartbeatController_transmitCommand(const mavlink_mace_heartbeat_t &heartbeat) = 0;
};


class HeartbeatController_ExternalLink : public Thread
{
public:
    HeartbeatController_ExternalLink(HeartbeatController_Interface *cb, const int &interval);

    ~HeartbeatController_ExternalLink() {
        std::cout << "Destructor on the external link heartbeat controller" << std::endl;
        mToExit = true;
    }

    void connectCallback(HeartbeatController_Interface *cb)
    {
        m_CB = cb;
    }

    void updateIDS(const int &targetID, const int &originatingID);

    void run();

private:
    int systemID;
    int transmittingID;

    Timer mTimer;
    int responseTimeout;

private:
    HeartbeatController_Interface *m_CB;

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


} //end of namespace ExternalLink

#endif // HEARTBEAT_CONTROLLER_EXTERNALLINK_H
