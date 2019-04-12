#ifndef COMMAND_CONTROLLER_MAVLINK_H
#define COMMAND_CONTROLLER_MAVLINK_H

#include <iostream>
#include <QDate>

#include "mavlink.h"

#include "data/controller_comms_state.h"
#include "common/thread_manager.h"
#include "data/timer.h"

#include "data_generic_command_item/command_item_components.h"

#include "generic/command_item.h"
#include "generic/helper_previous_command_mavlink.h"

#include "spdlog/spdlog.h"
#include "spdlog/fmt/ostr.h"

class CommandController_Interface
{
public:
    virtual void cbiCommandController_transmitCommand(const mavlink_command_int_t &cmd) = 0;
    virtual void cbiCommandController_transmitCommand(const mavlink_command_long_t &cmd) = 0;

    virtual void cbiCommandController_transmitNewMode(const mavlink_set_mode_t &mode) = 0;

    virtual void cbiCommandController_CommandACK(const mavlink_command_ack_t &ack) = 0;
};

namespace DataInterface_MAVLINK {

class CommandController_MAVLINK : public Thread
{
public:
    CommandController_MAVLINK(const int &targetID, const int &originatingID);

    ~CommandController_MAVLINK() {
        std::cout << "Destructor on the mavlink command controller" << std::endl;
        mToExit = true;
    }

    void run();

    void receivedCommandACK(const mavlink_command_ack_t &cmdACK);

    void getSystemHome(const int &compID = 0);
    void setNewMode(const int &newMode);
    void setHomePosition(const CommandItem::SpatialHome &commandItem, const int &compID = 0);
    void setSystemArm(const CommandItem::ActionArm &commandItem, const int &compID = 0);
    void setSystemTakeoff(const CommandItem::SpatialTakeoff &commandItem, const int &compID = 0);
    void setSystemLand(const CommandItem::SpatialLand &commandItem, const int &compID = 0);
    void setSystemRTL(const CommandItem::SpatialRTL &commandItem, const int &compID = 0);

    Data::ControllerCommsState getCommsState() const
    {
        return this->currentCommsState;
    }

    void connectCallback(CommandController_Interface *cb)
    {
        m_CB = cb;
    }

private:
    void logCommandACK(const mavlink_command_ack_t &cmdACK);
    void clearPreviousTransmit();
    mavlink_command_long_t initializeCommandLong();

private:
    int systemID;
    int transmittingID;

    Timer mTimer;
    bool mToExit;
    int currentRetry;
    int maxRetries;
    int responseTimeout;

private:
    std::shared_ptr<spdlog::logger> mLog;
    CommandController_Interface *m_CB;
    PreviousTransmissionBase<commandItemEnum> *prevTransmit;

    Data::ControllerCommsState currentCommsState;

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


} //end of namespace DataInterface_MAVLINK
#endif // COMMAND_CONTROLLER_MAVLINK_H
