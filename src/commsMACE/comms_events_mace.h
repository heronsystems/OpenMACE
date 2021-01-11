#ifndef COMMS_EVENTS_MACE_H
#define COMMS_EVENTS_MACE_H

#include <string>

#include <mavlink.h>
#include "common/common.h"

namespace CommsMACE
{

class CommsEvents
{
public:

    /////////////////////////////////////////////////////////
    /// Link Events
    /////////////////////////////////////////////////////////


    virtual void LinkCommunicationError(const std::string &linkName, const std::string &type, const std::string &msg) const
    {
        UNUSED(linkName);
        UNUSED(type);
        UNUSED(msg);
    }

    virtual void LinkCommunicationUpdate(const std::string &linkName, const std::string &name, const std::string &msg) const
    {
        UNUSED(linkName);
        UNUSED(name);
        UNUSED(msg);
    }

    virtual void LinkConnected(const std::string &linkName) const
    {
        UNUSED(linkName);
    }

    virtual void LinkConnectionRemoved(const std::string &linkName) const
    {
        UNUSED(linkName);
    }



    /////////////////////////////////////////////////////////
    /// Generic Protocol Events
    /////////////////////////////////////////////////////////


    virtual void ProtocolStatusMessage(const std::string &linkName, const std::string &title, const std::string &message) const
    {
        UNUSED(linkName);
        UNUSED(title);
        UNUSED(message);
    }


    virtual void ReceiveLossPercentChanged(const std::string &linkName, int uasId, float lossPercent) const
    {
        UNUSED(linkName);
        UNUSED(uasId);
        UNUSED(lossPercent);
    }

    virtual void ReceiveLossTotalChanged(const std::string &linkName, int uasId, int totalLoss) const
    {
        UNUSED(linkName);
        UNUSED(uasId);
        UNUSED(totalLoss);
    }


    /////////////////////////////////////////////////////////
    /// MAVLINK Protocol Events
    /////////////////////////////////////////////////////////


    //!
    //! \brief New Mavlink message received over a link
    //! \param linkName Name of link message received over
    //! \param msg Message received
    //!
    virtual void MACEMessage(const std::string &linkName, const mavlink_message_t &msg)
    {
        UNUSED(linkName);
        UNUSED(msg);
    }

    virtual void MACESyncMessage(const std::string &linkName, const int &systemID, const mavlink_vehicle_sync_t &syncMSG)
    {
        UNUSED(linkName);
        UNUSED(systemID);
        UNUSED(syncMSG);
    }

    virtual void MACEHeartbeatInfo(const std::string &linkName, const int &systemID, const mavlink_heartbeat_t &heartbeatMSG)
    {
        UNUSED(linkName);
        UNUSED(systemID);
        UNUSED(heartbeatMSG);
    }

    virtual void MACECommandACK(const std::string &linkName, const int &systemID, const mavlink_command_ack_t &cmdACK)
    {
        UNUSED(linkName);
        UNUSED(systemID);
        UNUSED(cmdACK);
    }


    //!
    //! \brief A new radio status packet received
    //! \param link
    //! \param rxerrors
    //! \param fixed
    //! \param rssi
    //! \param remrssi
    //! \param txbuf
    //! \param noise
    //! \param remnoise
    //!
    virtual void RadioStatusChanged(const std::string &linkName, unsigned rxerrors, unsigned fixed, int rssi, int remrssi, unsigned txbuf, unsigned noise, unsigned remnoise) const
    {
        UNUSED(linkName);
        UNUSED(rxerrors);
        UNUSED(fixed);
        UNUSED(rssi);
        UNUSED(remrssi);
        UNUSED(txbuf);
        UNUSED(noise);
        UNUSED(remnoise);
    }

};

}

#endif // COMMS_EVENTS_H
