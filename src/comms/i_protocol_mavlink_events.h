#ifndef I_MAVLINK_COMMS_EVENTS_H
#define I_MAVLINK_COMMS_EVENTS_H

#include <string>
#include <memory>

#include "mavlink.h"

#include "i_link.h"

#include "i_protocol_events.h"


namespace Comms
{

//!
//! \brief Interface that it to be implemented by users of MavlinkComms to listen for any events it fired
//!
class IProtocolMavlinkEvents : public IProtocolEvents
{
public:

    //!
    //! \brief A Message has been received over Mavlink protocol
    //! \param linkName Link identifier which generated command
    //! \param message Message that has been received
    //!
    virtual void MessageReceived(const ILink* link_ptr, const mavlink_message_t &message) const = 0;


    //!
    //! \brief Heartbeat of vehicle received
    //! \param link
    //! \param vehicleId
    //! \param vehicleMavlinkVersion
    //! \param vehicleFirmwareType
    //! \param vehicleType
    //!
    virtual void VehicleHeartbeatInfo(const ILink* link_ptr, const int &vehicleId, const mavlink_heartbeat_t &heartbeatMSG) const = 0;


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
    virtual void RadioStatusChanged(const ILink* link_ptr, unsigned rxerrors, unsigned fixed, int rssi, int remrssi, unsigned txbuf, unsigned noise, unsigned remnoise) const = 0;
};


} //END MAVLINKComms

#endif // I_MAVLINK_COMMS_EVENTS_H
