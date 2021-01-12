#ifndef MAVLINK_COMMS_MACE_H
#define MAVLINK_COMMS_MACE_H

#include "commsmace_global.h"

#include <memory>
#include <vector>
#include <functional>
#include <cmath>
#include <unordered_map>

#include <mavlink.h>

#include "i_link_mace.h"
#include "i_protocol_mavlink_events_mace.h"

#include "i_protocol_mace.h"

#include "mavlink_configuration_mace.h"

#include "common/optional_parameter.h"


namespace CommsMACE
{

class COMMSMACESHARED_EXPORT MavlinkProtocol : public IProtocol
{

    class DecodeEncodeLibrary
    {

    };

public:
    MavlinkProtocol(const MavlinkConfiguration &config);

    void AddListner(const IProtocolMavlinkEvents* listener);

    MavlinkConfiguration Configuration() const;


    int getSystemId();

    void setSystemId(int id);

    int getComponentId();


    virtual void ResetMetadataForLink(const ILink* link);


    //!
    //! \brief Get the protocol channel being used for a specific link
    //! \param link Link to check
    //! \return Channel of the protocol being used
    //!
    virtual uint8_t GetChannel(ILink *link) const;

    //!
    //! \brief Set the channel being used for a specific link on the protocol
    //! \param link Link to set
    //! \param channel Channel to use
    //!
    virtual void SetChannel(ILink *link, uint8_t channel);


    //!
    //! \brief Send message onto some link
    //!
    //! This code is largely a copy from MAVLinkProtocol::sendMessage in qgroundcontrol
    //! \param link Link to put message onto
    //! \param message Message to send
    //!
    void SendProtocolMessage(ILink *link, const mavlink_message_t &message, const OptionalParameter<Resource> &target = OptionalParameter<Resource>());


    //!
    //! \brief Read data incoming from some link
    //!
    //! This code is largely a copy from MAVLinkProtocol::receiveBytes in qgroundcontrol
    //! \param link Link which data was read from
    //! \param buffer data that was read.
    //!
    virtual void ReceiveData(ILink *link, const std::vector<uint8_t> &buffer);

private:

    void Emit(const std::function<void(const IProtocolMavlinkEvents*)> func)
    {
        for(const IProtocolMavlinkEvents* listener : m_Listners)
            func(listener);
    }

private:

    MavlinkConfiguration m_config;

    int m_systemId;

    int lastIndex[256][256];    ///< Store the last received sequence ID for each system/componenet pair
    int totalReceiveCounter[MAVLINK_COMM_NUM_BUFFERS];    ///< The total number of successfully received messages
    int totalLossCounter[MAVLINK_COMM_NUM_BUFFERS];       ///< Total messages lost during transmission.
    int totalErrorCounter[MAVLINK_COMM_NUM_BUFFERS];      ///< Total count of all parsing errors. Generally <= totalLossCounter.
    int currReceiveCounter[MAVLINK_COMM_NUM_BUFFERS];     ///< Received messages during this sample time window. Used for calculating loss %.
    int currLossCounter[MAVLINK_COMM_NUM_BUFFERS];        ///< Lost messages during this sample time window. Used for calculating loss %.


    std::vector<const IProtocolMavlinkEvents*> m_Listners;

    std::unordered_map<const ILink*, uint8_t> m_MavlinkChannels;


};


} //END MAVLINKComms

#endif // MAVLINK_COMMS_H
