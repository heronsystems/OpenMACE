#include "protocol_mavlink_mace.h"

#include <iostream>

#include <algorithm>

namespace CommsMACE
{

MavlinkProtocol::MavlinkProtocol(const MavlinkConfiguration &config) :
    m_config(config)
{
    memset(&totalReceiveCounter, 0, sizeof(totalReceiveCounter));
    memset(&totalLossCounter, 0, sizeof(totalLossCounter));
    memset(&totalErrorCounter, 0, sizeof(totalErrorCounter));
    memset(&currReceiveCounter, 0, sizeof(currReceiveCounter));
    memset(&currLossCounter, 0, sizeof(currLossCounter));

    for(int i = 0 ; i < 256 ; i++)
    {
        for(int j = 0 ; j < 256 ; j++)
        {
            lastIndex[i][j] = 0;
        }
    }
}

void MavlinkProtocol::AddListner(const IProtocolMavlinkEvents* listener)
{
    m_Listners.push_back(listener);
}

MavlinkConfiguration MavlinkProtocol::Configuration() const
{
    return m_config;
}

int MavlinkProtocol::getSystemId()
{
    return m_systemId;
}

void MavlinkProtocol::setSystemId(int id)
{
    m_systemId = id;
}

int MavlinkProtocol::getComponentId()
{
    return 0;
}


void MavlinkProtocol::ResetMetadataForLink(const ILink* link)
{
    if(m_MavlinkChannels.find(link) == m_MavlinkChannels.cend())
        return;

    int channel = m_MavlinkChannels.at(link);
    totalReceiveCounter[channel] = 0;
    totalLossCounter[channel] = 0;
    totalErrorCounter[channel] = 0;
    currReceiveCounter[channel] = 0;
    currLossCounter[channel] = 0;
}


//!
//! \brief Get the protocol channel being used for a specific link
//! \param link Link to check
//! \return Channel of the protocol being used
//!
uint8_t MavlinkProtocol::GetChannel(ILink *link) const
{
    if(m_MavlinkChannels.find(link) == m_MavlinkChannels.cend())
        throw std::runtime_error("Link does not have a channel");

    return m_MavlinkChannels.at(link);
}


//!
//! \brief Set the channel being used for a specific link on the protocol
//! \param link Link to set
//! \param channel Channel to use
//!
void MavlinkProtocol::SetChannel(ILink *link, uint8_t channel)
{
    m_MavlinkChannels[link] = channel;
}


//!
//! \brief Send message onto some link
//!
//! This code is largely a copy from MAVLinkProtocol::sendMessage in qgroundcontrol
//! \param link Link to put message onto
//! \param message Message to send
//!
void MavlinkProtocol::SendProtocolMessage(ILink *link, const mavlink_message_t &message, const OptionalParameter<Resource> &target)
{
    // Create buffer
    static uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    // Write message into buffer, prepending start sign
    int len = mavlink_msg_to_send_buffer(buffer, &message);
    // If link is connected
    if (link->isConnected())
    {
        // Send the portion of the buffer now occupied by the message
        link->WriteBytes((const char*)buffer, len, target);
    }
}

//!
//! \brief Read data incoming from some link
//!
//! This code is largely a copy from MAVLinkProtocol::receiveBytes in qgroundcontrol
//! \param link Link which data was read from
//! \param buffer data that was read.
//!
void MavlinkProtocol::ReceiveData(ILink *link, const std::vector<uint8_t> &buffer)
{

//     std::cout << "MavlinkProtocol::ReceiveData " << std::endl;
    uint8_t mavlinkChannel = m_MavlinkChannels.at(link);

    mavlink_message_t message;
    message.seq = 0;
    message.compid = 0;
    mavlink_status_t status;

    static int mavlink09Count = 0;
    static int nonmavlinkCount = 0;
    static bool decodedFirstPacket = false;
    static bool warnedUser = false;
    static bool checkedUserNonMavlink = false;
    static bool warnedUserNonMavlink = false;

    for(uint8_t c: buffer)
    {
        unsigned int decodeState = mavlink_parse_char(mavlinkChannel, c, &message, &status);

        if (c == 0x55) mavlink09Count++;
        if ((mavlink09Count > 100) && !decodedFirstPacket && !warnedUser)
        {
            warnedUser = true;

            // Obviously the user tries to use a 0.9 autopilot
            // with QGroundControl built for version 1.0
            Emit([&link](const IProtocolMavlinkEvents* ptr)
            {
                ptr->ProtocolStatusMessage(link, "MAVLINK Protocol", std::string("There is a MAVLink Version or Baud Rate Mismatch. ") +
                                                               "Your MAVLink device seems to use the deprecated version 0.9, while QGroundControl only supports version 1.0+. " +
                                                               "Please upgrade the MAVLink version of your autopilot. " +
                                                               "If your autopilot is using version 1.0, check if the baud rates of QGroundControl and your autopilot are the same.");
            });
        }

        if (decodeState == 0 && !decodedFirstPacket)
        {
            nonmavlinkCount++;
            if (nonmavlinkCount > 2000 && !warnedUserNonMavlink)
            {
                //2000 bytes with no mavlink message. Are we connected to a mavlink capable device?
                if (!checkedUserNonMavlink)
                {
                    link->RequestReset();
                    checkedUserNonMavlink = true;
                }
                else
                {
                    warnedUserNonMavlink = true;
                    Emit([&link](const IProtocolMavlinkEvents* ptr)
                    {
                        ptr->ProtocolStatusMessage(link, "MAVLINK Protocol", std::string("There is a MAVLink Version or Baud Rate Mismatch. ") +
                                                                       "Please check if the baud rates of QGroundControl and your autopilot are the same.");
                    });
                }
            }
        }
        if (decodeState == 1)
        {
            decodedFirstPacket = true;
            if(message.msgid == MAVLINK_MSG_ID_PING)
            {
                // process ping requests (tgt_system and tgt_comp must be zero)
                mavlink_ping_t ping;
                mavlink_msg_ping_decode(&message, &ping);
                if(!ping.target_system && !ping.target_component)
                {
                    mavlink_message_t msg;
                    mavlink_msg_ping_pack(getSystemId(), getComponentId(), &msg, ping.time_usec, ping.seq, message.sysid, message.compid);
                    SendProtocolMessage(link, msg);
                }
            }

            if(message.msgid == MAVLINK_MSG_ID_RADIO_STATUS)
            {
                // process telemetry status message
                mavlink_radio_status_t rstatus;
                mavlink_msg_radio_status_decode(&message, &rstatus);
                int rssi = rstatus.rssi,
                    remrssi = rstatus.remrssi;
                // 3DR Si1k radio needs rssi fields to be converted to dBm
                if (message.sysid == '3' && message.compid == 'D') {
                    /* Per the Si1K datasheet figure 23.25 and SI AN474 code
                     * samples the relationship between the RSSI register
                     * and received power is as follows:
                     *
                     *                       10
                     * inputPower = rssi * ------ 127
                     *                       19
                     *
                     * Additionally limit to the only realistic range [-120,0] dBm
                     */
                    rssi    = std::min(std::max(round(static_cast<float>(rssi)    / 1.9 - 127.0), - 120.0), 0.0);
                    remrssi = std::min(std::max(round(static_cast<float>(remrssi) / 1.9 - 127.0), - 120.0), 0.0);
                } else {
                    rssi = (int8_t) rstatus.rssi;
                    remrssi = (int8_t) rstatus.remrssi;
                }

                Emit([&](const IProtocolMavlinkEvents* ptr){ptr->RadioStatusChanged(link, rstatus.rxerrors, rstatus.fixed, rssi, remrssi, rstatus.txbuf, rstatus.noise, rstatus.remnoise);});
            }

//            if (message.msgid == MACE_MSG_ID_HEARTBEAT)
//            {
//                mace_heartbeat_t heartbeat;
//                mavlink_msg_heartbeat_decode(&message, &heartbeat);
//                Emit([&](const IProtocolMavlinkEvents* ptr){ptr->HeartbeatInfo(link, message.sysid, heartbeat);});
//            }else if(message.msgid == MAVLINK_MSG_ID_COMMAND_ACK)
//            {
//                mavlink_command_ack_t commandACK;
//                mavlink_msg_command_ack_decode(&message, &commandACK);
//                Emit([&](const IProtocolMavlinkEvents* ptr){ptr->CommandACK(link, message.sysid, commandACK);});
//            }else if(message.msgid == MACE_MSG_ID_VEHICLE_SYNC)
//            {
//                mace_vehicle_sync_t syncRequest;
//                mavlink_msg_vehicle_sync_decode(&message, &syncRequest);
//                Emit([&](const IProtocolMavlinkEvents* ptr){ptr->SyncRequest(link, message.sysid, syncRequest);});
//            }

            // Increase receive counter
            totalReceiveCounter[mavlinkChannel]++;
            currReceiveCounter[mavlinkChannel]++;

            // Determine what the next expected sequence number is, accounting for
            // never having seen a message for this system/component pair.
            int lastSeq = lastIndex[message.sysid][message.compid];
            int expectedSeq = (lastSeq == -1) ? message.seq : (lastSeq + 1);

            // And if we didn't encounter that sequence number, record the error
            if (message.seq != expectedSeq)
            {

                // Determine how many messages were skipped
                int lostMessages = message.seq - expectedSeq;

                // Out of order messages or wraparound can cause this, but we just ignore these conditions for simplicity
                if (lostMessages < 0)
                {
                    lostMessages = 0;
                }

                // And log how many were lost for all time and just this timestep
                totalLossCounter[mavlinkChannel] += lostMessages;
                currLossCounter[mavlinkChannel] += lostMessages;
            }

            // And update the last sequence number for this system/component pair
            lastIndex[message.sysid][message.compid] = expectedSeq;

            // Update on every 32th packet
            if ((totalReceiveCounter[mavlinkChannel] & 0x1F) == 0)
            {
                // Calculate new loss ratio
                // Receive loss
                float receiveLossPercent = (double)currLossCounter[mavlinkChannel]/(double)(currReceiveCounter[mavlinkChannel]+currLossCounter[mavlinkChannel]);
                receiveLossPercent *= 100.0f;
                currLossCounter[mavlinkChannel] = 0;
                currReceiveCounter[mavlinkChannel] = 0;
                Emit([&](const IProtocolMavlinkEvents* ptr){ptr->ReceiveLossPercentChanged(link, message.sysid, receiveLossPercent);});
                Emit([&](const IProtocolMavlinkEvents* ptr){ptr->ReceiveLossPercentChanged(link, message.sysid, totalLossCounter[mavlinkChannel]);});
            }

            // The packet is emitted as a whole, as it is only 255 - 261 bytes short
            // kind of inefficient, but no issue for a groundstation pc.
            // It buys as reentrancy for the whole code over all threads
            Emit([&message,&link](const IProtocolMavlinkEvents* ptr){ptr->MessageReceived(link, message);});
        }
    }
}


} //END MAVLINKComms
