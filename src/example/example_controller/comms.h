#ifndef COMMS_H
#define COMMS_H

#include <vector>
#include <stdint.h>
#include <string>

///////////////////////////////////////////////////////////////////////////////
/// Define objects to do communication. In MACE this is MAVLINK or MAVLINK variants
///////////////////////////////////////////////////////////////////////////////


//!
//! \brief Class that identifies an entity utilizing the controller
//!
class ExampleEntity
{
public:
    uint8_t ID;

    bool operator ==(const ExampleEntity &rhs) const
    {
        return this->ID == rhs.ID;
    }
};


enum PACKET_TYPES
{
    RESOURCE_NOTIFICATION,
    RESOURCE_ACK
};

class CommPacket
{
public:
    PACKET_TYPES msgid;
    std::vector<uint8_t> data;
    ExampleEntity sender;
};


class resource_notification_t
{
public:
    uint8_t ID;
    std::string str;
};

class resource_notification_ack_t
{
public:
    uint8_t ID;
};




///////////////////////////////////////////////////////////////////////////////
/// Declare static functions to convert data into CommPacket
///   In MACE, this is handled by mavlink functions
///////////////////////////////////////////////////////////////////////////////

void resource_notification_encode(ExampleEntity entity, uint8_t chan, CommPacket* packet, const resource_notification_t *data)
{
    packet->msgid = RESOURCE_NOTIFICATION;
    packet->data = {data->ID};
    //, std::vector<uint8_t>(data.str.begin(), data.str.end())};
    packet->sender = entity;
}

void resource_notification_decode(const CommPacket *packet, resource_notification_t*data)
{
    data->ID = packet->data.at(0);
    //data = std::string(++packet.data.begin(), packet.data.end());
}


void resource_notification_ack_encode(ExampleEntity entity, uint8_t chan, CommPacket* packet, const resource_notification_ack_t *data)
{
    packet->msgid = RESOURCE_ACK;
    packet->data = {data->ID};
    //, std::vector<uint8_t>(data.str.begin(), data.str.end())};
    packet->sender = entity;
}


void resource_notification_ack_decode(const CommPacket *packet, resource_notification_ack_t*data)
{
    data->ID = packet->data.at(0);
    //data = std::string(++packet.data.begin(), packet.data.end());
}

#endif // COMMS_H
