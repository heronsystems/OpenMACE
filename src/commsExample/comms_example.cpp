#include "comms_example.h"

// EXAMPLE: To see an example of how this class should be implemented, see commsMAVLINK
//              - Specific MultiWii implementation can be found here: https://github.com/christianrauch/msp
//              - MAVLINK can be replaced with any protocol, e.g. MultiWii

Comms::CommsMarshaler g_CommsMarshaler;

CommsExample::CommsExample() :
    m_LinkName(""),
    m_LinkChan(0)
{
}

CommsExample::~CommsExample()
{
}

uint8_t CommsExample::getLinkChannel() const
{
    return this->m_LinkChan;
}

std::string CommsExample::getLinkName() const
{
    return this->m_LinkName;
}

// EXAMPLE: This would be replaced with whatever protocol is being used
void CommsExample::TransmitMAVLINKMessage(const mavlink_message_t &msg)
{
    g_CommsMarshaler.SendMAVMessage(m_LinkName, msg);
}

// EXAMPLE: This would be replaced with whatever protocol is being used
void CommsExample::VehicleHeartbeatInfo(const std::string &linkName, const int &systemID, const mavlink_heartbeat_t &heartbeatMSG)
{
    UNUSED(linkName);
    UNUSED(systemID);
    UNUSED(heartbeatMSG);
}

// EXAMPLE: This would be replaced with whatever protocol is being used
bool CommsExample::MavlinkMessage(const std::string &linkName, const mavlink_message_t &message)
{
    UNUSED(linkName);
    UNUSED(message);
    return false;
}

// EXAMPLE: This would be replaced with whatever protocol is being used
void CommsExample::ConfigureExampleStructure(MaceCore::ModuleParameterStructure &structure) const
{
    UNUSED(structure);
}


//!
//! \brief Provides object contains parameters values to configure module with
//! \param params Parameters to configure
//!
void CommsExample::ConfigureComms(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
{
    UNUSED(params);
}


void CommsExample::Shutdown()
{
    g_CommsMarshaler.DisconnectFromLink(m_LinkName);
}
