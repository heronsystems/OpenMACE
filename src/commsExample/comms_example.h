#ifndef COMMSEXAMPLE_H
#define COMMSEXAMPLE_H

#include "comms_example_global.h"

#include <iostream>
#include <QMap>
#include <QThread>
#include <QSerialPort>

#include "common/common.h"

#include "comms/comms_marshaler.h"
#include "comms/i_protocol_mavlink_events.h"
#include "comms/serial_configuration.h"

#include "comms/serial_link.h"
#include "comms/udp_link.h"
#include "comms/protocol_mavlink.h"

#include "mace_core/module_factory.h"

class COMMSEXAMPLESHARED_EXPORT CommsExample :
        public Comms::CommsEvents
{

public:
    CommsExample();

    virtual ~CommsExample();

    virtual void ConfigureExampleStructure(MaceCore::ModuleParameterStructure &structure) const;

    virtual void ConfigureComms(const std::shared_ptr<MaceCore::ModuleParameterValue> &params);

    virtual void Shutdown();

    // EXAMPLE: This would be replaced with whatever protocol is being used
    virtual bool MavlinkMessage(const std::string &linkName, const mavlink_message_t &message);

    // EXAMPLE: This would be replaced with whatever heartbeat message is used
    virtual void VehicleHeartbeatInfo(const std::string &linkName, const int &systemID, const mavlink_heartbeat_t &heartbeatMSG);

    // EXAMPLE: This would be replaced with whatever protocol is used
    void TransmitMAVLINKMessage(const mavlink_message_t &msg);

    uint8_t getLinkChannel() const;

    std::string getLinkName() const;

protected:
    std::string m_LinkName;
    uint8_t m_LinkChan;

private:
    std::unordered_map<Comms::Protocols, std::shared_ptr<Comms::ProtocolConfiguration>, EnumClassHash> m_AvailableProtocols;

};

#endif // COMMSEXAMPLE_H
