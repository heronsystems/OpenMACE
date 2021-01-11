#ifndef COMMS_MACE_HELPER_H
#define COMMS_MACE_HELPER_H

#include "commsmacehelper_global.h"

#include <iostream>
#include <QMap>
#include <QThread>
#include <QSerialPort>

#include "common/common.h"

#include "commsMACE/comms_marshaler_mace.h"
#include "commsMACE/i_protocol_mavlink_events_mace.h"
#include "commsMACE/serial_configuration_mace.h"

#include "commsMACE/serial_link_mace.h"
#include "commsMACE/udp_link_mace.h"
#include "commsMACE/protocol_mavlink_mace.h"
#include "commsMACE/ethernet_link_mace.h"

#include "mace_core/module_factory.h"

class COMMSMACEHELPERSHARED_EXPORT CommsMACEHelper :
        public CommsMACE::CommsEvents
{

public:
    CommsMACEHelper();

    virtual ~CommsMACEHelper();

    virtual void ConfigureMACEStructure(MaceCore::ModuleParameterStructure &structure) const;

    virtual void ConfigureMACEComms(const std::shared_ptr<MaceCore::ModuleParameterValue> &params);

    virtual void MACEMessage(const std::string &linkName, const mavlink_message_t &message);

    virtual std::unordered_map<std::string, MaceCore::TopicCharacteristic> GetTopics()
    {
        //return IModuleCommandVehicle::GetTopics();
        return {};
    }

protected:
    CommsMACE::CommsMarshaler *m_LinkMarshaler;
    std::string m_LinkName;
    uint8_t m_LinkChan;

private:
    std::unordered_map<CommsMACE::Protocols, std::shared_ptr<CommsMACE::ProtocolConfiguration>, EnumClassHash> m_AvailableProtocols;

};

#endif // COMMS_MACE_HELPER_H
