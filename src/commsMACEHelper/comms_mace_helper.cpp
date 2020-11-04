#include "comms_mace_helper.h"

CommsMACEHelper::CommsMACEHelper() :
    m_LinkMarshaler(new CommsMACE::CommsMarshaler), m_LinkName(""), m_LinkChan(0)
{
    m_LinkMarshaler->AddSubscriber(this);
}

CommsMACEHelper::~CommsMACEHelper()
{

}

void CommsMACEHelper::MACEMessage(const std::string &linkName, const mace_message_t &message)
{
    UNUSED(linkName);
    UNUSED(message);
    std::cout<<"I am in the comms_mavlink library MavlinkMessage callback."<<std::endl;
}

//!
//! \brief Describes the strucure of the parameters for this module
//! \return Strucure
//!
void CommsMACEHelper::ConfigureMACEStructure(MaceCore::ModuleParameterStructure &structure) const
{
    std::shared_ptr<MaceCore::ModuleParameterStructure> serialSettings = std::make_shared<MaceCore::ModuleParameterStructure>();
    serialSettings->AddTerminalParameters("PortName", MaceCore::ModuleParameterTerminalTypes::STRING, true);
    serialSettings->AddTerminalParameters("BaudRate", MaceCore::ModuleParameterTerminalTypes::INT, true);
    serialSettings->AddTerminalParameters("DataBits", MaceCore::ModuleParameterTerminalTypes::INT, true);
    serialSettings->AddTerminalParameters("StopBits", MaceCore::ModuleParameterTerminalTypes::INT, true);
    serialSettings->AddTerminalParameters("Parity", MaceCore::ModuleParameterTerminalTypes::BOOLEAN, true);
    serialSettings->AddTerminalParameters("FlowControl", MaceCore::ModuleParameterTerminalTypes::INT, true);
    //        structure.AddNonTerminal("SerialParameters", serialSettings, true);

    std::shared_ptr<MaceCore::ModuleParameterStructure> udpSettings = std::make_shared<MaceCore::ModuleParameterStructure>();
    udpSettings->AddTerminalParameters("ListenAddress", MaceCore::ModuleParameterTerminalTypes::STRING, true);
    udpSettings->AddTerminalParameters("ListenPortNumber", MaceCore::ModuleParameterTerminalTypes::INT, true);

    std::shared_ptr<MaceCore::ModuleParameterStructure> tcpSettings = std::make_shared<MaceCore::ModuleParameterStructure>();
    tcpSettings->AddTerminalParameters("TCPServerAddress", MaceCore::ModuleParameterTerminalTypes::STRING, true);
    tcpSettings->AddTerminalParameters("TCPServerPort", MaceCore::ModuleParameterTerminalTypes::INT, true);
    tcpSettings->AddTerminalParameters("UDPBroadcastPort", MaceCore::ModuleParameterTerminalTypes::INT, true);

    std::shared_ptr<MaceCore::ModuleParameterStructure> ethernetSettings = std::make_shared<MaceCore::ModuleParameterStructure>();
    ethernetSettings->AddTerminalParameters("PortNumber",MaceCore::ModuleParameterTerminalTypes::INT, true);

    std::shared_ptr<MaceCore::ModuleParameterStructure> digiMeshSettings = std::make_shared<MaceCore::ModuleParameterStructure>();
    digiMeshSettings->AddTerminalParameters("PortName", MaceCore::ModuleParameterTerminalTypes::STRING, true);
    digiMeshSettings->AddTerminalParameters("BaudRate", MaceCore::ModuleParameterTerminalTypes::INT, true);

    std::shared_ptr<MaceCore::ModuleParameterStructure> protocolSettings = std::make_shared<MaceCore::ModuleParameterStructure>();
    protocolSettings->AddTerminalParameters("Name", MaceCore::ModuleParameterTerminalTypes::STRING, true, "Mavlink", {"Mavlink"});
    protocolSettings->AddTerminalParameters("Version", MaceCore::ModuleParameterTerminalTypes::STRING, true, "V1", {"V1", "V2"});

    structure.AddMutuallyExclusiveNonTerminal({{"SerialParameters", serialSettings}, {"UDPParameters", udpSettings}, {"DigiMeshParameters", digiMeshSettings}, {"TCPParameters", tcpSettings}, {"EthernetParameters", ethernetSettings}}, true);
    structure.AddNonTerminal("ProtocolParameters", protocolSettings, true);
}


//!
//! \brief Provides object contains parameters values to configure module with
//! \param params Parameters to configure
//!
void CommsMACEHelper::ConfigureMACEComms(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
{
    std::shared_ptr<CommsMACE::ProtocolConfiguration> protocolConfig;
    if(params->HasNonTerminal("ProtocolParameters"))
    {
        std::shared_ptr<MaceCore::ModuleParameterValue> protocolSettings = params->GetNonTerminalValue("ProtocolParameters");
        std::string protocolName = protocolSettings->GetTerminalValue<std::string>("Name");
        std::string versionName = protocolSettings->GetTerminalValue<std::string>("Version");


        if(protocolName == "Mavlink")
        {
            std::shared_ptr<CommsMACE::MavlinkConfiguration> mavlinkConfig = std::make_shared<CommsMACE::MavlinkConfiguration>();

            if(versionName == "V1")
            {
                mavlinkConfig->SetVersion(CommsMACE::MavlinkConfiguration::MavlinkVersion::MavlinkVersionAlways1);
            }
            else if(versionName == "V2")
            {
                mavlinkConfig->SetVersion(CommsMACE::MavlinkConfiguration::MavlinkVersion::MavlinkVersionAlways2);
            }
            else
            {
                throw std::runtime_error("Unknown mavlink version seen");
            }

            m_LinkMarshaler->AddProtocol(*mavlinkConfig);

            m_AvailableProtocols.insert({CommsMACE::Protocols::MAVLINK, std::static_pointer_cast<CommsMACE::ProtocolConfiguration>(mavlinkConfig)});
            protocolConfig = mavlinkConfig;
        }
        else
        {
            throw std::runtime_error("Unknown Protocol encountered");
        }

    }
    if(params->HasNonTerminal("SerialParameters"))
    {
        std::shared_ptr<MaceCore::ModuleParameterValue> serialSettings = params->GetNonTerminalValue("SerialParameters");


        std::string portName = serialSettings->GetTerminalValue<std::string>("PortName");
        int baudRate = serialSettings->GetTerminalValue<int>("BaudRate");
        int dataBits = serialSettings->GetTerminalValue<int>("DataBits");
        int stopBits = serialSettings->GetTerminalValue<int>("StopBits");

        bool parity = serialSettings->GetTerminalValue<bool>("Parity");
        int flowControl = serialSettings->GetTerminalValue<int>("FlowControl");


        CommsMACE::Protocols protocolToUse = CommsMACE::Protocols::MAVLINK;

        CommsMACE::SerialConfiguration config("config");
        config.setPortName(portName);
        config.setBaud(baudRate);
        config.setDataBits(dataBits);
        config.setStopBits(stopBits);
        config.setParity(parity);
        config.setFlowControl(flowControl);

        m_LinkName = "link_" + portName;
        m_LinkMarshaler->AddLink(m_LinkName, config);


        //now configure to use link with desired protocol
        if(protocolToUse == CommsMACE::Protocols::MAVLINK)
        {
            m_LinkMarshaler->SetProtocolForLink(m_LinkName, CommsMACE::Protocols::MAVLINK);

            std::shared_ptr<CommsMACE::MavlinkConfiguration> mavlinkConfig = std::static_pointer_cast<CommsMACE::MavlinkConfiguration>(m_AvailableProtocols.at(CommsMACE::Protocols::MAVLINK));

            //set version on mavlink channel
            // I would prefer to put this in CommsMACE library, but because the mavlinkstatus is static variable, things get messed up when linking
            m_LinkChan = m_LinkMarshaler->GetProtocolChannel(m_LinkName);
            mace_status_t* maceStatus = mace_get_channel_status(m_LinkChan);
            std::cout << maceStatus << std::endl;
            switch (mavlinkConfig->GetVersion()) {
            case CommsMACE::MavlinkConfiguration::MavlinkVersion::MavlinkVersion2IfVehicle2:
                if (maceStatus->flags & MACE_STATUS_FLAG_IN_MACE1) {
                    maceStatus->flags |= MACE_STATUS_FLAG_OUT_MACE1;
                    break;
                }
                // fall through
            case CommsMACE::MavlinkConfiguration::MavlinkVersion::MavlinkVersionAlways2:
                maceStatus->flags &= ~MACE_STATUS_FLAG_OUT_MACE1;
                break;
            default:
            case CommsMACE::MavlinkConfiguration::MavlinkVersion::MavlinkVersionAlways1:
                maceStatus->flags |= MACE_STATUS_FLAG_OUT_MACE1;
                break;
            }
        }


        //connect link
        bool success = m_LinkMarshaler->ConnectToLink(m_LinkName);
        if(success == false) {
            throw std::runtime_error("Connection to link failed");
        }
    }
    else if(params->HasNonTerminal("DigiMeshParameters"))
    {
        std::shared_ptr<MaceCore::ModuleParameterValue> digiMeshSettings = params->GetNonTerminalValue("DigiMeshParameters");


        std::string portName = digiMeshSettings->GetTerminalValue<std::string>("PortName");
        int baudRate = digiMeshSettings->GetTerminalValue<int>("BaudRate");


        CommsMACE::Protocols protocolToUse = CommsMACE::Protocols::MAVLINK;

        CommsMACE::DigiMeshConfiguration config("config");
        config.setPortName(portName);
        config.setBaud(baudRate);

        m_LinkName = "digiMesh_" + portName;
        m_LinkMarshaler->AddDigiMeshLink(m_LinkName, config);


        //now configure to use link with desired protocol
        if(protocolToUse == CommsMACE::Protocols::MAVLINK)
        {
            m_LinkMarshaler->SetProtocolForLink(m_LinkName, CommsMACE::Protocols::MAVLINK);

            std::shared_ptr<CommsMACE::MavlinkConfiguration> mavlinkConfig = std::static_pointer_cast<CommsMACE::MavlinkConfiguration>(m_AvailableProtocols.at(CommsMACE::Protocols::MAVLINK));

            //set version on mavlink channel
            // I would prefer to put this in CommsMACE library, but because the mavlinkstatus is static variable, things get messed up when linking
            m_LinkChan = m_LinkMarshaler->GetProtocolChannel(m_LinkName);
            mace_status_t* maceStatus = mace_get_channel_status(m_LinkChan);
            std::cout << maceStatus << std::endl;
            switch (mavlinkConfig->GetVersion()) {
            case CommsMACE::MavlinkConfiguration::MavlinkVersion::MavlinkVersion2IfVehicle2:
                if (maceStatus->flags & MACE_STATUS_FLAG_IN_MACE1) {
                    maceStatus->flags |= MACE_STATUS_FLAG_OUT_MACE1;
                    break;
                }
                // fall through
            case CommsMACE::MavlinkConfiguration::MavlinkVersion::MavlinkVersionAlways2:
                maceStatus->flags &= ~MACE_STATUS_FLAG_OUT_MACE1;
                break;
            default:
            case CommsMACE::MavlinkConfiguration::MavlinkVersion::MavlinkVersionAlways1:
                maceStatus->flags |= MACE_STATUS_FLAG_OUT_MACE1;
                break;
            }
        }


        //connect link
        bool success = m_LinkMarshaler->ConnectToLink(m_LinkName);
        if(success == false) {
            throw std::runtime_error("Connection to link failed");
        }
    }
    else if(params->HasNonTerminal("UDPParameters"))
    {
        std::shared_ptr<MaceCore::ModuleParameterValue> udpSettings = params->GetNonTerminalValue("UDPParameters");

        std::string listenAddress = udpSettings->GetTerminalValue<std::string>("ListenAddress");
        int listenPortNumber = udpSettings->GetTerminalValue<int>("ListenPortNumber");

        CommsMACE::Protocols protocolToUse = CommsMACE::Protocols::MAVLINK;
        CommsMACE::UdpConfiguration config(listenAddress, listenPortNumber);

        // ********************************************************************************************
        // TODO-PAT: This function is blocking while it listens for the sender port.
        //             --Need to figure out a way to move this to a thread to execute in the background until
        //                  a UDP connection is seen on this address and port number.
//            config.listenForPort(listenPortNumber);

        m_LinkName = "udplink_" + std::to_string(listenPortNumber);
        m_LinkMarshaler->AddUDPLink(m_LinkName, config);

        //now configure to use link with desired protocol
        if(protocolToUse == CommsMACE::Protocols::MAVLINK)
        {
            m_LinkMarshaler->SetProtocolForLink(m_LinkName, CommsMACE::Protocols::MAVLINK);

            std::shared_ptr<CommsMACE::MavlinkConfiguration> mavlinkConfig = std::static_pointer_cast<CommsMACE::MavlinkConfiguration>(m_AvailableProtocols.at(CommsMACE::Protocols::MAVLINK));

            //set version on mavlink channel
            // I would prefer to put this in CommsMACE library, but because the mavlinkstatus is static variable, things get messed up when linking
            m_LinkChan = m_LinkMarshaler->GetProtocolChannel(m_LinkName);
            mace_status_t* maceStatus = mace_get_channel_status(m_LinkChan);
            std::cout << maceStatus << std::endl;
            switch (mavlinkConfig->GetVersion()) {
            case CommsMACE::MavlinkConfiguration::MavlinkVersion::MavlinkVersion2IfVehicle2:
                if (maceStatus->flags & MACE_STATUS_FLAG_IN_MACE1) {
                    maceStatus->flags |= MACE_STATUS_FLAG_OUT_MACE1;
                    break;
                }
                // fall through
            case CommsMACE::MavlinkConfiguration::MavlinkVersion::MavlinkVersionAlways2:
                maceStatus->flags &= ~MACE_STATUS_FLAG_OUT_MACE1;
                break;
            default:
            case CommsMACE::MavlinkConfiguration::MavlinkVersion::MavlinkVersionAlways1:
                maceStatus->flags |= MACE_STATUS_FLAG_OUT_MACE1;
                break;
            }
        }

        //  TODO-PAT: Everything above this to the previous "TODO-PAT" should be moved onto a thread
        // ********************************************************************************************

        //connect link
        if(m_LinkMarshaler->ConnectToLink(m_LinkName) == false){
            throw std::runtime_error("Connection to udp link failed");
        }
    }

    else if(params->HasNonTerminal("EthernetParameters"))
    {
        std::shared_ptr<MaceCore::ModuleParameterValue> ethernetSettings = params->GetNonTerminalValue("EthernetParameters");

        int portNumber = ethernetSettings->GetTerminalValue<int>("PortNumber");

        CommsMACE::Protocols protocolToUse = CommsMACE::Protocols::MAVLINK;
        CommsMACE::EthernetConfiguration config(portNumber);

        // ********************************************************************************************
        // TODO-PAT: This function is blocking while it listens for the sender port.
        //             --Need to figure out a way to move this to a thread to execute in the background until
        //                  a UDP connection is seen on this address and port number.
//            config.listenForPort(listenPortNumber);

        m_LinkName = "ethernetlink_" + std::to_string(portNumber);
        m_LinkMarshaler->AddEthernetLink(m_LinkName, config);

        //now configure to use link with desired protocol
        if(protocolToUse == CommsMACE::Protocols::MAVLINK)
        {
            m_LinkMarshaler->SetProtocolForLink(m_LinkName, CommsMACE::Protocols::MAVLINK);

            std::shared_ptr<CommsMACE::MavlinkConfiguration> mavlinkConfig = std::static_pointer_cast<CommsMACE::MavlinkConfiguration>(m_AvailableProtocols.at(CommsMACE::Protocols::MAVLINK));

            //set version on mavlink channel
            // I would prefer to put this in CommsMACE library, but because the mavlinkstatus is static variable, things get messed up when linking
            m_LinkChan = m_LinkMarshaler->GetProtocolChannel(m_LinkName);
            mace_status_t* maceStatus = mace_get_channel_status(m_LinkChan);
            std::cout << maceStatus << std::endl;
            switch (mavlinkConfig->GetVersion()) {
            case CommsMACE::MavlinkConfiguration::MavlinkVersion::MavlinkVersion2IfVehicle2:
                if (maceStatus->flags & MACE_STATUS_FLAG_IN_MACE1) {
                    maceStatus->flags |= MACE_STATUS_FLAG_OUT_MACE1;
                    break;
                }
                // fall through
            case CommsMACE::MavlinkConfiguration::MavlinkVersion::MavlinkVersionAlways2:
                maceStatus->flags &= ~MACE_STATUS_FLAG_OUT_MACE1;
                break;
            default:
            case CommsMACE::MavlinkConfiguration::MavlinkVersion::MavlinkVersionAlways1:
                maceStatus->flags |= MACE_STATUS_FLAG_OUT_MACE1;
                break;
            }
        }
        //  TODO-PAT: Everything above this to the previous "TODO-PAT" should be moved onto a thread
        // ********************************************************************************************

        //connect link
        if(m_LinkMarshaler->ConnectToLink(m_LinkName) == false){
            throw std::runtime_error("Connection to Ethernet link failed");
        }
    }
    else if(params->HasNonTerminal("TCPParameters"))
    {
        std::shared_ptr<MaceCore::ModuleParameterValue> tcpSettings = params->GetNonTerminalValue("TCPParameters");

        std::string tcpServerAddress = tcpSettings->GetTerminalValue<std::string>("TCPServerAddress");
        int tcpServerPort = tcpSettings->GetTerminalValue<int>("TCPServerPort");
        int udpBroadcastPort = tcpSettings->GetTerminalValue<int>("UDPBroadcastPort");


        CommsMACE::Protocols protocolToUse = CommsMACE::Protocols::MAVLINK;
        CommsMACE::TcpConfiguration config(tcpServerAddress, tcpServerPort, udpBroadcastPort);

        m_LinkName = "tcplink_" + std::to_string(tcpServerPort);
        m_LinkMarshaler->AddTCPLink(m_LinkName, config);

        //now configure to use link with desired protocol
        if(protocolToUse == CommsMACE::Protocols::MAVLINK)
        {
            m_LinkMarshaler->SetProtocolForLink(m_LinkName, CommsMACE::Protocols::MAVLINK);

            std::shared_ptr<CommsMACE::MavlinkConfiguration> mavlinkConfig = std::static_pointer_cast<CommsMACE::MavlinkConfiguration>(m_AvailableProtocols.at(CommsMACE::Protocols::MAVLINK));

            //set version on mavlink channel
            // I would prefer to put this in CommsMACE library, but because the mavlinkstatus is static variable, things get messed up when linking
            m_LinkChan = m_LinkMarshaler->GetProtocolChannel(m_LinkName);
            mace_status_t* maceStatus = mace_get_channel_status(m_LinkChan);
            std::cout << maceStatus << std::endl;
            switch (mavlinkConfig->GetVersion()) {
            case CommsMACE::MavlinkConfiguration::MavlinkVersion::MavlinkVersion2IfVehicle2:
                if (maceStatus->flags & MACE_STATUS_FLAG_IN_MACE1) {
                    maceStatus->flags |= MACE_STATUS_FLAG_OUT_MACE1;
                    break;
                }
                // fall through
            case CommsMACE::MavlinkConfiguration::MavlinkVersion::MavlinkVersionAlways2:
                maceStatus->flags &= ~MACE_STATUS_FLAG_OUT_MACE1;
                break;
            default:
            case CommsMACE::MavlinkConfiguration::MavlinkVersion::MavlinkVersionAlways1:
                maceStatus->flags |= MACE_STATUS_FLAG_OUT_MACE1;
                break;
            }
        }

        //connect link
        if(m_LinkMarshaler->ConnectToLink(m_LinkName) == false){
            throw std::runtime_error("Connection to tcp link failed");
        }
    }
    else
    {
        throw std::runtime_error("No Link has been configured for the vehicle MAVLINK module");
    }

}
