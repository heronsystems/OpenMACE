#include <QCoreApplication>
#include <windows.h>

#include "commsMACE/comms_marshaler_mace.h"




class Listener : public CommsMACE::CommsEvents
{
    //!
    //! \brief New Mavlink message received over a link
    //! \param linkName Name of link message received over
    //! \param msg Message received
    //!
    virtual void MACEMessage(const std::string &linkName, const mace_message_t &msg)
    {
        printf("MACE Message from: %s\n", linkName.c_str());

        if(msg.msgid != MACE_MSG_ID_HEARTBEAT) { printf("ERROR! RECEIVED MSG NOT HEARTBEAT"); }

        mace_heartbeat_t heartbeat;
        mace_msg_heartbeat_decode(&msg, &heartbeat);
        printf("%d %d\n", heartbeat.mavlinkID, heartbeat.type);

    }
};


int main(int argc, char *argv[])
{


    CommsMACE::CommsMarshaler marshaler;

    CommsMACE::MavlinkConfiguration mavlinkConfig;
    marshaler.AddProtocol(mavlinkConfig);

    CommsMACE::EthernetConfiguration config1(45553);
    marshaler.AddEthernetLink("Link1", config1);

    CommsMACE::EthernetConfiguration config2(45554);
    marshaler.AddEthernetLink("Link2", config2);

    marshaler.ConnectToLink("Link1");
    marshaler.ConnectToLink("Link2");

    marshaler.SetProtocolForLink("Link1", CommsMACE::Protocols::MAVLINK);
    marshaler.SetProtocolForLink("Link2", CommsMACE::Protocols::MAVLINK);

    Listener myListener;
    marshaler.AddSubscriber(&myListener);

    Sleep(1000);

    CommsMACE::Resource resource1A;
    resource1A.Add("MaceInstance", 1);

    CommsMACE::Resource resource1B;
    resource1B.Add("MaceInstance", 1);
    resource1B.Add("Vehicle", 1);

    marshaler.AddResource("Link1", resource1A);
    marshaler.AddResource("Link1", resource1B);


    CommsMACE::Resource resource2A;
    resource2A.Add("MaceInstance", 2);

    CommsMACE::Resource resource2B;
    resource2B.Add("MaceInstance", 2);
    resource2B.Add("Vehicle", 1);

    marshaler.AddResource("Link2", resource2A);
    marshaler.AddResource("Link2", resource2B);

    Sleep(1000);

    mace_message_t msg;

    uint8_t index;
    while(true)
    {
        mace_heartbeat_t heartbeat;
        heartbeat.mavlinkID = 1;
        heartbeat.type = index;
        mace_msg_heartbeat_encode(1,1,&msg, &heartbeat);
        marshaler.SendMACEMessage("Link2", msg);

        index++;

        Sleep(100);
    }

    while(true){

    }
}

