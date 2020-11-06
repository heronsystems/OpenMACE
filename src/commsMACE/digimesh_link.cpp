#include "digimesh_link.h"

//Set this flag to cause random failures on digimesh link. Usefull for testing communication
//#define CAUSE_RANDOM_FAILURES

#ifdef CAUSE_RANDOM_FAILURES
    static const int G_FAILURETEST_CHANCE = 30;
    int G_NUM_CONSECUTIVE_FAILURES = 0;
    static const int G_MAX_CONSECUTIVE_FAILURES = 2;
#endif

namespace CommsMACE
{

char MACE_INSTANCE_STR[] = "MaceInstance";
char VEHICLE_STR[] = "Vehicle";
char GROUNDSTATION_STR[] = "GroundStation";
char MLSTATION_STR[] = "MLStation";
char RTA_STR[] = "RTA";
char EXTERNAL_LINK_STR[] = "ExternalLink";

DigiMeshLink::DigiMeshLink(const DigiMeshConfiguration &config) :
    _config(config),
    m_Link(nullptr)
{
#ifdef CAUSE_RANDOM_FAILURES
    std::srand(std::time(nullptr));
#endif
}

DigiMeshLink::~DigiMeshLink()
{
    if(m_Link != nullptr)
    {
        delete m_Link;
    }
}


void DigiMeshLink::RequestReset()
{

}

void DigiMeshLink::WriteBytes(const char *bytes, int length, const OptionalParameter<Resource> &target)
{
    //pack into std::vector
    std::vector<uint8_t> data;
    for(int i = 0 ; i < length ; i++) {
        data.push_back(bytes[i]);
    }

    //either broadcast or send to specific vehicle
    if(target.IsSet() == true) {

        //convert the target into a datastructure that digimesh library can understand
        ResourceKey key;
        ResourceValue value;

        for(std::size_t i = 0 ; i < target().Size() ; i++)
        {
            key.AddNameToResourceKey(target().NameAt(i));
            value.AddValueToResourceKey(target().IDAt(i));
        }

#ifdef CAUSE_RANDOM_FAILURES
        if((std::rand() % 100) < G_FAILURETEST_CHANCE && G_NUM_CONSECUTIVE_FAILURES <= G_MAX_CONSECUTIVE_FAILURES)
        {
            printf("!!!!Madison Testing!!! Causing a transmission failure. THIS SHOULD NOT HAPPEN IN MASTER, CHECK CAUSE_RANDOM_FAILURES VARIABLE IN DIGIMESH_LINK.CPP\n");
            G_NUM_CONSECUTIVE_FAILURES++;
        }
        else {
            G_NUM_CONSECUTIVE_FAILURES = 0;
            m_Link->SendData(data, key, value);
        }
#else
        m_Link->SendData(data, key, value);
#endif
    }
    else {
        m_Link->BroadcastData(data);
    }
}

void DigiMeshLink::AddResource(const Resource &resource)
{
    //convert the target into a datastructure that digimesh library can understand
    ResourceKey key;
    ResourceValue value;

    for(std::size_t i = 0 ; i < resource.Size() ; i++)
    {
        key.AddNameToResourceKey(resource.NameAt(i));
        value.AddValueToResourceKey(resource.IDAt(i));
    }

    m_Link->AddResource(key, value);
}


bool DigiMeshLink::HasResource(const Resource &resource) const
{
    //convert the target into a datastructure that digimesh library can understand
    ResourceKey key;
    ResourceValue value;

    for(std::size_t i = 0 ; i < resource.Size() ; i++)
    {
        key.AddNameToResourceKey(resource.NameAt(i));
        value.AddValueToResourceKey(resource.IDAt(i));
    }

    return m_Link->HasResource(key, value);
}

void DigiMeshLink::RequestRemoteResources() const
{
    return m_Link->RequestRemoteResources();
}


bool DigiMeshLink::isConnected() const
{
    if(m_Link == nullptr)
    {
        return false;
    }
    return true;
}

std::string DigiMeshLink::getPortName() const
{
    return _config.portName();
}

uint64_t DigiMeshLink::getConnectionSpeed() const
{
    throw std::runtime_error("Not Implimented");
}

bool DigiMeshLink::Connect()
{
    m_Link = new MACEDigiMeshWrapper<MACE_INSTANCE_STR, VEHICLE_STR, GROUNDSTATION_STR,MLSTATION_STR, RTA_STR, EXTERNAL_LINK_STR>(_config.portName(), _config.baud());

    m_Link->AddHandler_NewRemoteComponentItem_Generic([this](const ResourceKey &resourceKey, const ResourceValue &resourceValue, uint64_t addr){
        UNUSED(addr);

        Resource r;
        for(std::size_t i = 0 ; i < resourceKey.size() ; i++)
        {
            r.Add(resourceKey.at(i), resourceValue.at(i));
        }

        EmitEvent([this, &r](ILinkEvents *ptr){ptr->AddedExternalResource(this, r);});
    });

    m_Link->AddHandler_Data([this](const std::vector<uint8_t> &data){
        EmitEvent([this,&data](const ILinkEvents *ptr){ptr->ReceiveData(this, data);});
    });

    return true;
}

void DigiMeshLink::Disconnect()
{

}

} // END Comms
