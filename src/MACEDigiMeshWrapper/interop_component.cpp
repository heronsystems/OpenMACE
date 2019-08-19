#include "interop_component.h"

/**
 * @brief Constructor
 *
 * If no name is given, this node will set names according to the vehicles present
 * @param port Port to communicate with DigiMesh Radio
 * @param rate Baud Rate to communicate at
 * @param nameOfNode [""] Optional name of node
 * @param scanForVehicles [false] Indicate if this radio should scan for other MACE vehicles, or rely upon messages sent.
 */
InteropComponent::InteropComponent(const std::string &port, DigiMeshBaudRates rate, const std::string &nameOfNode, bool scanForNodes)
    : Interop(port, rate, nameOfNode, scanForNodes)
{

}

void InteropComponent::AddResource(const ResourceKey &key, const ResourceValue &value)
{
    m_Resources.AddInternalResource(key, value);
    send_item_present_message(key, value);
}

bool InteropComponent::HasResource(const ResourceKey &key, const ResourceValue &value) const
{
    return m_Resources.HasAddr(key, value);
}


void InteropComponent::RemoveResource(const ResourceKey &key, const ResourceValue &value)
{
    m_Resources.RemoveInternalResource(key, value);
    send_item_remove_message(key, value);
}


/**
 * @brief Add handler to be called when a new vehicle is added to the network
 * @param lambda Lambda function whoose parameters are the vehicle ID and node address of new vechile.
 */
void InteropComponent::AddHandler_NewRemoteComponentItem(const ResourceKey &key, const std::function<void(ResourceValue, uint64_t)> &lambda)
{
    if(m_Handlers_NewRemoteVehicle.find(key) == m_Handlers_NewRemoteVehicle.cend()) {
        m_Handlers_NewRemoteVehicle.insert({key, {}});
    }
    m_Handlers_NewRemoteVehicle.at(key).push_back(lambda);
}


/**
 * @brief Add handler to be called when a new vehicle has been removed from the network
 * @param lambda Lambda function whoose parameters are the vehicle ID of removed vechile.
 */
void InteropComponent::AddHandler_RemoteComponentItemRemoved(const ResourceKey &key, const std::function<void(ResourceValue)> &lambda)
{
    if(m_Handlers_RemoteVehicleRemoved.find(key) == m_Handlers_RemoteVehicleRemoved.cend()) {
        m_Handlers_RemoteVehicleRemoved.insert({key, {}});
    }
    m_Handlers_RemoteVehicleRemoved.at(key).push_back(lambda);
}


/**
 * @brief Add handler to be called when tranmission to a vehicle failed for some reason.
 * @param lambda Lambda function to pass vehicle ID and status code
 */
void InteropComponent::AddHandler_ComponentItemTransmitError(const ResourceKey &key, const std::function<void(ResourceValue vehicle, TransmitStatusTypes status)> &lambda)
{
    if(m_Handlers_VehicleNotReached.find(key) == m_Handlers_VehicleNotReached.cend()) {
        m_Handlers_VehicleNotReached.insert({key, {}});
    }
    m_Handlers_VehicleNotReached.at(key).push_back(lambda);
}



/**
 * @brief Add handler to be called when a new vehicle is added to the network
 * @param lambda Lambda function whoose parameters are the vehicle ID and node address of new vechile.
 */
void InteropComponent::AddHandler_NewRemoteComponentItem_Generic(const std::function<void(ResourceKey, ResourceValue, uint64_t)> &lambda)
{
    m_Handlers_NewRemoteVehicle_Generic.push_back(lambda);
}


/**
 * @brief Add handler to be called when a new vehicle has been removed from the network
 * @param lambda Lambda function whoose parameters are the vehicle ID of removed vechile.
 */
void InteropComponent::AddHandler_RemoteComponentItemRemoved_Generic(const std::function<void(ResourceKey, ResourceValue)> &lambda)
{
    m_Handlers_RemoteVehicleRemoved_Generic.push_back(lambda);
}


/**
 * @brief Add handler to be called when tranmission to a vehicle failed for some reason.
 * @param lambda Lambda function to pass vehicle ID and status code
 */
void InteropComponent::AddHandler_ComponentItemTransmitError_Generic(const std::function<void(ResourceKey, ResourceValue, TransmitStatusTypes)> &lambda)
{
    m_Handlers_VehicleNotReached_Generic.push_back(lambda);
}


/**
 * @brief Send data to a component item
 * @param component Name of component to send to
 * @param destVechileID ID of item
 * @param data Data to send
 * @return False if given ID/component doesn't exists
 */
bool InteropComponent::SendData(const ResourceKey &resourceKey, const ResourceValue &resourceValue, const std::vector<uint8_t> &data)
{
    if(m_Resources.HasAddr(resourceKey, resourceValue) == false)
    {
        std::string str = "[ ";
        for(int i = 0 ; i < resourceKey.size() ; i++)
        {
            str += resourceKey.at(i) + " ";
        }
        str += "] { ";
        for(int i = 0 ; i < resourceValue.size() ; i++)
        {
            str += std::to_string(resourceValue.at(i)) + " ";
        }
        str += "}";

        throw std::runtime_error("No address known for given target: " + str);
    }

    //address to send to
    uint64_t addr = m_Resources.GetAddr(resourceKey, resourceValue);

    SendDataToAddress(addr, data, [this, resourceKey, resourceValue](const TransmitStatusTypes &status){

        if(status != TransmitStatusTypes::SUCCESS)
        {
            if(m_Handlers_VehicleNotReached.find(resourceKey) != m_Handlers_VehicleNotReached.cend())
            {
                Notify<ResourceValue, TransmitStatusTypes>(m_Handlers_VehicleNotReached.at(resourceKey), resourceValue, status);
            }
            else {
                Notify<ResourceKey, ResourceValue, TransmitStatusTypes>(m_Handlers_VehicleNotReached_Generic, resourceKey, resourceValue, status);
            }
        }
    });

    return true;
}



void InteropComponent::onNewRemoteComponentItem(const ResourceKey &resourceKey, const ResourceValue &resourceValue, uint64_t addr)
{
    bool newResource = m_Resources.AddExternalResource(resourceKey, resourceValue, addr);

    //if not a new resource, then we are done.
    if(newResource == false)
    {
        return;
    }

    if(m_Handlers_NewRemoteVehicle.find(resourceKey) != m_Handlers_NewRemoteVehicle.cend())
    {
        Notify<ResourceValue, uint64_t>(m_Handlers_NewRemoteVehicle.at(resourceKey), resourceValue, addr);
    }
    else {
        Notify<ResourceKey, ResourceValue, uint64_t>(m_Handlers_NewRemoteVehicle_Generic, resourceKey, resourceValue, addr);
    }
}

void InteropComponent::onRemovedRemoteComponentItem(const ResourceKey &resourceKey, const ResourceValue &resourceValue)
{
    m_Resources.RemoveExternalResource(resourceKey, resourceValue);

    if(m_Handlers_RemoteVehicleRemoved.find(resourceKey) != m_Handlers_RemoteVehicleRemoved.cend())
    {
        Notify<ResourceValue>(m_Handlers_RemoteVehicleRemoved.at(resourceKey), resourceValue);
    }
    else {
        Notify<ResourceKey, ResourceValue>(m_Handlers_RemoteVehicleRemoved_Generic, resourceKey, resourceValue);
    }
}


std::vector<std::tuple<ResourceKey, ResourceValue>> InteropComponent::RetrieveComponentItems(const ResourceKey &key, bool internal)
{
    return m_Resources.getResourcesMatch(key, true, internal);
}
