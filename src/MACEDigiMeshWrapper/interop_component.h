#ifndef INTEROP_ENTITIES_H
#define INTEROP_ENTITIES_H

#include "interop.h"
#include "component.h"

#include "macewrapper_global.h"

class InteropComponent : public Interop
{
private:


    std::map<ResourceKey, std::vector<std::function<void(ResourceValue, uint64_t)>>> m_Handlers_NewRemoteVehicle;
    std::map<ResourceKey, std::vector<std::function<void(ResourceValue)>>> m_Handlers_RemoteVehicleRemoved;
    std::map<ResourceKey, std::vector<std::function<void(ResourceValue, TransmitStatusTypes)>>> m_Handlers_VehicleNotReached;

    std::vector<std::function<void(ResourceKey, ResourceValue resource, uint64_t)>> m_Handlers_NewRemoteVehicle_Generic;
    std::vector<std::function<void(ResourceKey, ResourceValue resource)>> m_Handlers_RemoteVehicleRemoved_Generic;
    std::vector<std::function<void(ResourceKey, ResourceValue resource, TransmitStatusTypes)>> m_Handlers_VehicleNotReached_Generic;

    ResourceList m_Resources;


public:
    /**
     * @brief Constructor
     *
     * If no name is given, this node will set names according to the vehicles present
     * @param port Port to communicate with DigiMesh Radio
     * @param rate Baud Rate to communicate at
     * @param nameOfNode [""] Optional name of node
     * @param scanForVehicles [false] Indicate if this radio should scan for other MACE vehicles, or rely upon messages sent.
     */
    InteropComponent(const std::string &port, DigiMeshBaudRates rate, const std::string &nameOfNode = "", bool scanForNodes = false);

protected:

    void AddResource(const ResourceKey &key, const ResourceValue &value);

    bool HasResource(const ResourceKey &key, const ResourceValue &value) const;


    void RemoveResource(const ResourceKey &key, const ResourceValue &resource);


    /**
     * @brief Add handler to be called when a new vehicle is added to the network
     * @param lambda Lambda function whoose parameters are the vehicle ID and node address of new vechile.
     */
    void AddHandler_NewRemoteComponentItem(const ResourceKey &key, const std::function<void(ResourceValue, uint64_t)> &lambda);


    /**
     * @brief Add handler to be called when a new vehicle has been removed from the network
     * @param lambda Lambda function whoose parameters are the vehicle ID of removed vechile.
     */
    void AddHandler_RemoteComponentItemRemoved(const ResourceKey &key, const std::function<void(ResourceValue)> &lambda);


    /**
     * @brief Add handler to be called when tranmission to a vehicle failed for some reason.
     * @param lambda Lambda function to pass vehicle ID and status code
     */
    void AddHandler_ComponentItemTransmitError(const ResourceKey &key, const std::function<void(ResourceValue value, TransmitStatusTypes status)> &lambda);






    /**
     * @brief Add handler to be called when a new vehicle is added to the network
     * @param lambda Lambda function whoose parameters are the vehicle ID and node address of new vechile.
     */
    void AddHandler_NewRemoteComponentItem_Generic(const std::function<void(ResourceKey, ResourceValue, uint64_t)> &lambda);


    /**
     * @brief Add handler to be called when a new vehicle has been removed from the network
     * @param lambda Lambda function whoose parameters are the vehicle ID of removed vechile.
     */
    void AddHandler_RemoteComponentItemRemoved_Generic(const std::function<void(ResourceKey, ResourceValue)> &lambda);


    /**
     * @brief Add handler to be called when tranmission to a vehicle failed for some reason.
     * @param lambda Lambda function to pass vehicle ID and status code
     */
    void AddHandler_ComponentItemTransmitError_Generic(const std::function<void(ResourceKey, ResourceValue, TransmitStatusTypes)> &lambda);




protected:

    /**
     * @brief Send data to a component item
     * @param component Name of component to send to
     * @param destVechileID ID of item
     * @param data Data to send
     * @return False if given ID/component doesn't exists
     */
    bool SendData(const ResourceKey &key, const ResourceValue &resource, const std::vector<uint8_t> &data);

protected:


    virtual void onNewRemoteComponentItem(const ResourceKey &key, const ResourceValue &resource, uint64_t addr);

    virtual void onRemovedRemoteComponentItem(const ResourceKey &resourceKey, const ResourceValue &resourceValue);

    virtual std::vector<std::tuple<ResourceKey, ResourceValue> > RetrieveComponentItems(const ResourceKey &key, bool internal = false);


protected:


};

#endif // INTEROP_ENTITIES_H
