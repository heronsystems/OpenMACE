#ifndef MACE_DIGIMESH_INTEROP_H
#define MACE_DIGIMESH_INTEROP_H

#include <vector>
#include <functional>
#include <mutex>

#include "digi_common/digi_mesh_baud_rates.h"
#include "digi_common/transmit_status_types.h"
#include "resource.h"

#include "macewrapper_global.h"






/**
 * @brief The Interop class
 *
 * This object employs its own protocol internal to itself.
 *
 * Byte Array (N+1) - Send data to a remote node
 *      0x01 | <data1> | <data1> | ... | <dataN>
 *
 * Entity Present (N+5) - Entity that a vehicle has been attached to the node
 *      0x02 | Name0 | Name1 | ... | NameN | '\0' | ID byte 1 (MSB) | ID byte 2 | ID byte 3 | ID byte 1 (LSB)
 *
 * Contained Vehciles Request (1) - Make request to remote node(s) to send all vehicles present
 *      0x03
 *
 * Remove Entity (N+5) - Signal that a vehicle attached to a node is no longer
 *      0x04 | Name0 | Name1 | ... | NameN | '\0' | ID byte 1 (MSB) | ID byte 2 | ID byte 3 | ID byte 1 (LSB)
 *
 */
class Interop
{

private:

    enum class PacketTypes
    {
        DATA = 0x01,
        COMPONENT_ITEM_PRESENT = 0x02,
        CONTAINED_VECHILES_REQUEST = 0x03,
        REMOVE_COMPONENT_ITEM = 0x04
    };

    static const char NI_NAME_VEHICLE_delimiter = '|';

    void* m_Radio;

    std::mutex m_NIMutex;

    std::vector<std::function<void(const std::vector<uint8_t>&)>> m_Handlers_Data;

    std::string m_NodeName;

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
    Interop(const std::string &port, DigiMeshBaudRates rate, const std::string &nameOfNode = "", bool scanForNodes = false);

    ~Interop();


    /**
     * @brief Add handler to be called when data has been received to this node
     * @param lambda Lambda function accepting an array of single byte values
     */
    void AddHandler_Data(const std::function<void (const std::vector<uint8_t> &)> &lambda);


    /**
     * @brief Broadcast data to all nodes
     * @param data Data to broadcast out
     */
    void BroadcastData(const std::vector<uint8_t> &data);



protected:

    void SendDataToAddress(uint64_t addr, const std::vector<uint8_t> &data, const std::function<void(const TransmitStatusTypes &)> &cb);

    void RequestContainedResources(const ResourceKey &key) const;

protected:

    virtual void onNewRemoteComponentItem(const ResourceKey &key, const ResourceValue &resource, uint64_t addr) = 0;

    virtual void onRemovedRemoteComponentItem(const ResourceKey &key, const ResourceValue &resource) = 0;

    virtual std::vector<std::tuple<ResourceKey, ResourceValue>> RetrieveComponentItems(const ResourceKey &key, bool internal = false) = 0;

protected:

    /**
     * @brief Logic to perform upon reception of a message
     * @param msg Message received
     */
    void on_message_received(const std::vector<uint8_t> &msg, uint64_t addr);


    void send_item_present_message(const ResourceKey &key, const ResourceValue &resource);

    void send_item_remove_message(const ResourceKey &key, const ResourceValue &resource);

};

#endif // MACE_DIGIMESH_INTEROP_H
