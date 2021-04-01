///
/// EXAMPLE CONTROLLER
///
/// This controller sets up an example of some instance sending resource notifications to another controller.
///
/// A "resource notification" is a packet containing a single string.
/// Upon receiving such notificaiton the receiver shall respond with an ack so sender knows it was received
///
///

#ifndef EXAMPLE_CONTROLLER_H
#define EXAMPLE_CONTROLLER_H

#include <vector>

#include "controllers/generic_controller.h"

#include "controllers/actions/action_send.h"
#include "controllers/actions/action_finish.h"
#include "controllers/actions/action_unsolicited_receive_respond.h"

#include "comms.h"


///////////////////////////////////////////////////////////////////////////////
/// Define classes that we are going to utilize for this example
///   In MACE this is either a single identifier (vehicle) or ModuleCharacterstic (modules)
///   or some other variation depending on specific controller
///////////////////////////////////////////////////////////////////////////////




//!
//! \brief Class that identifies the data on the local entity.
//!
//! To identify data among all entities, both DataIdentifier and ExampleEntity will be required
//!
class ResourceIdentifier
{
public:
    uint8_t ID;
};


//!
//! \brief Class representing the data desired to be delivered
//!
class ResourceData
{
public:
    std::string name;
};


class EntityResourceTuple
{
public:
    uint8_t EntityID;
    uint8_t ResourceID;

    bool operator ==(const EntityResourceTuple &rhs) const
    {
        if(this->EntityID != rhs.EntityID)
        {
            return false;
        }
        if(this->ResourceID != rhs.ResourceID) { return false; }

        return true;
    }

    bool operator !=(const EntityResourceTuple &rhs) const
    {
        return !(*this == rhs);
    }
};

namespace std {

  template <>
  struct hash<EntityResourceTuple>
  {
    std::size_t operator()(const EntityResourceTuple& k) const
    {
      using std::size_t;
      using std::hash;
      using std::string;

      // Compute individual hash values for first,
      // second and third and combine them using XOR
      // and bit shifting:

      return hash<int>()(k.EntityID) ^ hash<int>()(k.ResourceID);
    }
  };

}

///////////////////////////////////////////////////////////////////////////////
/// Decalare template instantions
///////////////////////////////////////////////////////////////////////////////


//!
//! Declaring the template instantiations of the GenericController for this example controller
//!
//! In this case our I/O is through CommPacket object
//! Entities are Identified using ExampleEntity class
//! We are planning on queing transmissions based on both ExampleEntity and ResourceIdentifier classes
//! When finished we are expecting a single byte ack code
//! Finally the Controller will be kicked-off/finalized by providing a key (ResourceIdentifier) and data (ResourceData)
//!
using ExampleControllerType = Controllers::GenericController<
    CommPacket, ExampleEntity,
    TransmitQueueWithKeys<ExampleEntity, ObjectIntTuple<EntityResourceTuple>>,
    uint8_t,
    Controllers::DataItem<ResourceIdentifier, ResourceData>
>;


///////////////////////////////////////////////////////////////////////////////
/// Declare actions that we are going to use
///////////////////////////////////////////////////////////////////////////////


//!
//! Send resource data to a specific entity.
//!
//! This action will be triggered by sender
//!
//! This action will cause a resource_notification_t message to be sent about a specific resource to a specific entity.
//! By queing on std::tuple<ExampleEntity, ResourceIdentifier>, this allows the controller to send/manage transmissions to any combination of entity/resource.
//!
//! The action will configure the transmission to end upon receiving a RESOURCE_ACK message that maches the entity/resource combination.
//!
using ActionSend = Controllers::ActionSend<
    CommPacket, ExampleEntity,
    ExampleControllerType,
    EntityResourceTuple,
    std::tuple<ResourceIdentifier, ResourceData>,
    resource_notification_t,
    RESOURCE_ACK
>;


//!
//! Action to receive the unsolicited (not requested) Resource notitfication and respond with an ack
//!
//! This action will be triggered by receiver
//!
//! The ACK response is not reliable (not placed in queue) because the resource notification sender will not respond upon receiving an ACK.
//! If the ACK gets lost the resource notification sender will resend the notification, so the user/controller must be able to adapt to such a scenerio.
//!
using ActionReceiveRespond = Controllers::ActionUnsolicitedReceiveRespond<
    CommPacket, ExampleEntity,
    ExampleControllerType,
    ResourceIdentifier,
    ResourceData,
    resource_notification_t,
    resource_notification_ack_t,
    RESOURCE_NOTIFICATION
>;


//!
//! Receive ack and finish out notification transmission
//!
//! This action will be triggered by sender when it receives an ACK from receiver
//! The action will dequeue the previous tranmission and notify user that the controller has completed it's action with an ack (of type uint8_t for this example)
//!
using ActionFinish = Controllers::ActionFinish<
    CommPacket, ExampleEntity,
    ExampleControllerType,
    EntityResourceTuple,
    uint8_t,
    resource_notification_ack_t,
    RESOURCE_ACK
>;


///////////////////////////////////////////////////////////////////////////////
/// Instantiate controller
///////////////////////////////////////////////////////////////////////////////

class ExampleController : public ExampleControllerType,
        public ActionSend,
        public ActionReceiveRespond,
        public ActionFinish
{
public:

    virtual bool Construct_Send(const std::tuple<ResourceIdentifier, ResourceData> &data, const ExampleEntity &sender, const ExampleEntity &target, resource_notification_t &msg, EntityResourceTuple &queue)
    {
        msg.ID = std::get<0>(data).ID;
        msg.str = std::get<1>(data).name;

        //set up queue such that transmission terminates upon receiving a message from the target for the given resource ID
        queue.EntityID = target.ID;
        queue.ResourceID = std::get<0>(data).ID;

        std::cout << "Sending Resource Notification" << std::endl;

        return true;
    }


    virtual bool Construct_FinalObjectAndResponse(const resource_notification_t &msg, const ExampleEntity &sender, resource_notification_ack_t &response, ExampleEntity &componentResponding, ResourceIdentifier &dataKey, ResourceData &data)
    {
        //construct response packet
        response.ID = msg.ID;

        //construct data to pass up
        dataKey.ID = msg.ID;
        data.name = msg.str;

        //determine the component that is to respond
        componentResponding = this->GetHostKey();

        std::cout << "Received Resource Notification, sending ACK" << std::endl;

        return true;
    }


    virtual bool Finish_Receive(const resource_notification_ack_t &msg, const ExampleEntity &sender, uint8_t& ack, EntityResourceTuple &queueObj)
    {
        ack = 0;

        queueObj.EntityID = sender.ID;
        queueObj.ResourceID = msg.ID;

        std::cout << "Received Resource Notification ACK" << std::endl;

        return true;
    }


    ExampleController(const Controllers::IMessageNotifier<CommPacket, ExampleEntity> *cb, TransmitQueue *queue, int linkChan) :
        ExampleControllerType(cb, queue, linkChan),
        ActionSend(this, resource_notification_encode),
        ActionReceiveRespond(this, resource_notification_decode, resource_notification_ack_encode),
        ActionFinish(this, resource_notification_ack_decode)
    {

    }
};

#endif // EXAMPLE_CONTROLLER_H
