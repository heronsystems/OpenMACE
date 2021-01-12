#ifndef CONTROLLER_BOUNDARY_H
#define CONTROLLER_BOUNDARY_H

#include "common/watchdog.h"

#include "base/pose/cartesian_position_2D.h"
#include "data_generic_command_item/command_item_components.h"

#include "controllers/generic_controller.h"

#include "controllers/actions/action_send.h"
#include "controllers/actions/action_intermediate_respond.h"
#include "controllers/actions/action_intermediate_receive.h"
#include "controllers/actions/action_intermediate.h"
#include "controllers/actions/action_intermediate_unsolicited.h"
#include "controllers/actions/action_final_receive_respond.h"
#include "controllers/actions/action_finish.h"
#include "controllers/actions/action_request.h"

#include "controllers/actions/action_broadcast.h"
#include "controllers/actions/action_broadcast_reliable.h"
#include "controllers/actions/action_unsolicited_receive.h"
#include "controllers/actions/action_unsolicited_receive_respond.h"

#include "module_external_link/controllers/common.h"

#include "../pair_module_boundary_identifier.h"


using namespace BoundaryItem;


//!
//! \brief Structure of data to send when notifying the presense of a border.
//!
//! Because mavlink messages can not send vectors without a significant protocol
//! a single vehicle is sent per packet.
//!
//! If a boundary contains multiple vehicles, the packet will have to be sent multiple times
//! with different vehicleApplicableTo. The recevier will then use uniqueIdentifier to assemble them into one.
//!
struct BoundaryNotificationData
{
    BoundaryItem::BoundaryCharacterisic characteristic; //! Characterstic of boundary
    uint8_t uniqueIdentifier; //! Unique number to identify the boundary ON THE HOST
};



namespace ExternalLink{

using CONTROLLER_BOUNDARY_TYPE = Controllers::GenericController<
    mavlink_message_t, MaceCore::ModuleCharacteristic,
    TransmitQueueWithKeys<MaceCore::ModuleCharacteristic, ObjectMaceMsgIDTuple<MaceCore::BoundaryIdentifierType>, ObjectMaceMsgIDTuple<ModuleBoundaryIdentifier>>,
    uint8_t,
    Controllers::DataItem<MaceCore::ModuleCharacteristic, BoundaryNotificationData>,
    Controllers::DataItem<ModuleBoundaryIdentifier, BoundaryItem::BoundaryList>

>;

using SendBoundaryHelper_RequestDownload = Controllers::ActionSend<
    mavlink_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_BOUNDARY_TYPE,
    ModuleBoundaryIdentifier,
    uint8_t,
    mavlink_boundary_request_list_t,
    MAVLINK_MSG_ID_BOUNDARY_COUNT
>;


using BoundaryControllerAction_ReceiveUnsolicitedRequestList_SendCount = Controllers::ActionIntermediateUnsolicited<
    mavlink_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_BOUNDARY_TYPE,
    ModuleBoundaryIdentifier,
    mavlink_boundary_request_list_t,
    MAVLINK_MSG_ID_BOUNDARY_REQUEST_LIST,
    mavlink_boundary_count_t,
    MAVLINK_MSG_ID_BOUNDARY_REQUEST_ITEM
>;


using SendBoundaryHelper_ReceiveCountRespondItemRequest = Controllers::ActionIntermediate<
    mavlink_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_BOUNDARY_TYPE,
    ModuleBoundaryIdentifier,
    ModuleBoundaryIdentifier,
    mavlink_boundary_count_t,
    MAVLINK_MSG_ID_BOUNDARY_COUNT,
    mavlink_boundary_request_item_t,
    MAVLINK_MSG_ID_BOUNDARY_ITEM
>;


using SendBoundaryHelper_RequestItem = Controllers::ActionIntermediate<
    mavlink_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_BOUNDARY_TYPE,
    ModuleBoundaryIdentifier,
    ModuleBoundaryIdentifier,
    mavlink_boundary_request_item_t,
    MAVLINK_MSG_ID_BOUNDARY_REQUEST_ITEM,
    mavlink_boundary_item_t,
    MAVLINK_MSG_ID_BOUNDARY_REQUEST_ITEM,
    MAVLINK_MSG_ID_BOUNDARY_ACK
>;


using SendBoundaryHelper_ReceiveItem = Controllers::ActionIntermediateReceive<
    mavlink_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_BOUNDARY_TYPE,
    ModuleBoundaryIdentifier,
    ModuleBoundaryIdentifier,
    mavlink_boundary_item_t,
    MAVLINK_MSG_ID_BOUNDARY_ITEM,
    mavlink_boundary_request_item_t
>;

using SendBoundaryHelper_Final = Controllers::ActionFinalReceiveRespond<
    mavlink_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_BOUNDARY_TYPE,
    ModuleBoundaryIdentifier,
    ModuleBoundaryIdentifier,
    BoundaryItem::BoundaryList,
    mavlink_boundary_item_t,
    mavlink_boundary_ack_t,
    MAVLINK_MSG_ID_BOUNDARY_ITEM
>;

using SendBoundaryHelper_FinalFinal = Controllers::ActionFinish<
    mavlink_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_BOUNDARY_TYPE,
    ModuleBoundaryIdentifier,
    uint8_t,
    mavlink_boundary_ack_t,
    MAVLINK_MSG_ID_BOUNDARY_ACK
>;







using NewBoundaryNotification = Controllers::ActionBroadcastReliable_MultiPacket<
    mavlink_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_BOUNDARY_TYPE,
    BoundaryNotificationData,
    MaceCore::BoundaryIdentifierType,
    mavlink_new_boundary_object_t,
    MAVLINK_MSG_ID_BOUNDARY_ACK
>;

using UsolicitedReceiveNewBoundaryNotification = Controllers::ActionUnsolicitedReceiveRespond<
    mavlink_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_BOUNDARY_TYPE,
    MaceCore::ModuleCharacteristic,
    BoundaryNotificationData,
    mavlink_new_boundary_object_t,
    mavlink_boundary_ack_t,
    MAVLINK_MSG_ID_NEW_BOUNDARY_OBJECT
>;

using NewBoundaryNotification_AckReceive = Controllers::ActionFinish<
    mavlink_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_BOUNDARY_TYPE,
    MaceCore::BoundaryIdentifierType,
    uint8_t,
    mavlink_boundary_ack_t,
    MAVLINK_MSG_ID_BOUNDARY_ACK
>;


class ControllerBoundary : public CONTROLLER_BOUNDARY_TYPE,
        public SendBoundaryHelper_RequestDownload,
        public BoundaryControllerAction_ReceiveUnsolicitedRequestList_SendCount,
        public SendBoundaryHelper_ReceiveCountRespondItemRequest,
        public SendBoundaryHelper_RequestItem,
        public SendBoundaryHelper_ReceiveItem,
        public SendBoundaryHelper_Final,
        public SendBoundaryHelper_FinalFinal,
        public NewBoundaryNotification,
        public UsolicitedReceiveNewBoundaryNotification,
        public NewBoundaryNotification_AckReceive
//        public Action_RequestCurrentBoundary_Initiate,
//        public Action_RequestCurrentBoundary_Response,
//        public Action_RequestCurrentBoundary_NoBoundaryResponse
{

private:

    OptionalParameter<MaceCore::ModuleCharacteristic> m_GenericRequester;

    //! Object that maps HostModule,BoundaryID pair to Destination,Object pair
    std::map<ModuleBoundaryIdentifier, std::tuple<MaceCore::ModuleCharacteristic, BoundaryItem::BoundaryList>> m_BoundariesBeingFetching;

    std::map<ModuleBoundaryIdentifier, BoundaryItem::BoundaryList> m_BoundariesUploading;

    //! Member to hold the vehicles in a boundary. Will be built up as the boundary is learned about.
    std::unordered_map<ModuleBoundaryIdentifier, std::vector<int>> m_VehiclesInBoundary;
    std::unordered_map<ModuleBoundaryIdentifier, std::shared_ptr<Watchdog>> m_BoundaryBuilderWatchdogs;

protected:

    //!
    //! \brief Download Action - Initiate download with a request list message
    //! \param data Data given to send.
    //! \param sender Module sending data
    //! \param target Module targeting
    //! \param cmd Message to contruct
    //! \param queueObj Queue object to contruct identifitying this transmission
    //! \return True if transmission should continue
    //!
    virtual bool Construct_Send(const uint8_t &data, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mavlink_boundary_request_list_t &cmd, ModuleBoundaryIdentifier &queueObj);


    //!
    //! \brief Upload Action - Receive request list and respond with count
    //! \param cmd Message received asking for count of a boundary
    //! \param sender Module that sent request
    //! \param rtn Message to construct containing count
    //! \param moduleUploadingFrom Module to contruct that contains the boundary to upload
    //! \param respondQueueObj Object to to construct that indicates how the message should be queued
    //! \return True if transmission should continue
    //!
    virtual bool IntermediateUnsolicitedReceive(const mavlink_boundary_request_list_t &cmd, const MaceCore::ModuleCharacteristic &sender, mavlink_boundary_count_t &rtn, MaceCore::ModuleCharacteristic &vehicleObj, ModuleBoundaryIdentifier &respondQueueObj);


    //!
    //! \brief Download Action - Function on downloading side that receives a count and makes request for first item
    //! \param msg Count message received
    //! \param sender Module that contains the boundary
    //! \param rtn Object to set containing message to send after following up.
    //! \param moduleDownloadingTo Object to set indicating what module requested the boundary
    //! \param receiveQueueObj Object to set that will remove any queued transmission
    //! \param respondQueueObj Object to set to queue next transmission
    //! \return True if message is to be used
    //!
    virtual bool BuildData_Send(const mavlink_boundary_count_t &boundary, const MaceCore::ModuleCharacteristic &sender, mavlink_boundary_request_item_t &request, MaceCore::ModuleCharacteristic &vehicleObj, ModuleBoundaryIdentifier &receiveQueueObj, ModuleBoundaryIdentifier &respondQueueObj);


    //!
    //! \brief Upload Action - Receive a item request and respond with item
    //! \param msg Request Item message received
    //! \param sender Module that sent message
    //! \param boundaryItem Message to construct to send back
    //! \param moduleUploadingFrom Object to construct indicating what module boundary is downloading from
    //! \param receiveQueueObj Object to set that will remove any queued transmission
    //! \param respondQueueObj Object to set to queue next transmission
    //! \return True if message should be used
    //!
    virtual bool BuildData_Send(const mavlink_boundary_request_item_t &boundaryRequest, const MaceCore::ModuleCharacteristic &sender, mavlink_boundary_item_t &boundaryItem, MaceCore::ModuleCharacteristic &vehicleObj, ModuleBoundaryIdentifier &receiveQueueObj, ModuleBoundaryIdentifier &respondQueueObj);


    //!
    //! \brief Download Action - Receive an item and make request for next item.
    //!
    //! This action will NOT be used for the final action
    //! \param msg Item message received
    //! \param sender Module that sent message
    //! \param request Message to construct requesting next item
    //! \param moduleDownloadingTo Module that boundary is downloading to
    //! \param receiveQueueObj Object to set that will remove any queued transmission
    //! \param respondQueueObj Object to set to queue next transmission
    //! \return True if message should be used
    //!
    virtual bool BuildData_Send(const mavlink_boundary_item_t &boundaryItem, const MaceCore::ModuleCharacteristic &sender, mavlink_boundary_request_item_t &request, MaceCore::ModuleCharacteristic &vehicleObj, ModuleBoundaryIdentifier &receiveQueueObj, ModuleBoundaryIdentifier &respondQueueObj);


    //!
    //! \brief Download Action - Receive final item and configure ack
    //!
    //! This action will ONLY be used on last item
    //! \param msg Item message recieved
    //! \param sender Module that sent message
    //! \param ackBoundary Ack to construct to indicate the success/failure of boundary transmission
    //! \param finalList Final boundary downloaded to be returned to module
    //! \param moduleDownloadingTo Module boundary is being downloaded to
    //! \param queueObj Object to set that will remove any queued transmission
    //! \return True if message should be used
    //!
    virtual bool Construct_FinalObjectAndResponse(const mavlink_boundary_item_t &boundaryItem, const MaceCore::ModuleCharacteristic &sender, mavlink_boundary_ack_t &ackBoundary, ModuleBoundaryIdentifier &key, BoundaryItem::BoundaryList &finalList, MaceCore::ModuleCharacteristic &vehicleObj, ModuleBoundaryIdentifier &queueObj);


    //!
    //! \brief Upload Action - Receive final ACK and end the previous transmission
    //! \param msg Ack message received
    //! \param sender Module that sent message
    //! \param ack Ack code to return
    //! \param queueObj Object to set that will remove any queued transmission
    //! \return
    //!
    virtual bool Finish_Receive(const mavlink_boundary_ack_t &boundaryItem, const MaceCore::ModuleCharacteristic &sender, uint8_t & ack, ModuleBoundaryIdentifier &queueObj);




    virtual void Construct_ReliableBroadcast_Vector(const BoundaryNotificationData &data, const MaceCore::ModuleCharacteristic &sender, std::vector<mavlink_new_boundary_object_t> &vec, MaceCore::BoundaryIdentifierType &queue);


    //!
    //! \brief Notify Receive - Receive the notification of a new boundary
    //!
    //! When being notified of a boundary, a seperate message will be received for each vehicle.
    //! This function is to receive those and assemble them into a single BoundaryCharacterstic.
    //!
    //! A watchdog is kicked off to re-request if a full BoundaryCharacterstic isn't received.
    //!
    //! \param msg Message received indicating a new boundary was received
    //! \param sender Module that generated the boundary on the remote machine
    //! \param data Data to return to local instance when boundary is fully received
    //! \return True if boundary is fully received, false otherwise
    //!
    virtual bool Construct_FinalObjectAndResponse(const mavlink_new_boundary_object_t &msg, const MaceCore::ModuleCharacteristic &sender, mavlink_boundary_ack_t &ack, MaceCore::ModuleCharacteristic &module_from, MaceCore::ModuleCharacteristic &key, BoundaryNotificationData &data);

    virtual bool Finish_Receive(const mavlink_boundary_ack_t &boundaryItem, const MaceCore::ModuleCharacteristic &sender, uint8_t & ack, MaceCore::BoundaryIdentifierType &queueObj);

public:

    ControllerBoundary(const Controllers::IMessageNotifier<mavlink_message_t, MaceCore::ModuleCharacteristic> *cb, TransmitQueue *queue, int linkChan);


    //!
    //! \brief Request a boundary from a remote instance
    //! \param key Boundary identifer on remote
    //! \param sender Module making request
    //! \param target Target module that contains the boundary to download
    //!
    void RequestBoundary(const uint8_t &key, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target);


private:


    //!
    //! \brief Function to be called when the watchdog for receiving vehicles in a boundary expires
    //!
    //! This would indicates that messages where lost on transmission.
    //! \param pair Details on whom and what boundary was incomplete
    //!
    void BoundaryBuilderWatchdogExpired(const ModuleBoundaryIdentifier &pair);


public:

    void Send(const uint8_t &data, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target)
    {
        SendBoundaryHelper_RequestDownload::Send(data, sender, target);
    }

    void DistributeNewBoundary(const BoundaryNotificationData &data, const MaceCore::ModuleCharacteristic &sender, const std::vector<MaceCore::ModuleCharacteristic> &targets)
    {
        NewBoundaryNotification::BroadcastReliable(data, sender, targets);
    }

};

}


#endif // CONTROLLER_BOUNDARY_H
