#include "controller_boundary.h"

#include <algorithm>

namespace ExternalLink{


    //!
    //! \brief Download Action - Initiate download with a request list message
    //! \param data Data given to send.
    //! \param sender Module sending data
    //! \param target Module targeting
    //! \param cmd Message to contruct
    //! \param queueObj Queue object to contruct identifitying this transmission
    //! \return True if transmission should continue
    //!
    bool ControllerBoundary::Construct_Send(const uint8_t &data, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mavlink_boundary_request_list_t &cmd, ModuleBoundaryIdentifier &queueObj)
    {
        queueObj = ModuleBoundaryIdentifier(target, data);;

        cmd.boundary_host_sysid = target.MaceInstance;
        cmd.boundary_host_compid = target.ModuleID;
        cmd.boundary_identifier = data;

        if(m_BoundariesBeingFetching.find(queueObj) != m_BoundariesBeingFetching.cend())
        {
            throw std::runtime_error("Boundary is already being downloaded");
            return false;
        }

        BoundaryItem::BoundaryList newList;
        newList.clearQueue();
        m_BoundariesBeingFetching.insert({queueObj, std::make_tuple(sender, newList)});

        std::cout << "Boundary Controller: Sending Boundary Request List" << std::endl;

        return true;
    }


    //!
    //! \brief Upload Action - Receive request list and respond with count
    //! \param cmd Message received asking for count of a boundary
    //! \param sender Module that sent request
    //! \param rtn Message to construct containing count
    //! \param moduleUploadingFrom Module to contruct that contains the boundary to upload
    //! \param respondQueueObj Object to to construct that indicates how the message should be queued
    //! \return True if transmission should continue
    //!
    bool ControllerBoundary::IntermediateUnsolicitedReceive(const mavlink_boundary_request_list_t &cmd, const MaceCore::ModuleCharacteristic &sender, mavlink_boundary_count_t &rtn, MaceCore::ModuleCharacteristic &moduleUploadingFrom, ModuleBoundaryIdentifier &respondQueueObj)
    {
        /// Set up the object to identity who is receiving the boundary
        ModuleBoundaryIdentifier uniqueUploadIdentifier = ModuleBoundaryIdentifier(sender, cmd.boundary_identifier);

        /// Establish the module that has the boundary
        moduleUploadingFrom.MaceInstance = cmd.boundary_host_sysid;
        moduleUploadingFrom.ModuleID = cmd.boundary_host_compid;

        /// Set up the object to identify the boundary on the local machine
        ModuleBoundaryIdentifier uniqueBoundaryIdentifier = ModuleBoundaryIdentifier(moduleUploadingFrom, cmd.boundary_identifier);


        /// Ask for the boundary and make sure it all makes sense
        std::vector<std::tuple<ModuleBoundaryIdentifier, BoundaryItem::BoundaryList>> boundaries;
        Controllers::DataItem<ModuleBoundaryIdentifier, BoundaryItem::BoundaryList>::FetchDataFromKey(uniqueBoundaryIdentifier, boundaries);
        if(boundaries.size() == 0)
        {
            printf("ERROR!  No Boundary found for given key\n");
            return false;
        }
        if(boundaries.size() > 1)
        {
            throw std::runtime_error("Multiple boundaries assigned to the same key returned, This is a non-op");
        }
        if(std::get<0>(boundaries.at(0)).BoundaryIdentifier() != cmd.boundary_identifier)
        {
            throw std::runtime_error("Requesting a specific boundary key did not return the same key, This is a non-op");
        }


        // set up the receiver and establish queue objects
        respondQueueObj = uniqueUploadIdentifier;


        // Add boundaries upload. If already added then must be a retransmitt from a lost packet
        if(m_BoundariesUploading.find(uniqueUploadIdentifier) == m_BoundariesUploading.cend())
        {
            BoundaryItem::BoundaryList boundary = std::get<1>(boundaries.at(0));
            m_BoundariesUploading.insert({uniqueUploadIdentifier, boundary});
        }


        //construct packet
        rtn.count = m_BoundariesUploading.at(uniqueUploadIdentifier).getQueueSize();
        rtn.boundary_host_sysid = cmd.boundary_host_sysid;
        rtn.boundary_host_compid = cmd.boundary_host_compid;
        rtn.boundary_identifier = cmd.boundary_identifier;


        std::cout << "Boundary Controller: Sending Boundary Count" << std::endl;
        return true;
    }


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
    bool ControllerBoundary::BuildData_Send(const mavlink_boundary_count_t &msg, const MaceCore::ModuleCharacteristic &sender, mavlink_boundary_request_item_t &rtn, MaceCore::ModuleCharacteristic &moduleDownloadingTo, ModuleBoundaryIdentifier &receiveQueueObj, ModuleBoundaryIdentifier &respondQueueObj)
    {
        /// Set up the object to identity the boundary on the remote instance
        ModuleBoundaryIdentifier downloadFromUniqueIdentifier = ModuleBoundaryIdentifier(sender, msg.boundary_identifier);

        /// set up the receiver and establish queue objects based on remote boundary identifier
        receiveQueueObj = downloadFromUniqueIdentifier;
        respondQueueObj = downloadFromUniqueIdentifier;


        // check if boundary is being downloaded and inizialize the boundary
        if(m_BoundariesBeingFetching.find(downloadFromUniqueIdentifier) == m_BoundariesBeingFetching.cend())
        {
            return false;
        }

        // Establish the module that is requesting boundary (so other side knows where to send response to)
        moduleDownloadingTo = std::get<0>(m_BoundariesBeingFetching.at(downloadFromUniqueIdentifier));

        std::get<1>(m_BoundariesBeingFetching.at(downloadFromUniqueIdentifier)).initializeBoundary(msg.count);


        // establish return message
        rtn.boundary_host_sysid = msg.boundary_host_sysid;
        rtn.boundary_host_compid = msg.boundary_host_compid;
        rtn.boundary_identifier = msg.boundary_identifier;
        rtn.seq = 0;

        std::cout << "Boundary Controller: Requesting Item " << 0 << std::endl;

        return true;
    }


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
    bool ControllerBoundary::BuildData_Send(const mavlink_boundary_request_item_t &msg, const MaceCore::ModuleCharacteristic &sender, mavlink_boundary_item_t &boundaryItem, MaceCore::ModuleCharacteristic &moduleUploadingFrom, ModuleBoundaryIdentifier &receiveQueueObj, ModuleBoundaryIdentifier &respondQueueObj)
    {
        // Set up the object to identity who is receiving the boundary
        ModuleBoundaryIdentifier pair = ModuleBoundaryIdentifier(sender, msg.boundary_identifier);

        // Establish the module that has the boundary
        moduleUploadingFrom.MaceInstance = msg.boundary_host_sysid;
        moduleUploadingFrom.ModuleID = msg.boundary_host_compid;

        // set up the receiver and establish queue objects
        receiveQueueObj = pair;
        respondQueueObj = pair;


        //check that the given boundary has been initiated for an upload
        if(m_BoundariesUploading.find(pair) == m_BoundariesUploading.cend())
        {
            printf("ERROR!!!!!!!!!!!!\n  -- Boundary Controller was asked to send a item to a boundary it doesn't have knowledge of\n");
//            if(CONTROLLER_BOUNDARY_TYPE::mLog)
//                CONTROLLER_BOUNDARY_TYPE::mLog->error("BoundaryController_ExternalLink has been told to transmit a boundary item from a boundary which keys dont match the contained.");
            return false;
        }


        //pull index we need to fetch
        uint index = msg.seq;

        //ensure the requested index is within the expected range
        if(index >= m_BoundariesUploading[pair].getQueueSize())
        {
            printf("ERROR!!!!!!!!!!!!\n  -- Boundary Controller was asked to send a item whoose sequence is larger than what the boundary it knows about has\n");
//            if(CONTROLLER_BOUNDARY_TYPE::mLog)
//                CONTROLLER_BOUNDARY_TYPE::mLog->error("BoundaryController_ExternalLink has been told to transmit a boundary item with index " + std::to_string(index) + " which is greater than the size of the list contained.");
            return false;
        }

//        if(CONTROLLER_BOUNDARY_TYPE::mLog)
//            CONTROLLER_BOUNDARY_TYPE::mLog->info("BoundaryController_ExternalLink has been told to transmit a boundary item with index " + std::to_string(index) + ".");


        //construct return message
        mace::pose::CartesianPosition_2D ptrItem = this->m_BoundariesUploading[pair].getBoundaryItemAtIndex(index);
        boundaryItem.boundary_host_sysid = msg.boundary_host_sysid;
        boundaryItem.boundary_host_compid = msg.boundary_host_compid;
        boundaryItem.boundary_identifier = msg.boundary_identifier;
        boundaryItem.frame = MAV_FRAME_LOCAL_ENU;
        boundaryItem.seq = index;
        boundaryItem.x = ptrItem.getXPosition();
        boundaryItem.y = ptrItem.getYPosition();
        boundaryItem.z = 0.0;

        std::cout << "Boundary Controller: Sending Item " << index << std::endl;
        return true;
    }


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
    bool ControllerBoundary::BuildData_Send(const mavlink_boundary_item_t &msg, const MaceCore::ModuleCharacteristic &sender, mavlink_boundary_request_item_t &request, MaceCore::ModuleCharacteristic &moduleDownloadingTo, ModuleBoundaryIdentifier &receiveQueueObj, ModuleBoundaryIdentifier &respondQueueObj)
    {
        // Set up the object to identity who is receiving the boundary
        ModuleBoundaryIdentifier pair = ModuleBoundaryIdentifier(sender, msg.boundary_identifier);

        // set up the receiver and establish queue objects
        receiveQueueObj = pair;
        respondQueueObj = pair;

        //check if boundary item received is part of a boundary we are activly downloading
        if(this->m_BoundariesBeingFetching.find(pair) == m_BoundariesBeingFetching.cend())
        {
            printf("ERROR!!!!!!!!!\n  --Not Activly downloading boundary for key received\n");
//            if(CONTROLLER_BOUNDARY_TYPE::mLog)
//                CONTROLLER_BOUNDARY_TYPE::mLog->error("Boundary controller received a boundary item with a key that is not equal to the one we were originally told.");
            return false;
        }


        //check that the sequence received is less than the previous COUNT message indicated earlier
        uint seqReceived = msg.seq;
        if(seqReceived > (std::get<1>(m_BoundariesBeingFetching[pair]).getQueueSize() - 1)) //this should never happen
        {
            std::cout << "Boundary download Error: received a boundary item with an index greater than available in the queue" << std::endl;
//            if(CONTROLLER_BOUNDARY_TYPE::mLog)
//                CONTROLLER_BOUNDARY_TYPE::mLog->error("Boundary controller received a boundary item with an index greater than available in the queue.");
            return false;
        }

        // Establish the module that is requesting boundary (so other side knows where to send response to)
        moduleDownloadingTo = std::get<0>(m_BoundariesBeingFetching.at(pair));

        //execution will only continue if not last item
        //If it is the last item then it should go to another action
        if(seqReceived == (std::get<1>(m_BoundariesBeingFetching[pair]).getQueueSize() - 1))
        {
            return false;
        }

        CartesianPosition_2D newVertex;
        newVertex.setXPosition(msg.x);
        newVertex.setYPosition(msg.y);

        std::get<1>(m_BoundariesBeingFetching[pair]).replaceVertexItemAtIndex(&newVertex, seqReceived);

        BoundaryItem::BoundaryList::BoundaryListStatus status = std::get<1>(m_BoundariesBeingFetching[pair]).getBoundaryListStatus();
        if(status.state == BoundaryItem::BoundaryList::COMPLETE)
        {
            throw std::runtime_error("Still have more items to request, but boundary is full");
        }


        int indexRequest = status.remainingItems.at(0);

        request.boundary_host_sysid = msg.boundary_host_sysid;
        request.boundary_host_compid = msg.boundary_host_compid;
        request.boundary_identifier = msg.boundary_identifier;
        request.seq = indexRequest;

        std::cout << "Boundary Controller: Requesting Item " << indexRequest << std::endl;
        return true;
    }


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
    bool ControllerBoundary::Construct_FinalObjectAndResponse(const mavlink_boundary_item_t &msg, const MaceCore::ModuleCharacteristic &sender, mavlink_boundary_ack_t &ackBoundary, ModuleBoundaryIdentifier &key, BoundaryItem::BoundaryList &finalList, MaceCore::ModuleCharacteristic &moduleDownloadingTo, ModuleBoundaryIdentifier &queueObj)
    {

        // Set up the object to identity who is receiving the boundary
        ModuleBoundaryIdentifier pair = ModuleBoundaryIdentifier(sender, msg.boundary_identifier);

        // set queue so previous transmission can be removed
        queueObj = pair;

        //check if boundary item received is part of a boundary we are activly downloading
        if(this->m_BoundariesBeingFetching.find(pair) == m_BoundariesBeingFetching.cend())
        {
            printf("ERROR!!!!!!!!!\n  --Not Activly downloading boundary for key received\n");
//            if(CONTROLLER_BOUNDARY_TYPE::mLog)
//                CONTROLLER_BOUNDARY_TYPE::mLog->error("Boundary controller received a boundary item with a key that is not equal to the one we were originally told.");
            return false;
        }


        //check that the sequence received is less than the previous COUNT message indicated earlier
        uint seqReceived = msg.seq;
        if(seqReceived > (std::get<1>(m_BoundariesBeingFetching[pair]).getQueueSize() - 1)) //this should never happen
        {
            std::cout << "Boundary download Error: received a boundary item with an index greater than available in the queue" << std::endl;
//            if(CONTROLLER_BOUNDARY_TYPE::mLog)
//                CONTROLLER_BOUNDARY_TYPE::mLog->error("Boundary controller received a boundary item with an index greater than available in the queue.");
            return false;
        }

        // Establish the module that is requesting boundary (so other side knows where to send response to)
        moduleDownloadingTo = std::get<0>(m_BoundariesBeingFetching.at(pair));

        //execution will only continue if last item
        if(seqReceived < (std::get<1>(m_BoundariesBeingFetching[pair]).getQueueSize() - 1))
        {
            return false;
        }

        CartesianPosition_2D newVertex;
        newVertex.setXPosition(msg.x);
        newVertex.setYPosition(msg.y);

        std::get<1>(m_BoundariesBeingFetching[pair]).replaceVertexItemAtIndex(&newVertex, seqReceived);

        BoundaryItem::BoundaryList::BoundaryListStatus status = std::get<1>(m_BoundariesBeingFetching[pair]).getBoundaryListStatus();
        if(status.state == BoundaryItem::BoundaryList::INCOMPLETE)
        {
            throw std::runtime_error("Reached end of request but boundaries are not completed");
        }

//        if(CONTROLLER_BOUNDARY_TYPE::mLog)
//        {
//            CONTROLLER_BOUNDARY_TYPE::mLog->info("Boundary Controller has received the entire boundary");
//        }

        ackBoundary.boundary_host_sysid = msg.boundary_host_sysid;
        ackBoundary.boundary_host_compid = msg.boundary_host_compid;
        ackBoundary.boundary_identifier = msg.boundary_identifier;
        ackBoundary.boundary_result = BOUNDARY_ACCEPTED;

        key = pair;
        finalList = std::get<1>(m_BoundariesBeingFetching[pair]);
        m_BoundariesBeingFetching.erase(pair);

        std::cout << "Boundary Controller: Sending Final ACK" << std::endl;

        return true;
    }


    //!
    //! \brief Upload Action - Receive final ACK and end the previous transmission
    //! \param msg Ack message received
    //! \param sender Module that sent message
    //! \param ack Ack code to return
    //! \param queueObj Object to set that will remove any queued transmission
    //! \return
    //!
    bool ControllerBoundary::Finish_Receive(const mavlink_boundary_ack_t &msg, const MaceCore::ModuleCharacteristic &sender, uint8_t & ack, ModuleBoundaryIdentifier &queueObj)
    {
        // Set up the object to identity who is receiving the boundary
        ModuleBoundaryIdentifier pair = ModuleBoundaryIdentifier(sender, msg.boundary_identifier);

        // set queue so previous transmission can be removed
        queueObj = pair;

        ack = msg.boundary_result;

        if(m_BoundariesUploading.find(pair) != m_BoundariesUploading.cend())
        {
            printf("Boundary Controller - Done uploading boundary\n");
            m_BoundariesUploading.erase(pair);
            return true;
        }

        return false;
    }


    void ControllerBoundary::Construct_ReliableBroadcast_Vector(const BoundaryNotificationData &data, const MaceCore::ModuleCharacteristic &sender, std::vector<mavlink_new_boundary_object_t> &vec, MaceCore::BoundaryIdentifierType &queue)
    {
        queue = data.uniqueIdentifier;

        std::vector<int> vehicles = data.characteristic.List();
        if(vehicles.size() == 0)
        {
            mavlink_new_boundary_object_t msg;
            msg.boundary_host_sysid = sender.MaceInstance;
            msg.boundary_host_compid = sender.ModuleID;
            msg.boundary_type = (uint8_t)data.characteristic.Type();
            msg.boundary_identifier = data.uniqueIdentifier;
            msg.vehicle_aplicable = 0;
            msg.num_vehicles = 0;

            vec.push_back(msg);
        }



        for(auto it = vehicles.cbegin() ; it != vehicles.cend() ; ++it)
        {
            mavlink_new_boundary_object_t msg;
            msg.boundary_host_sysid = sender.MaceInstance;
            msg.boundary_host_compid = sender.ModuleID;
            msg.boundary_type = (uint8_t)data.characteristic.Type();
            msg.boundary_identifier = data.uniqueIdentifier;
            msg.vehicle_aplicable = *it;
            msg.num_vehicles = vehicles.size();

            vec.push_back(msg);
        }
    }




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
    bool ControllerBoundary::Construct_FinalObjectAndResponse(const mavlink_new_boundary_object_t &msg, const MaceCore::ModuleCharacteristic &sender, mavlink_boundary_ack_t &ack, MaceCore::ModuleCharacteristic &module_from, MaceCore::ModuleCharacteristic &key, BoundaryNotificationData &data)
    {
        module_from = this->GetHostKey();

        /////////
        /// A global boundary. No need to wait for any other messages
        /////////
        if(msg.num_vehicles == 0)
        {
            key = sender;
            data.characteristic = BoundaryItem::BoundaryCharacterisic((BoundaryItem::BOUNDARYTYPE)msg.boundary_type);
            data.uniqueIdentifier = msg.boundary_identifier;

            ack.boundary_result = 0;
            ack.boundary_identifier = msg.boundary_identifier;
            return true;
        }


        /////////
        /// A boundary with one vehicle. No need to wait for any other messages
        /////////
        if(msg.num_vehicles == 1)
        {
            key = sender;
            data.characteristic = BoundaryItem::BoundaryCharacterisic(msg.vehicle_aplicable, (BoundaryItem::BOUNDARYTYPE)msg.boundary_type);
            data.uniqueIdentifier = msg.boundary_identifier;

            ack.boundary_result = 0;
            ack.boundary_identifier = msg.boundary_identifier;
            return true;
        }


        /////////
        /// A boundary with 2+ vehicles. Must wait for additional messages and set up failsafes for lost messages
        /////////

        //Setup variables to host the boundary being constructed.
        //Setup watchdog to issue retransmissions if needed
        ModuleBoundaryIdentifier pair = ModuleBoundaryIdentifier(sender, msg.boundary_identifier);
        if(m_VehiclesInBoundary.find(pair) == m_VehiclesInBoundary.cend())
        {
            m_VehiclesInBoundary.insert({pair, {}});
            //m_BoundaryBuilderWatchdogs.insert({pair, std::make_shared<Watchdog>(std::chrono::seconds(2), [this, pair](){ BoundaryBuilderWatchdogExpired(pair); })});
        }

        //Update with known vehicle if new
        //Kick the dog, so we don't timeout waiting for the other mavlink_new_boundary_object_t messages
        if(std::find(m_VehiclesInBoundary.at(pair).begin(), m_VehiclesInBoundary.at(pair).end(), msg.vehicle_aplicable) == m_VehiclesInBoundary.at(pair).end()) {
            m_VehiclesInBoundary[pair].push_back(msg.vehicle_aplicable);
            //m_BoundaryBuilderWatchdogs.at(pair)->Kick();
        }

        //if total number of vehicles received then return indicating we are done
        if(m_VehiclesInBoundary.size() == msg.num_vehicles)
        {
            //m_BoundaryBuilderWatchdogs.erase(pair);
            key = sender;
            data.characteristic = BoundaryItem::BoundaryCharacterisic(m_VehiclesInBoundary.at(pair), (BoundaryItem::BOUNDARYTYPE)msg.boundary_type);
            data.uniqueIdentifier = msg.boundary_identifier;

            ack.boundary_result = 0;
            ack.boundary_identifier = msg.boundary_identifier;
            return true;
        }

        //otherwise not done so return false such that the action doesn't invoke a return.
        return false;
    }


    bool ControllerBoundary::Finish_Receive(const mavlink_boundary_ack_t &msg, const MaceCore::ModuleCharacteristic &sender, uint8_t & ack, MaceCore::BoundaryIdentifierType &queueObj)
    {
        // Set up the object to identity who is receiving the boundary
        ModuleBoundaryIdentifier pair(sender, msg.boundary_identifier);

        // set queue so previous transmission can be removed
        queueObj = msg.boundary_identifier;

        ack = msg.boundary_result;

        if(m_BoundariesUploading.find(pair) == m_BoundariesUploading.cend())
        {
            printf("Bounary Controller - Done notifying remote instance of new boundary\n");
            return true;
        }

        return false;
    }



    ControllerBoundary::ControllerBoundary(const Controllers::IMessageNotifier<mavlink_message_t, MaceCore::ModuleCharacteristic> *cb, TransmitQueue *queue, int linkChan) :
        CONTROLLER_BOUNDARY_TYPE(cb, queue, linkChan),
        SendBoundaryHelper_RequestDownload(this, ModuleToSysIDCompIDConverter<mavlink_boundary_request_list_t>(mavlink_msg_boundary_request_list_encode_chan)),
        BoundaryControllerAction_ReceiveUnsolicitedRequestList_SendCount(this, mavlink_msg_boundary_request_list_decode, ModuleToSysIDCompIDConverter<mavlink_boundary_count_t>(mavlink_msg_boundary_count_encode_chan)),
        SendBoundaryHelper_ReceiveCountRespondItemRequest(this, mavlink_msg_boundary_count_decode, ModuleToSysIDCompIDConverter<mavlink_boundary_request_item_t>(mavlink_msg_boundary_request_item_encode_chan)),
        SendBoundaryHelper_RequestItem(this, mavlink_msg_boundary_request_item_decode, ModuleToSysIDCompIDConverter<mavlink_boundary_item_t>(mavlink_msg_boundary_item_encode_chan)),
        SendBoundaryHelper_ReceiveItem(this,
                                       [this](const mavlink_boundary_request_item_t &A, const MaceCore::ModuleCharacteristic &B, const ModuleBoundaryIdentifier &C, const MaceCore::ModuleCharacteristic &D){SendBoundaryHelper_ReceiveCountRespondItemRequest::NextTransmission(A,B,C,D);},
                                        mavlink_msg_boundary_item_decode),
        SendBoundaryHelper_Final(this, mavlink_msg_boundary_item_decode, ModuleToSysIDCompIDConverter<mavlink_boundary_ack_t>(mavlink_msg_boundary_ack_encode_chan)),
        SendBoundaryHelper_FinalFinal(this, mavlink_msg_boundary_ack_decode),

        NewBoundaryNotification(this, ModuleToSysIDCompIDConverter<mavlink_new_boundary_object_t>(mavlink_msg_new_boundary_object_encode_chan)),
        UsolicitedReceiveNewBoundaryNotification(this, mavlink_msg_new_boundary_object_decode, ModuleToSysIDCompIDConverter<mavlink_boundary_ack_t>(mavlink_msg_boundary_ack_encode_chan)),
        NewBoundaryNotification_AckReceive(this, mavlink_msg_boundary_ack_decode)
//        Action_RequestCurrentBoundary_Initiate(this, mavlink_msg_boundary_request_list_encode_chan),
//        Action_RequestCurrentBoundary_Response(this,
//                                [this](const mavlink_boundary_count_t &A, const MaceCore::ModuleCharacteristic &B, const BoundaryItem::BoundaryKey &C, const MaceCore::ModuleCharacteristic &D){SendHelper_RequestList::NextTransmission(A,B,C,D);},
//                                mavlink_msg_boundary_request_list_decode),
//        Action_RequestCurrentBoundary_NoBoundaryResponse(this,
//                                [this](const mavlink_boundary_ack_t &A, const MaceCore::ModuleCharacteristic &B, const BoundaryItem::BoundaryKey &C, const MaceCore::ModuleCharacteristic &D){SendHelper_Final::FinalResponse(A,B,C,D);},
//                                mavlink_msg_boundary_request_list_decode)
    {

    }


    //!
    //! \brief Request a boundary from a remote instance
    //! \param key Boundary identifer on remote
    //! \param sender Module making request
    //! \param target Target module that contains the boundary to download
    //!
    void ControllerBoundary::RequestBoundary(const uint8_t &key, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target)
    {
        SendBoundaryHelper_RequestDownload::Send(key, sender, target);
    }


    //!
    //! \brief Function to be called when the watchdog for receiving vehicles in a boundary expires
    //!
    //! This would indicates that messages where lost on transmission.
    //! \param pair Details on whom and what boundary was incomplete
    //!
    void ControllerBoundary::BoundaryBuilderWatchdogExpired(const ModuleBoundaryIdentifier &pair)
    {
        m_BoundaryBuilderWatchdogs.erase(pair);

        printf("ERROR!!! Boundary watchdog expired. All expected vehicles in a boundary where not received\n");
        throw std::runtime_error("Boundary watchdown expired. Perhaps some messages where lost?");
    }

}
