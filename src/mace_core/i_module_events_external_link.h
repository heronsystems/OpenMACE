#ifndef I_MODULE_EVENTS_EXTERNAL_LINK_H
#define I_MODULE_EVENTS_EXTERNAL_LINK_H

#include "i_module_events_general.h"
#include "i_module_events_vehicle.h"
#include "i_module_events_general_vehicle.h"
#include "i_module_events_ai_support.h"

#include "i_module_events_boundary_generator.h"

namespace MaceCore
{
struct NewBoundaryData
{
public:
    NewBoundaryData(const ModuleCharacteristic &sender, const uint8_t remoteIdentifier, const BoundaryItem::BoundaryCharacterisic &characteistic) :
        Sender(sender),
        RemoteIdentifier(remoteIdentifier),
        Characteistic(characteistic)
    {

    }

    NewBoundaryData(const NewBoundaryData &that) :
        Sender(that.Sender),
        RemoteIdentifier(that.RemoteIdentifier),
        Characteistic(that.Characteistic)
    {

    }

    ModuleCharacteristic Sender;
    uint8_t RemoteIdentifier;
    BoundaryItem::BoundaryCharacterisic Characteistic;
};

class IModuleEventsExternalLink : virtual public IModuleEventsGeneral, virtual public IModuleEventsGeneralVehicle, virtual public IModuleEventsBoundaryGenerator, virtual public IModuleEvents_AISupport{

public:

    virtual void ExternalEvent_RequestingDataSync(const void *sender, const ModuleCharacteristic &module) = 0;
    //!
    //! \brief ExternalEvent_UpdateRemoteID
    //! \param sender
    //! \param remoteID
    //!
    virtual void ExternalEvent_UpdateRemoteID(const void *sender, const unsigned int &remoteID) = 0;


    //!
    //! \brief ExternalEvent_MissionACK
    //! \param sender
    //! \param missionACK
    //!
    virtual void ExternalEvent_MissionACK(const void* sender, const MissionItem::MissionACK &missionACK) = 0;

    //!
    //! \brief ExternalEvent_FinishedRXMissionList function event emitted by an external link module after the controller
    //! had completed receiving a mission. The type and state of the mission are contained in the list object. It will be
    //! up top the core to decypher this information and organize it in the appropriate place.
    //! \param sender pointer to the module emitting the event.
    //! \param missionList reference to the mission queue the module had received. This should be a complete mission
    //! without any holes or gaps in the queue.
    //!
    virtual void ExternalEvent_FinishedRXMissionList(const void *sender, const MissionItem::MissionList &missionList) = 0;

    //!
    //! \brief ExternalEvent_NewOnboardMission New onboard mission event
    //! \param sender Sender module
    //! \param mission New mission key
    //!
    virtual void ExternalEvent_NewOnboardMission(const ModuleBase *sender, const MissionItem::MissionKey &mission) = 0;


    virtual void ExternalEvent_NewBoundary(const ModuleBase *sender, const NewBoundaryData &data) = 0;

    //!
    //! \brief ExternalEvent_FinishedRXBoundaryList Event signaling the receipt of a boundary list
    //! \param sender Sender module
    //! \param boundaryList New boundary list
    //!
    virtual void ExternalEvent_FinishedRXBoundaryList(const void *sender, const BoundaryItem::BoundaryList &boundaryList) = 0;

};

} //End MaceCore Namespace

#endif // I_MODULE_EVENTS_EXTERNAL_LINK_H
