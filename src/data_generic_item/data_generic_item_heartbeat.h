#ifndef DATA_GENERIC_ITEM_HEARTBEAT_H
#define DATA_GENERIC_ITEM_HEARTBEAT_H

#include <iostream>
#include <string>
#include <stdint.h>

#include <mavlink.h>

#include "data/autopilot_types.h"
#include "data/comms_protocol.h"
#include "data/system_type.h"
#include "data/mission_execution_state.h"
#include "data/mace_hsm_state.h"

#include "data/jsonconverter.h"

namespace DataGenericItem {

class DataGenericItem_Heartbeat : JSONConverter
{
public:
    DataGenericItem_Heartbeat();

    DataGenericItem_Heartbeat(const DataGenericItem_Heartbeat &copyObj);


public:
    void setProtocol(const Data::CommsProtocol &protocol)
    {
        this->protocol = protocol;
    }
    void setType(const Data::SystemType &type)
    {
        this->type = type;
    }
    void setAutopilot(const Data::AutopilotType &autopilot)
    {
        this->autopilot = autopilot;
    }
    void setExecutionState(const Data::MissionExecutionState &state)
    {
        this->missionState = state;
    }

    void setCompanion(const bool &companion)
    {
        this->maceCompanion = companion;
    }

    void setMavlinkID(const uint8_t &ID)
    {
        this->mavlinkID = ID;
    }

    void setHSMState(const Data::MACEHSMState &currentState)
    {
        this->currentHSMState = currentState;
    }

public:
    Data::CommsProtocol getProtocol() const
    {
        return this->protocol;
    }
    Data::SystemType getType() const
    {
        return this->type;
    }
    Data::AutopilotType getAutopilot() const
    {
        return this->autopilot;
    }
    Data::MissionExecutionState getMissionState() const
    {
        return this->missionState;
    }
    bool getCompanion() const
    {
        return this->maceCompanion;
    }

    uint8_t getMavlinkID() const
    {
        return this->mavlinkID;
    }

    Data::MACEHSMState getHSMState() const
    {
        return this->currentHSMState;
    }

    mavlink_mace_heartbeat_t getMACECommsObject() const;
    mavlink_message_t getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const;
    virtual QJsonObject toJSON(const int &vehicleID, const std::string &dataType) const;

    virtual void fromJSON(const QJsonDocument &inputJSON) ;

    virtual std::string toCSV(const std::string &delimiter) const;

public:
    void operator = (const DataGenericItem_Heartbeat &rhs)
    {
        this->protocol = rhs.protocol;
        this->type = rhs.type;
        this->autopilot = rhs.autopilot;
        this->missionState = rhs.missionState;
        this->maceCompanion = rhs.maceCompanion;
        this->mavlinkID = rhs.mavlinkID;
        this->currentHSMState = rhs.currentHSMState;
    }

    bool operator == (const DataGenericItem_Heartbeat &rhs) {
        if(this->protocol != rhs.protocol){
            return false;
        }
        if(this->type != rhs.type){
            return false;
        }
        if(this->autopilot != rhs.autopilot){
            return false;
        }
        if(this->missionState != rhs.missionState){
            return false;
        }
        if(this->maceCompanion != rhs.maceCompanion){
            return false;
        }
        if(this->mavlinkID != rhs.mavlinkID) {
            return false;
        }
        if(this->currentHSMState != rhs.currentHSMState) {
            return false;
        }
        return true;
    }

    bool operator != (const DataGenericItem_Heartbeat &rhs) {
        return !(*this == rhs);
    }


protected:
    Data::AutopilotType autopilot;
    Data::CommsProtocol protocol;
    Data::SystemType type;
    Data::MissionExecutionState missionState;
    bool maceCompanion;
    uint8_t mavlinkID;
    Data::MACEHSMState currentHSMState;
};

} //end of namespace DataGenericItem

#endif // DATA_GENERIC_ITEM_HEARTBEAT_H
