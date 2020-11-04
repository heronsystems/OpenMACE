#ifndef MISSION_LIST_H
#define MISSION_LIST_H
#include <iostream>
#include <stdint.h>
#include <memory>
#include <vector>

#include "mission_key.h"

#include "data_generic_command_item/abstract_command_item.h"

#include "data/mission_execution_state.h"

#include "data/jsonconverter.h"

namespace MissionItem {

class MissionList : public JSONConverter
{
public:
    enum MissionListState{
        COMPLETE,
        INCOMPLETE
    };

    struct MissionListStatus{
        MissionListState state;
        std::vector<int> remainingItems;
    };

public:
    MissionList();
    MissionList(const unsigned int &targetID, const unsigned int &generatorID, const MISSIONTYPE &missionType, const MISSIONSTATE &state);
    MissionList(const unsigned int &targetID, const unsigned int &generatorID, const MISSIONTYPE &missionType, const MISSIONSTATE &state, const size_t &size);
    MissionList(const unsigned int &targetID, const unsigned int &generatorID, const unsigned int &missionID, const MISSIONTYPE &missionType, const MISSIONSTATE &state, const size_t &size);
    MissionList(const MissionList &rhs);

public:
    void initializeQueue(const size_t &size);
    void clearQueue();
    void replaceMissionQueue(const std::vector<std::shared_ptr<command_item::AbstractCommandItem>> &newQueue);
    void insertMissionItem(const std::shared_ptr<command_item::AbstractCommandItem> missionItem);
    void replaceMissionItemAtIndex(const std::shared_ptr<command_item::AbstractCommandItem> missionItem, const unsigned int &index);

    std::shared_ptr<command_item::AbstractCommandItem> getMissionItem(const unsigned int &index) const;

    size_t getQueueSize() const;
    MissionListStatus getMissionListStatus() const;

    virtual QJsonObject toJSON(const int &vehicleID, const std::string &dataType) const;


public:

    MissionKey getMissionKey() const{
        return this->missionKey;
    }

    void setMissionKey(const MissionKey &key){
        this->missionKey = key;
    }

    void setVehicleID(const unsigned int &vehicleID){
        this->missionKey.m_systemID = vehicleID;
    }

    unsigned int getVehicleID() const{
        return this->missionKey.m_systemID;
    }

    void setCreatorID(const unsigned int &creatorID){
        this->missionKey.m_creatorID = creatorID;
    }

    unsigned int getCreatorID() const {
        return this->missionKey.m_creatorID;
    }

    void setMissionID(const uint64_t &missionID){
        this->missionKey.m_missionID = missionID;
    }

    uint64_t getMissionID() const{
        return this->missionKey.m_missionID;
    }

    void setMissionType(const MISSIONTYPE &missionType){
        this->missionKey.m_missionType = missionType;
    }

    MISSIONTYPE getMissionType() const{
        return this->missionKey.m_missionType;
    }

    void setMissionTXState(const MISSIONSTATE &missionTypeState){
        this->missionKey.m_missionState = missionTypeState;
    }

    MISSIONSTATE getMissionTXState() const{
        return this->missionKey.m_missionState;
    }


    void setMissionExeState(const Data::MissionExecutionState &state){
        this->missionExeState = state;
    }

    Data::MissionExecutionState getMissionExeState() const{
        return missionExeState;
    }

    unsigned int getActiveIndex() const;

    command_item::AbstractCommandItemPtr getActiveMissionItem();

    void setActiveIndex(const unsigned int &activeIndex);

public:
    MissionList& operator = (const MissionList &rhs)
    {
        this->missionKey = rhs.missionKey;
        this->missionQueue = rhs.missionQueue;
        this->missionExeState = rhs.missionExeState;
        this->activeMissionItem = rhs.activeMissionItem;
        return *this;
    }

    bool operator == (const MissionList &rhs) const{
        if(this->missionKey != rhs.missionKey){
            return false;
        }
        if(this->missionQueue != rhs.missionQueue){
            return false;
        }
        if(this->missionExeState != rhs.missionExeState){
            return false;
        }
        if(this->activeMissionItem != rhs.activeMissionItem){
            return false;
        }
        return true;
    }

    bool operator != (const MissionList &rhs) const{
        return !(*this == rhs);
    }

private: 
    MissionKey missionKey;

    Data::MissionExecutionState missionExeState;

    //!
    //! \brief activeMissionItem
    //!
    unsigned int activeMissionItem;

public:
    friend std::ostream& operator<<(std::ostream& os, const MissionList& t);

    std::vector<std::shared_ptr<command_item::AbstractCommandItem>> missionQueue;
};

} //end of namespace MissionItem
#endif // MISSION_LIST_H
