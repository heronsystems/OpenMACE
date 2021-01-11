#include "mission_list.h"

#include <exception>

namespace MissionItem {

MissionList::MissionList() :
    missionKey(0,0,0,MISSIONTYPE::AUTO,MISSIONSTATE::CURRENT),missionExeState(Data::MissionExecutionState::MESTATE_UNEXECUTED),activeMissionItem(0)
{

}

MissionList::MissionList(const unsigned int &targetID, const unsigned int &generatorID, const MISSIONTYPE &missionType, const MISSIONSTATE &state) :
    missionKey(targetID,generatorID,0,missionType,state),missionExeState(Data::MissionExecutionState::MESTATE_UNEXECUTED),activeMissionItem(0)
{

}

MissionList::MissionList(const unsigned int &targetID, const unsigned int &generatorID, const MISSIONTYPE &missionType, const MISSIONSTATE &state, const size_t &size) :
    missionKey(targetID,generatorID,0,missionType,state),missionExeState(Data::MissionExecutionState::MESTATE_UNEXECUTED),activeMissionItem(0)
{
    initializeQueue(size);
}

MissionList::MissionList(const unsigned int &targetID, const unsigned int &generatorID, const unsigned int &missionID, const MISSIONTYPE &missionType, const MISSIONSTATE &state, const size_t &size) :
    missionKey(targetID,generatorID,missionID,missionType,state),missionExeState(Data::MissionExecutionState::MESTATE_UNEXECUTED),activeMissionItem(0)
{
    initializeQueue(size);
}

MissionList::MissionList(const MissionList &rhs)
{
    this->missionKey = rhs.missionKey;
    this->missionQueue = rhs.missionQueue;
    this->missionExeState = rhs.missionExeState;
    this->activeMissionItem = rhs.activeMissionItem;
}

void MissionList::initializeQueue(const size_t &size)
{
    missionQueue.clear();

    if(size > 0){
        std::vector<std::shared_ptr<command_item::AbstractCommandItem>> tmpVector(size,nullptr);
        missionQueue = tmpVector;
    }
}

void MissionList::clearQueue()
{
    missionQueue.clear();
}

void MissionList::replaceMissionQueue(const std::vector<std::shared_ptr<command_item::AbstractCommandItem>> &newQueue)
{
    missionQueue.clear();
    missionQueue = newQueue;
}

MissionList::MissionListStatus MissionList::getMissionListStatus() const
{
    std::vector<int> nullItems;
    MissionListState missionState = MissionListState::COMPLETE;

    int index = 0;
    for(std::vector<std::shared_ptr<command_item::AbstractCommandItem>>::const_iterator it = missionQueue.begin(); it != missionQueue.end(); ++it) {
        if(!*it)
        {
            //This should see that the value is null
            nullItems.push_back(index);
            missionState = MissionListState::INCOMPLETE;
        }
        index++;
    }

    MissionListStatus missionStatus;
    missionStatus.state = missionState;
    missionStatus.remainingItems = nullItems;

    return missionStatus;
}

void MissionList::insertMissionItem(const std::shared_ptr<command_item::AbstractCommandItem> missionItem)
{
    missionQueue.push_back(missionItem->getClone());
}

void MissionList::replaceMissionItemAtIndex(const std::shared_ptr<command_item::AbstractCommandItem> missionItem, const unsigned int &index)
{
    missionQueue[index] = missionItem;
}

std::shared_ptr<command_item::AbstractCommandItem> MissionList::getMissionItem(const unsigned int &index) const
{
    return missionQueue[index];
}

size_t MissionList::getQueueSize() const
{
    return missionQueue.size();
}

unsigned int MissionList::getActiveIndex() const
{
    return activeMissionItem;
}

command_item::AbstractCommandItemPtr MissionList::getActiveMissionItem()
{
    return (getMissionItem(getActiveIndex()));
}

void MissionList::setActiveIndex(const unsigned int &activeIndex)
{
    activeMissionItem = activeIndex;
}

QJsonObject MissionList::toJSON(const int &vehicleID, const std::string &dataType) const
{
    QJsonObject json = toJSON_base(vehicleID, dataType);
    json["creatorID"] = static_cast<int>(getCreatorID());
    json["missionID"] = static_cast<int>(getMissionID());
    json["missionState"] = QString::fromStdString(Data::MissionExecutionStateToString(getMissionExeState()));
    json["missionType"] = QString::fromStdString(MissionItem::MissionTypeToString(getMissionType()));
    return json;
}

//!
//! \brief missionListToJSON Convert a mission list to a JSON array
//! \param list Mission list to convert to a JSON array
//! \param missionItems JSON Container for converted mission items
//!

std::ostream& operator<<(std::ostream& os, const MissionList& t)
{
    std::stringstream stream;
    stream.precision(6);
    stream << std::fixed << "Mission List Key" << t.getMissionKey()
           <<", Size: " << std::to_string(t.getQueueSize()) << ".";
    os << stream.str();

    return os;
}


}//end of namespace MissionItem
