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
    if(size <= 0){
        // TODO-Ken/Pat: Throw a message with exception
        std::cout << "Cannot initialize queue of 0" << std::endl;
        throw std::exception();
    }
    missionQueue.clear();
    std::vector<std::shared_ptr<command_item::AbstractCommandItem>> tmpVector(size,nullptr);
    missionQueue = tmpVector;
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
    missionQueue.push_back(missionItem);
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
    json["missionItems"] = missionListToJSON();
    return json;
}

//!
//! \brief missionListToJSON Convert a mission list to a JSON array
//! \param list Mission list to convert to a JSON array
//! \param missionItems JSON Container for converted mission items
//!
QJsonArray MissionList::missionListToJSON()
{
    QJsonArray missionItems;
    for(size_t i = 0; i < getQueueSize(); i++)
    {
        //TODO-PAT: Look into unique_ptr or auto_ptr?? Not sure I like this...
        std::shared_ptr<command_item::AbstractCommandItem> missionItem = getMissionItem(i);

        QJsonObject obj;
        obj["description"] = QString::fromStdString(missionItem->getDescription());
        obj["type"] = QString::fromStdString(command_item::CommandItemToString(missionItem->getCommandType()));

        switch (missionItem->getCommandType()) {
        case command_item::COMMANDTYPE::CI_ACT_ARM:
        {
            std::shared_ptr<command_item::ActionArm> castItem = std::dynamic_pointer_cast<command_item::ActionArm>(missionItem);
            obj["positionalFrame"] = "global";
            UNUSED(castItem);
            break;
        }
        case command_item::COMMANDTYPE::CI_ACT_CHANGEMODE:
        {
            std::shared_ptr<command_item::ActionChangeMode> castItem = std::dynamic_pointer_cast<command_item::ActionChangeMode>(missionItem);
            obj["positionalFrame"] = "global";
            UNUSED(castItem);
            break;
        }
        case command_item::COMMANDTYPE::CI_NAV_LAND:
        {
            std::shared_ptr<command_item::SpatialLand> castItem = std::dynamic_pointer_cast<command_item::SpatialLand>(missionItem);
            obj["positionalFrame"] = "global";
            UNUSED(castItem);

            break;
        }
        case command_item::COMMANDTYPE::CI_NAV_RETURN_TO_LAUNCH:
        {
            std::shared_ptr<command_item::SpatialRTL> castItem = std::dynamic_pointer_cast<command_item::SpatialRTL>(missionItem);
            obj["positionalFrame"] = "global";
            UNUSED(castItem);
            break;
        }
        case command_item::COMMANDTYPE::CI_NAV_TAKEOFF:
        {
            std::shared_ptr<command_item::SpatialTakeoff> castItem = std::dynamic_pointer_cast<command_item::SpatialTakeoff>(missionItem);
            if(castItem->position->getCoordinateSystemType() == CoordinateSystemTypes::GEODETIC)
            {
                obj["positionalFrame"] = "global";
                if(castItem->position->is3D())
                {
                    const mace::pose::GeodeticPosition_3D* castPosition = castItem->position->positionAs<mace::pose::GeodeticPosition_3D>();
                    obj["lat"] = castPosition->getLatitude();
                    obj["lng"] = castPosition->getLongitude();
                    obj["alt"] = castPosition->getAltitude();
                }
            }

            break;
        }
        case command_item::COMMANDTYPE::CI_NAV_WAYPOINT:
        {
            std::shared_ptr<command_item::SpatialWaypoint> castItem = std::dynamic_pointer_cast<command_item::SpatialWaypoint>(missionItem);
            obj["positionalFrame"] = "global";
            castItem->getPosition()->updateQJSONObject(obj);
            break;
        }
        case command_item::COMMANDTYPE::CI_NAV_LOITER_TIME:
        {
            std::shared_ptr<command_item::SpatialLoiter_Time> castItem = std::dynamic_pointer_cast<command_item::SpatialLoiter_Time>(missionItem);
            obj["positionalFrame"] = "global";
            //            obj["lat"] = castItem->position->getX();
            //            obj["lng"] = castItem->position->getY();
            //            obj["alt"] = castItem->position->getZ();
            obj["duration"] = castItem->duration;
            if(castItem->direction == Data::LoiterDirection::CW)
            {
                obj["radius"] = castItem->radius;
            }else{
                obj["radius"] = 0-castItem->radius;
            }
            break;
        }
        case command_item::COMMANDTYPE::CI_NAV_LOITER_TURNS:
        {
            std::shared_ptr<command_item::SpatialLoiter_Turns> castItem = std::dynamic_pointer_cast<command_item::SpatialLoiter_Turns>(missionItem);
            obj["positionalFrame"] = "global";
            //            obj["lat"] = castItem->position->getX();
            //            obj["lng"] = castItem->position->getY();
            //            obj["alt"] = castItem->position->getZ();
            obj["turns"] = static_cast<int>(castItem->turns);
            if(castItem->direction == Data::LoiterDirection::CW)
            {
                obj["radius"] = castItem->radius;
            }else{
                obj["radius"] = 0-castItem->radius;
            }
            break;
        }
        case command_item::COMMANDTYPE::CI_NAV_LOITER_UNLIM:
        {
            std::shared_ptr<command_item::SpatialLoiter_Unlimited> castItem = std::dynamic_pointer_cast<command_item::SpatialLoiter_Unlimited>(missionItem);
            obj["positionalFrame"] = "global";
            //            obj["lat"] = castItem->position->getX();
            //            obj["lng"] = castItem->position->getY();
            //            obj["alt"] = castItem->position->getZ();
            if(castItem->direction == Data::LoiterDirection::CW)
            {
                obj["radius"] = castItem->radius;
            }else{
                obj["radius"] = 0-castItem->radius;
            }
            break;
        }
        default:
            break;
        }

        missionItems.push_back(obj);
    }
    return missionItems;
}
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
