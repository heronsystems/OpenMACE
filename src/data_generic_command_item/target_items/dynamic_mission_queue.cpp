#include "dynamic_mission_queue.h"

namespace TargetItem {

DynamicMissionQueue::DynamicMissionQueue()
{

}

DynamicMissionQueue::DynamicMissionQueue(const MissionItem::MissionKey &key, const unsigned int &index):
    missionKey(key), describingMissionItem(index)
{

}

DynamicMissionQueue::DynamicMissionQueue(const DynamicMissionQueue &copy)
{
    this->describingMissionItem = copy.describingMissionItem;
    this->missionKey = copy.missionKey;
    this->m_TargetList = copy.m_TargetList;
}

DynamicMissionQueue::~DynamicMissionQueue()
{

}



} //end of namespace TargetItem
