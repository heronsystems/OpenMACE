#include "dynamic_mission_queue.h"

namespace TargetItem {

DynamicMissionQueue::DynamicMissionQueue()
{

}

DynamicMissionQueue::DynamicMissionQueue(const MissionItem::MissionKey &key, const unsigned int &index):
    m_missionKey(key), associatedMissionItem(index)
{

}

DynamicMissionQueue::DynamicMissionQueue(const DynamicMissionQueue &copy)
{
    this->associatedMissionItem = copy.associatedMissionItem;
    this->m_missionKey = copy.m_missionKey;
    this->m_TargetList = copy.m_TargetList;
}

DynamicMissionQueue::~DynamicMissionQueue()
{

}



} //end of namespace TargetItem
