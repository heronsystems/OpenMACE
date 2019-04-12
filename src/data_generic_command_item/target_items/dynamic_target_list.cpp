#include "dynamic_target_list.h"

#include <exception>

namespace TargetItem {

DynamicTargetList::DynamicTargetList()
{
    this->activeTargetItem = 0;
    this->targetList.clear();
}

DynamicTargetList::DynamicTargetList(const DynamicTargetList &rhs)
{
    this->activeTargetItem = rhs.activeTargetItem;
    this->targetList = rhs.targetList;
}

size_t DynamicTargetList::listSize() const
{
    return targetList.size();
}

void DynamicTargetList::clearList()
{
    targetList.clear();
}

void DynamicTargetList::appendDynamicTarget(const CartesianDynamicTarget &target, const DynamicTargetStorage::TargetCompletion &state)
{
    DynamicTargetStorage obj(target,state);
    targetList.push_back(obj);
}

void DynamicTargetList::removeTargetAtIndex(const unsigned int &index)
{
    std::list<DynamicTargetStorage>::iterator it = targetList.begin();
    std::advance(it,index);
    targetList.erase(it);
}

void DynamicTargetList::replaceTargetAtIndex(const unsigned int &index, const CartesianDynamicTarget &target, const DynamicTargetStorage::TargetCompletion &state)
{
    DynamicTargetStorage obj(target,state);
    std::list<DynamicTargetStorage>::iterator it = targetList.begin();
    std::advance(it,index);
    targetList.insert(it,obj);
}

void DynamicTargetList::spliceTargetListAtIndex(const unsigned int &index, const std::list<DynamicTargetStorage> &list)
{
    std::list<DynamicTargetStorage> listCopy = list;
    std::list<DynamicTargetStorage>::iterator it = targetList.begin();
    std::advance(it,index);
    targetList.splice(it,listCopy);
}

bool DynamicTargetList::isCompleted() const
{
    if(getNextIncomplete() != nullptr)
        return false;
    return true;
}

unsigned int DynamicTargetList::getActiveTargetItem() const
{
    return this->activeTargetItem;
}

const DynamicTargetStorage* DynamicTargetList::getTargetStorageAtIndex(const unsigned int &index) const
{
    std::list<DynamicTargetStorage>::const_iterator it = targetList.begin();
    std::advance(it,index);
    return &(*it);
}

CartesianDynamicTarget DynamicTargetList::getTargetAtIndex(const unsigned int &index) const
{
    std::list<DynamicTargetStorage>::const_iterator it = targetList.begin();
    std::advance(it,index);
    return *(*it).getDynamicTarget();
}

const CartesianDynamicTarget* DynamicTargetList::getTargetPointerAtIndex(const unsigned int &index) const
{
    std::list<DynamicTargetStorage>::const_iterator it = targetList.begin();
    std::advance(it,index);
    return (*it).getDynamicTarget();
}

const CartesianDynamicTarget* DynamicTargetList::getNextIncomplete() const
{
    std::list<DynamicTargetStorage>::const_iterator it = targetList.begin();
    for (; it != targetList.end(); ++it)
    {
        if((*it).getTargetState() == DynamicTargetStorage::TargetCompletion::INCOMPLETE)
            return (*it).getDynamicTarget();
    }
    return nullptr;
}


const CartesianDynamicTarget* DynamicTargetList::markCompletionState(const unsigned int &index, const DynamicTargetStorage::TargetCompletion &state)
{
    std::list<DynamicTargetStorage>::iterator it = targetList.begin();
    std::advance(it,index);
    (*it).setTargetState(state);

    for (; it != targetList.end(); ++it)
    {
        if((*it).getTargetState() == DynamicTargetStorage::TargetCompletion::INCOMPLETE)
            return (*it).getDynamicTarget();
        activeTargetItem++;
    }
    return nullptr;
}

}//end of namespace MissionItem
