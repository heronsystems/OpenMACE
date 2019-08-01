#include "dynamic_target_list.h"

#include <exception>

namespace command_target {

DynamicTargetList::DynamicTargetList()
{
    this->m_TargetList.clear();

    this->activeTargetItem = 0;
}

DynamicTargetList::DynamicTargetList(const DynamicTargetList &rhs)
{
    this->m_TargetList = rhs.m_TargetList;

    this->activeTargetItem = rhs.activeTargetItem;
}

void DynamicTargetList::clearList()
{
    this->m_TargetList.clear();

    this->activeTargetItem = 0;
}

void DynamicTargetList::appendDynamicTarget(const DynamicTarget &target, const DynamicTargetState::TARGETSTATE &state)
{
    DynamicTargetState obj(target,state);
    m_TargetList.push_back(obj);
}

void DynamicTargetList::removeTargetAtIndex(const unsigned int &index)
{
    std::list<DynamicTargetState>::iterator it = m_TargetList.begin();
    std::advance(it,index);
    m_TargetList.erase(it);
}

void DynamicTargetList::replaceTargetAtIndex(const unsigned int &index, const DynamicTarget &target, const DynamicTargetState::TARGETSTATE &state)
{
    DynamicTargetState obj(target,state);
    std::list<DynamicTargetState>::iterator it = m_TargetList.begin();
    std::advance(it,index);
    m_TargetList.insert(it,obj);
}

void DynamicTargetList::spliceTargetListAtIndex(const unsigned int &index, const std::list<DynamicTargetState> &list)
{
    std::list<DynamicTargetState> listCopy = list;
    std::list<DynamicTargetState>::iterator it = m_TargetList.begin();
    std::advance(it,index);
    m_TargetList.splice(it,listCopy);
}

bool DynamicTargetList::isListCompleted() const
{
    if(getNextIncompleteTarget() != nullptr)
        return false;
    return true;
}

unsigned int DynamicTargetList::getActiveTargetIndex() const
{
    return this->activeTargetItem;
}

const DynamicTargetState* DynamicTargetList::getDynamicTargetState(const unsigned int &index) const
{
    std::list<DynamicTargetState>::const_iterator it = m_TargetList.begin();
    std::advance(it,index);
    return &(*it);
}

DynamicTarget DynamicTargetList::getDynamicTarget(const unsigned int &index) const
{
    std::list<DynamicTargetState>::const_iterator it = m_TargetList.begin();
    std::advance(it,index);
    return (*it);
}

const DynamicTarget* DynamicTargetList::getDynamicTarget_Pointer(const unsigned int &index) const
{
    std::list<DynamicTargetState>::const_iterator it = m_TargetList.begin();
    std::advance(it,index);
    return &(*it);
}

const DynamicTarget* DynamicTargetList::getNextIncompleteTarget() const
{
    std::list<DynamicTargetState>::const_iterator it = m_TargetList.begin();
    for (; it != m_TargetList.end(); ++it)
    {
        if((*it).getTargetState() == DynamicTargetState::TARGETSTATE::INCOMPLETE)
            return &(*it);
    }
    return nullptr;
}


const DynamicTarget* DynamicTargetList::updateTargetState(const unsigned int &index, const DynamicTargetState::TARGETSTATE &state)
{
    std::list<DynamicTargetState>::iterator it = m_TargetList.begin();
    std::advance(it,index);
    (*it).setTargetState(state);

    for (; it != m_TargetList.end(); ++it)
    {
        if((*it).getTargetState() == DynamicTargetState::TARGETSTATE::INCOMPLETE)
            return &(*it);
        activeTargetItem++;
    }
    return nullptr;
}

}//end of namespace MissionItem
