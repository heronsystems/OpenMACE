#ifndef DYNAMIC_TARGET_LIST_H
#define DYNAMIC_TARGET_LIST_H

#include <list>

#include "dynamic_target_state.h"

namespace command_target {

MACE_CLASS_FORWARD(DynamicTargetList);

class DynamicTargetList{

public:
    //!
    //! \brief DynamicTargetList
    //!
    DynamicTargetList();

    //!
    //! \brief DynamicTargetList
    //! \param rhs
    //!
    DynamicTargetList(const DynamicTargetList &rhs);

public:
    void clearList();

    void appendDynamicTarget(const DynamicTarget &target, const DynamicTargetState::TARGETSTATE &state = DynamicTargetState::TARGETSTATE::INCOMPLETE);

    void removeTargetAtIndex(const unsigned int &index);

    void replaceTargetAtIndex(const unsigned int &index, const DynamicTarget &target, const DynamicTargetState::TARGETSTATE &state = DynamicTargetState::TARGETSTATE::INCOMPLETE);

    void spliceTargetListAtIndex(const unsigned int &index, const std::list<DynamicTargetState> &list);

    bool isListCompleted() const;

    unsigned int getActiveTargetIndex() const;

public:
    const DynamicTargetState* getDynamicTargetState(const unsigned int &index) const;

    DynamicTarget getDynamicTarget(const unsigned int &index) const;

    const DynamicTarget* getDynamicTarget_Pointer(const unsigned int &index) const;

    const DynamicTarget* getNextIncompleteTarget() const;

    const DynamicTarget* updateTargetState(const unsigned int &index, const DynamicTargetState::TARGETSTATE &state);

    /** Assignment Operators */
public:
    DynamicTargetList& operator = (const DynamicTargetList &rhs)
    {
        this->activeTargetItem = rhs.activeTargetItem;

        this->m_TargetList = rhs.m_TargetList;
        return *this;
    }

    /** Relational Operators */
public:
    bool operator == (const DynamicTargetList &rhs) const{
        if(this->activeTargetItem != rhs.activeTargetItem){
            return false;
        }
        if(this->m_TargetList != rhs.m_TargetList){
            return false;
        }
        return true;
    }

    bool operator != (const DynamicTargetList &rhs) const{
        return !(*this == rhs);
    }

public:
    std::list<DynamicTargetState> m_TargetList;

private:
    unsigned int activeTargetItem = 0;
};

} //end of namespace MissionItem
#endif // DYNAMIC_TARGET_LIST_H
