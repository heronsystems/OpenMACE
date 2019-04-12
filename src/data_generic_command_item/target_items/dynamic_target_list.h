#ifndef DYNAMIC_TARGET_LIST_H
#define DYNAMIC_TARGET_LIST_H
#include <iostream>
#include <stdint.h>
#include <memory>
#include <list>
#include <map>

#include "../mission_items/mission_list.h"

#include "base/pose/cartesian_position_3D.h"
#include "base/pose/cartesian_velocity_3D.h"

#include "dynamic_target_storage.h"

namespace TargetItem {

class DynamicTargetList{

public:
    DynamicTargetList();
    DynamicTargetList(const DynamicTargetList &rhs);

public:
    size_t listSize() const;
    void clearList();

    void appendDynamicTarget(const CartesianDynamicTarget &target, const DynamicTargetStorage::TargetCompletion &state = DynamicTargetStorage::TargetCompletion::INCOMPLETE);
    void removeTargetAtIndex(const unsigned int &index);

    void replaceTargetAtIndex(const unsigned int &index, const CartesianDynamicTarget &target, const DynamicTargetStorage::TargetCompletion &state = DynamicTargetStorage::TargetCompletion::INCOMPLETE);
    void spliceTargetListAtIndex(const unsigned int &index, const std::list<DynamicTargetStorage> &list);

    bool isCompleted() const;

    unsigned int getActiveTargetItem() const;


public:
    const DynamicTargetStorage* getTargetStorageAtIndex(const unsigned int &index) const;
    CartesianDynamicTarget getTargetAtIndex(const unsigned int &index) const;
    const CartesianDynamicTarget* getTargetPointerAtIndex(const unsigned int &index) const;
    const CartesianDynamicTarget* getNextIncomplete() const;
    const CartesianDynamicTarget* markCompletionState(const unsigned int &index, const DynamicTargetStorage::TargetCompletion &state);

public:

    DynamicTargetList& operator = (const DynamicTargetList &rhs)
    {
        this->activeTargetItem = rhs.activeTargetItem;
        this->targetList = rhs.targetList;
        return *this;
    }

    bool operator == (const DynamicTargetList &rhs) const{
        if(this->activeTargetItem != rhs.activeTargetItem){
            return false;
        }
        if(this->targetList != rhs.targetList){
            return false;
        }
        return true;
    }

    bool operator != (const DynamicTargetList &rhs) const{
        return !(*this == rhs);
    }

private:
    std::list<DynamicTargetStorage> targetList;
    unsigned int activeTargetItem = 0;

};

} //end of namespace MissionItem
#endif // DYNAMIC_TARGET_LIST_H
