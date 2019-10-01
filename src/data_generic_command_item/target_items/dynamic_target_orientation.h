#ifndef DYNAMIC_TARGET_ORIENTATION_H
#define DYNAMIC_TARGET_ORIENTATION_H

#include <iostream>
#include <string>
#include <sstream>

#include "common/class_forward.h"

#include "base/pose/abstract_position.h"
#include "base/pose/abstract_velocity.h"
#include "base/pose/abstract_rotation.h"

using namespace mace::pose;

namespace command_target {

MACE_CLASS_FORWARD(DynamicTarget_Orientation);

class DynamicTarget_Orientation{
public:
    DynamicTarget_Orientation();

    DynamicTarget_Orientation(const AbstractRotation* rot, const double &thrust);

    DynamicTarget_Orientation(const DynamicTarget_Orientation &copy);

    ~DynamicTarget_Orientation();

public:
    void setTargetOrientation(const AbstractRotation* rot);

    void setTargetThrust(const double &thrust);

public:
    const AbstractRotation* getTargetOrientation() const;

    AbstractRotation* getTargetOrientation();

public:
    uint16_t getCurrentTargetMask() const;

    bool isCurrentTargetValid() const;

public:
    DynamicTarget_Orientation& operator = (const DynamicTarget_Orientation &rhs)
    {
        if(rhs.m_Rotation != nullptr)
            this->m_Rotation = rhs.m_Rotation->getRotationalClone();
        else
            this->m_Rotation = nullptr;

        return *this;
    }

    bool operator == (const DynamicTarget_Orientation &rhs) const{
        if(this->m_Rotation != rhs.m_Rotation){
            return false;
        }
        return true;
    }

    bool operator != (const DynamicTarget_Orientation &rhs) const{
        return !(*this == rhs);
    }

public:
    std::string printDynamicTarget_Orientation() const
    {
        std::stringstream stream;
        this->printDynamicTargetLog(stream);
        return stream.str();
    }

    friend std::ostream& operator<<(std::ostream& os, const DynamicTarget_Orientation& t)
    {
        std::stringstream newStream;
        t.printDynamicTargetLog(newStream);
        os << newStream.str();
        return os;
    }

    void printDynamicTargetLog(std::stringstream &stream) const
    {
        stream << "DYNTARGET|";

    }

protected:
    uint16_t targetMask = std::numeric_limits<uint16_t>::max();

    AbstractRotation* m_Rotation;
};

} //end of namespace command_target

#endif // DYNAMIC_TARGET_ORIENTATION_H
