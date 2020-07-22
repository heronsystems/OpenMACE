#ifndef DYNAMIC_TARGET_ORIENTATION_H
#define DYNAMIC_TARGET_ORIENTATION_H

#include <iostream>
#include <string>
#include <sstream>

#include "common/class_forward.h"

#include "abstract_dynamic_target.h"

#include "base/pose/abstract_position.h"
#include "base/pose/abstract_velocity.h"
#include "base/pose/abstract_rotation.h"

using namespace mace::pose;

namespace command_target {

MACE_CLASS_FORWARD(DynamicTarget_Orientation);

class DynamicTarget_Orientation : public DynamicTarget{

public:
    DynamicTarget_Orientation();

    DynamicTarget_Orientation(const AbstractRotation* rot, const AbstractRotation* rotRate, const double &thrust);

    DynamicTarget_Orientation(const DynamicTarget_Orientation &copy);

    ~DynamicTarget_Orientation() override;

public: //imposed interface items from Dynamic Target
    TargetTypes getTargetType() const override;

    DynamicTarget* getDynamicTargetClone() const override
    {
        return (new DynamicTarget_Orientation(*this));
    }

    void getDynamicTargetClone(DynamicTarget** target) const override
    {
        *target = new DynamicTarget_Orientation(*this);
    }

public:
    void setTargetOrientation(const AbstractRotation* rot);

    //void setTargetOrientationRate(const AbstractRotation* rot);

    void setTargetThrust(const double &thrust);

    void setTargetYawRate(const double &yawRate);


public:
    const AbstractRotation* getTargetOrientation() const;

    //const AbstractRotation* getTargetOrientationRate() const;

    AbstractRotation* getTargetOrientation();

    //AbstractRotation* getTargetOrientationRate();

    double getTargetThrust() const;

    double getTargetYawRate() const;

public:
    uint8_t getCurrentTargetMask() const;

    bool isCurrentTargetValid() const;

public:
    DynamicTarget_Orientation& operator = (const DynamicTarget_Orientation &rhs)
    {
        if(rhs.m_Rotation != nullptr)
            this->m_Rotation = rhs.m_Rotation->getRotationalClone();
        else
            this->m_Rotation = nullptr;

        this->m_Thrust = rhs.m_Thrust;

        this->m_YawRate = rhs.m_YawRate;
        return *this;
    }

    bool operator == (const DynamicTarget_Orientation &rhs) const{
        if(this->m_Rotation != rhs.m_Rotation){
            return false;
        }
        if(this->m_Thrust != rhs.m_Thrust){
            return false;
        }
        if(this->m_YawRate != rhs.m_YawRate){
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

    double m_Thrust = -1000; //this is set to a value it shouldn't be acceptable values are in the range of -1 to 1 or 0 to 1 depending upon if throttle capable of inversion

    double m_YawRate = 0.0;
};

} //end of namespace command_target

#endif // DYNAMIC_TARGET_ORIENTATION_H
