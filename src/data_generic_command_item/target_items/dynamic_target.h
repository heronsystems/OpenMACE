#ifndef DYNAMIC_TARGET_H
#define DYNAMIC_TARGET_H

#include <iostream>
#include <string>
#include <sstream>

#include "common/class_forward.h"

#include "base/pose/abstract_position.h"
#include "base/pose/abstract_velocity.h"
#include "base/pose/rotation_2D.h"

using namespace mace::pose;

namespace command_target {

MACE_CLASS_FORWARD(DynamicTarget);

class DynamicTarget{
public:
    DynamicTarget();

    DynamicTarget(const Position* pos, const Velocity* vel, const Rotation_2D* rot, const Rotation_2D* rotRate);

    DynamicTarget(const DynamicTarget &copy);

    ~DynamicTarget();

public:
    void setPosition(const Position* pos);

    void setVelocity(const Velocity* vel);

    void setYaw(const Rotation_2D *rot);

    void setYawRate(const Rotation_2D* rotRate);

public:
    const Position* getPosition() const;

    const Velocity* getVelocity() const;

    const Rotation_2D* getYaw() const;

    const Rotation_2D* getYawRate() const;

    Position* getPosition();

    Velocity* getVelocity();

    Rotation_2D* getYaw();

    Rotation_2D* getYawRate();

public:
    uint16_t getCurrentTargetMask() const;

    bool isCurrentTargetValid() const;

public:
    DynamicTarget& operator = (const DynamicTarget &rhs)
    {
        this->m_Position = rhs.m_Position->getPositionalClone();
        this->m_Velocity = rhs.m_Velocity->getVelocityClone();
        this->m_Yaw = rhs.m_Yaw->getRotationalClone()->rotationAs<Rotation_2D>();
        this->m_YawRate = rhs.m_YawRate->getRotationalClone()->rotationAs<Rotation_2D>();
        return *this;
    }

    bool operator == (const DynamicTarget &rhs) const{
        if(this->m_Position != rhs.m_Position){
            return false;
        }
        if(this->m_Velocity != rhs.m_Velocity){
            return false;
        }
        if(this->m_Yaw != rhs.m_Yaw){
            return false;
        }
        if(this->m_YawRate != rhs.m_YawRate){
            return false;
        }
        return true;
    }

    bool operator != (const DynamicTarget &rhs) const{
        return !(*this == rhs);
    }

public:
    std::string printDynamicTarget() const
    {
        std::stringstream stream;
        this->printDynamicTargetLog(stream);
        return stream.str();
    }

    friend std::ostream& operator<<(std::ostream& os, const DynamicTarget& t)
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
    uint16_t targetMask;

    Position* m_Position;
    Velocity* m_Velocity;
    Rotation_2D* m_Yaw;
    Rotation_2D* m_YawRate;
};

} //end of namespace command_target

#endif // DYNAMIC_TARGET_H
