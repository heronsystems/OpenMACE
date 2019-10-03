#ifndef DYNAMIC_TARGET_KINEMATIC_H
#define DYNAMIC_TARGET_KINEMATIC_H

#include <iostream>
#include <string>
#include <sstream>

#include "common/class_forward.h"

#include "base/pose/abstract_position.h"
#include "base/pose/abstract_velocity.h"
#include "base/pose/rotation_2D.h"

using namespace mace::pose;

namespace command_target {

MACE_CLASS_FORWARD(DynamicTarget_Kinematic);

class DynamicTarget_Kinematic{
public:
    DynamicTarget_Kinematic();

    DynamicTarget_Kinematic(const Position* pos, const Velocity* vel, const Rotation_2D* rot, const Rotation_2D* rotRate);

    DynamicTarget_Kinematic(const DynamicTarget_Kinematic &copy);

    ~DynamicTarget_Kinematic();

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
    DynamicTarget_Kinematic& operator = (const DynamicTarget_Kinematic &rhs)
    {
        if(rhs.m_Position != nullptr)
            this->m_Position = rhs.m_Position->getPositionalClone();
        else
            this->m_Position = nullptr;

        if(rhs.m_Velocity != nullptr)
            this->m_Velocity = rhs.m_Velocity->getVelocityClone();
        else
            this->m_Velocity = nullptr;

        if(rhs.m_Yaw != nullptr)
            this->m_Yaw = rhs.m_Yaw->getRotationalClone()->rotationAs<Rotation_2D>();
        else
            this->m_Yaw = nullptr;

        if(rhs.m_YawRate != nullptr)
            this->m_YawRate = rhs.m_YawRate->getRotationalClone()->rotationAs<Rotation_2D>();
        else
            this->m_YawRate = nullptr;

        return *this;
    }

    bool operator == (const DynamicTarget_Kinematic &rhs) const{
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

    bool operator != (const DynamicTarget_Kinematic &rhs) const{
        return !(*this == rhs);
    }

public:
    std::string printDynamicTarget() const
    {
        std::stringstream stream;
        this->printDynamicTargetLog(stream);
        return stream.str();
    }

    friend std::ostream& operator<<(std::ostream& os, const DynamicTarget_Kinematic& t)
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

    Position* m_Position;
    Velocity* m_Velocity;
    Rotation_2D* m_Yaw;
    Rotation_2D* m_YawRate;
};

} //end of namespace command_target

#endif // DYNAMIC_TARGET_KINEMATIC_H
