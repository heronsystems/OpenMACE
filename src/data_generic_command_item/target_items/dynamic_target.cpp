#include "dynamic_target.h"

namespace command_target {

DynamicTarget::DynamicTarget():
    m_Position(nullptr), m_Velocity(nullptr), m_Yaw(nullptr), m_YawRate(nullptr)
{

}

DynamicTarget::DynamicTarget(const Position* pos, const Velocity* vel, const Rotation_2D *rot, const Rotation_2D* rotRate)
{
    this->setPosition(pos);
    this->setVelocity(vel);
    this->setYaw(rot);
    this->setYawRate(rotRate);
}

DynamicTarget::DynamicTarget(const DynamicTarget &copy)
{
    this->setPosition(copy.getPosition());
    this->setVelocity(copy.getVelocity());
    this->setYaw(copy.getYaw());
    this->setYawRate(copy.getYawRate());
}

DynamicTarget::~DynamicTarget()
{
    delete m_Position; m_Position = nullptr;
    delete m_Velocity; m_Velocity = nullptr;
    delete m_Yaw; m_Yaw = nullptr;
    delete m_YawRate; m_YawRate = nullptr;
}

void DynamicTarget::setPosition(const Position* pos)
{
    delete m_Position; m_Position = nullptr;
    this->m_Position = pos->getPositionalClone();
}

void DynamicTarget::setVelocity(const Velocity* vel)
{
    delete m_Velocity; m_Velocity = nullptr;
    this->m_Velocity = vel->getVelocityClone();
}

void DynamicTarget::setYaw(const Rotation_2D* rot)
{
    delete m_Yaw; m_Yaw = nullptr;
    this->m_Yaw = rot->getRotationalClone()->rotationAs<Rotation_2D>();
}

void DynamicTarget::setYawRate(const Rotation_2D* rotRate)
{
    delete m_YawRate; m_YawRate = nullptr;
    this->m_YawRate = rotRate->getRotationalClone()->rotationAs<Rotation_2D>();
}

const Position* DynamicTarget::getPosition() const
{
    return this->m_Position;
}

const Velocity* DynamicTarget::getVelocity() const
{
    return this->m_Velocity;
}

const Rotation_2D* DynamicTarget::getYaw() const
{
    return this->m_Yaw;
}

const Rotation_2D* DynamicTarget::getYawRate() const
{
    return this->m_YawRate;
}

Position* DynamicTarget::getPosition()
{
    return this->m_Position;
}

Velocity* DynamicTarget::getVelocity()
{
    return this->m_Velocity;
}

Rotation_2D* DynamicTarget::getYaw()
{
    return this->m_Yaw;
}

Rotation_2D* DynamicTarget::getYawRate()
{
    return this->m_YawRate;
}

bool DynamicTarget::isCurrentTargetValid() const
{
    if(getCurrentTargetMask() != std::numeric_limits<uint16_t>::max())
        return true;
    return false;
}

uint16_t DynamicTarget::getCurrentTargetMask() const
{
    uint16_t currentMask = 0;

    if(m_Position)
        currentMask = currentMask|m_Position->getDimensionMask();
    if(m_Velocity)
        currentMask = currentMask|m_Velocity->getDimensionMask();
    if(m_Yaw)
        currentMask = currentMask|m_Yaw->getDimensionMask();
    if(m_YawRate)
        currentMask = currentMask|m_YawRate->getDimensionMask();

    return currentMask;
}

} //end of namespace command_target
