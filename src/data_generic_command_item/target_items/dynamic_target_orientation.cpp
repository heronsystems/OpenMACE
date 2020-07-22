#include "dynamic_target_orientation.h"

namespace command_target {

DynamicTarget_Orientation::DynamicTarget_Orientation():
    m_Rotation(nullptr), m_Thrust(0.0)
{

}

DynamicTarget_Orientation::DynamicTarget_Orientation(const AbstractRotation* orientation, const AbstractRotation* orientationRate, const double &thrust):
    m_Rotation(nullptr), m_Thrust(0.0), m_YawRate(0.0)
{
    this->setTargetOrientation(orientation);

    this->setTargetThrust(thrust);
}

DynamicTarget_Orientation::DynamicTarget_Orientation(const DynamicTarget_Orientation &copy):
    m_Rotation(nullptr), m_Thrust(0.0), m_YawRate(0.0)
{
    this->setTargetOrientation(copy.getTargetOrientation());

    this->setTargetThrust(copy.getTargetThrust());

    this->setTargetYawRate(copy.getTargetYawRate());
}

DynamicTarget_Orientation::~DynamicTarget_Orientation()
{
    delete m_Rotation; m_Rotation = nullptr;
    //    delete m_RotationRate; m_RotationRate = nullptr;
}

DynamicTarget::TargetTypes DynamicTarget_Orientation::getTargetType() const
{
    return DynamicTarget::TargetTypes::ORIENTATION;
}

void DynamicTarget_Orientation::setTargetOrientation(const AbstractRotation* rot)
{
    if(m_Rotation != nullptr)
    {
        delete m_Rotation;
        m_Rotation = nullptr;
    }
    if(rot != nullptr)
        this->m_Rotation = rot->getRotationalClone();
}

/*
void DynamicTarget_Orientation::setTargetOrientationRate(const AbstractRotation* rotRate)
{
    if(m_RotationRate)
    {
        delete m_RotationRate;
        m_RotationRate = nullptr;
    }
    if(rotRate != nullptr)
        this->m_RotationRate = rotRate->getRotationalClone();
}
*/
void DynamicTarget_Orientation::setTargetThrust(const double &thrust)
{
    m_Thrust = thrust;
}

void DynamicTarget_Orientation::setTargetYawRate(const double &yawRate)
{
    m_YawRate = yawRate;
}

const AbstractRotation* DynamicTarget_Orientation::getTargetOrientation() const
{
    return this->m_Rotation;
}

/*
const AbstractRotation* DynamicTarget_Orientation::getTargetOrientationRate() const
{
    return this->m_RotationRate;
}
*/

AbstractRotation* DynamicTarget_Orientation::getTargetOrientation()
{
    return this->m_Rotation;
}

/*
AbstractRotation* DynamicTarget_Orientation::getTargetOrientationRate()
{
    return this->m_RotationRate;
}
*/

double DynamicTarget_Orientation::getTargetThrust() const
{
    return this->m_Thrust;
}

double DynamicTarget_Orientation::getTargetYawRate() const
{
    return this->m_YawRate;
}

bool DynamicTarget_Orientation::isCurrentTargetValid() const
{
    if(getCurrentTargetMask() != std::numeric_limits<uint16_t>::max())
        return true;
    return false;
}

uint8_t DynamicTarget_Orientation::getCurrentTargetMask() const
{
    uint8_t currentMask = 0;
    if(m_Rotation)
        currentMask = currentMask|128;
    if(fabs(m_Thrust - -1000) > std::numeric_limits<double>::epsilon())
        currentMask = currentMask|64;
    if(fabs(m_YawRate - 0.0) > std::numeric_limits<double>::epsilon())
        currentMask = currentMask|4;

    return currentMask;
}

} //end of namespace command_target
