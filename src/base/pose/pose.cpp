#include "pose.h"

namespace mace {
namespace pose {

Pose::Pose()
{

}

Pose::Pose(const Pose &copyObj)
{
    this->m_Position = copyObj.m_Position;
    this->m_Rotation = copyObj.m_Rotation;
    this->m_UpdateTime = copyObj.m_UpdateTime;
}

void Pose::setTimeNow()
{
    Data::EnvironmentTime::CurrentTime(Data::Devices::SYSTEMCLOCK, m_UpdateTime);
}

} //end of namespace pose
} //end of namespace mace

