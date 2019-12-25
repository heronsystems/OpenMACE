#include "pose.h"

namespace mace {
namespace pose {

Pose::Pose()
{

}

void Pose::setTimeNow()
{
    Data::EnvironmentTime::CurrentTime(Data::Devices::SYSTEMCLOCK, m_UpdateTime);
}

} //end of namespace pose
} //end of namespace mace

