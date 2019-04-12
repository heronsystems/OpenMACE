#ifndef DISCRETE_MOTION_VALIDITY_CHECK_H
#define DISCRETE_MOTION_VALIDITY_CHECK_H

#include "base/base_global.h"
#include "common/class_forward.h"
#include "base/state_space/abstract_motion_validity_check.h"

#include "base/state_space/abstract_state_validity_check.h"

namespace mace {
namespace state_space {

MACE_CLASS_FORWARD(DiscreteMotionValidityCheck);

class DiscreteMotionValidityCheck : public AbstractMotionValidityCheck
{
public:
    DiscreteMotionValidityCheck(const StateSpacePtr &space);

    void setStateValidityCheck(const AbstractStateValidityCheckPtr &stateChecker);

public:
    /**
     * @brief isValid
     * @param begin
     * @param end
     * @return
     */
    bool isValid(const State* begin, const State *end) const override;

private:
    AbstractStateValidityCheckPtr m_StateCheck;
};

} //end of namespace state_space
} //end of namespace mace

#endif // DISCRETE_MOTION_VALIDITY_CHECK_H
