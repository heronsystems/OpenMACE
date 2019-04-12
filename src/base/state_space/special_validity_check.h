#ifndef SPECIAL_VALIDITY_CHECK_H
#define SPECIAL_VALIDITY_CHECK_H

#include "base/base_global.h"
#include "common/class_forward.h"

#include "base/state_space/abstract_state_validity_check.h"
#include "base/pose/cartesian_position_2D.h"

namespace mace {
namespace state_space {

MACE_CLASS_FORWARD(SpecialValidityCheck);

class SpecialValidityCheck : public AbstractStateValidityCheck
{
public:
    SpecialValidityCheck(const StateSpacePtr &space);

public:
    bool isValid(const State *state) const override;

};

} //end of namespace state_space
} //end of namespace mace
#endif // SPECIAL_VALIDITY_CHECK_H
