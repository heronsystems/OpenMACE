#include "special_validity_check.h"

namespace mace {
namespace state_space {

SpecialValidityCheck::SpecialValidityCheck(const StateSpacePtr &space):
    AbstractStateValidityCheck(space)
{

}

bool SpecialValidityCheck::isValid(const State *state) const
{
    if(m_ValidityCheckLamba.IsSet())
    {
        return m_ValidityCheckLamba()(state);
    }
    else
        return true;

//    const pose::CartesianPosition_2D* castState = state->as<const pose::CartesianPosition_2D>();
//    double x = castState->getXPosition();
//    double y = castState->getYPosition();
//    if((x <= 7.5) && (x>=2.5))
//    {
//        if((y <= 7.5) && (y>=2.5))
//            return false;
//    }
}

} //end of namespace state_space
} //end of namespace mace


