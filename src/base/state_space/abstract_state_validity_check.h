#ifndef ABSTRACT_STATE_VALIDITY_CHECK_H
#define ABSTRACT_STATE_VALIDITY_CHECK_H

#include "base/base_global.h"
#include "common/common.h"
#include "common/class_forward.h"

#include "common/optional_parameter.h"

#include "state_space.h"

namespace mace {
namespace state_space {

/**
 *
 */
MACE_CLASS_FORWARD(AbstractStateValidityCheck);

class BASESHARED_EXPORT AbstractStateValidityCheck{

public:

    /**
     * @brief StateValidityCheck
     * @param space
     */
    AbstractStateValidityCheck(const StateSpacePtr &space):
        m_stateSpace(space.get())
    {

    }

    /**
     * @brief StateValidityCheck
     */
    AbstractStateValidityCheck(const AbstractStateValidityCheck &) = delete;

    /**
     * @brief operator =
     * @return
     */
    AbstractStateValidityCheck &operator=(const AbstractStateValidityCheck &) = delete;

    /**
    *
    */
    virtual ~AbstractStateValidityCheck() = default;

    /**
     * @brief updateStateSpace
     * @param space
     */
    void updateStateSpace(const StateSpacePtr &space)
    {
        m_stateSpace = space.get();
    }

public:
    /**
     * @brief isValid
     * @param state
     * @return
     */
    virtual bool isValid(const State *state) const = 0;

    void setLambda_Validity(const std::function<bool(const State *state)> &lambda){
        m_ValidityCheckLamba = lambda;
    }

protected:
    /**
     * @brief m_stateSpace
     */
    StateSpace* m_stateSpace;

    OptionalParameter<std::function<bool(const State *state)>> m_ValidityCheckLamba;

};

MACE_CLASS_FORWARD(AllStateValidityCheck);

class BASESHARED_EXPORT AllStateValidityCheck: public AbstractStateValidityCheck{

public:
    AllStateValidityCheck(const StateSpacePtr &space):
        AbstractStateValidityCheck(space)
    {

    }

    virtual ~AllStateValidityCheck() = default;


    bool isValid(const State *state) const override
    {
        UNUSED(state);
        return true;
    }

};

} //end of namespace state_space
} //end of namespace mace

#endif // STATE_VALIDITY_CHECK_H
