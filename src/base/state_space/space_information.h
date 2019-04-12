#ifndef SPACE_INFORMATION_H
#define SPACE_INFORMATION_H

#include <functional>

#include "common/class_forward.h"
#include "base/base_global.h"

#include "base/state_space/state_space.h"
#include "base/state_space/state_sampler.h"
#include "base/state_space/abstract_state_validity_check.h"
#include "base/state_space/abstract_motion_validity_check.h"

namespace mace {
namespace state_space {

typedef std::function<bool(const State* state)> StateValidityFunction;

MACE_CLASS_FORWARD(SpaceInformation);

class BASESHARED_EXPORT SpaceInformation
{

public:
    /**
     * @brief SpaceInformation
     * @param space
     */
    SpaceInformation(const StateSpacePtr &space);

    /**
      */
    virtual ~SpaceInformation() = default;

    /**
      */

    /**
     * @brief SpaceInformation
     */
    SpaceInformation(const SpaceInformation &) = delete;

    /**
     * @brief operator =
     * @return
     */
    SpaceInformation &operator=(const SpaceInformation &) = delete;

    /**
     * @brief updateStateSpace
     * @param space
     */
    void updateStateSpace(const StateSpacePtr &space);

    /**
     * @brief getStateSpace
     * @return
     */
    const StateSpacePtr& getStateSpace() const;

public:

    /**
     * @brief isStateValid
     * @param state
     * @return
     */
    bool isStateValid(const State* state) const;

    /**
     * @brief isEdgeValid
     * @param lhs
     * @param rhs
     * @return
     */
    bool isEdgeValid(const State* lhs, const State* rhs) const;

    /**
     * @brief distanceBetween
     * @param lhs
     * @param rhs
     * @return
     */
    double distanceBetween(const State* lhs, const State* rhs) const;

    /**
     * @brief isValid
     * @param state
     * @return
     */
    bool isValid(const State *state) const
    {
        UNUSED(state);
        return true;
        //return m_stateValidCheck.isValid(state);
    }

    /**
     * @brief setStateSampler
     * @param sampler
     */
    void setStateSampler(const StateSamplerPtr &sampler);

    /**
     * @brief getStateSampler
     * @return
     */
    StateSamplerPtr getStateSampler() const;

public:
    /** \brief Cast this instance to a desired type. */
    template <class T>
    const T *as() const
    {
        //ensure that we are attempting to cast it to a type of state
        return static_cast<const T *>(this);
    }

    /** \brief Cast this instance to a desired type. */
    template <class T>
    T *as()
    {
        //ensure that we are attempting to cast it to a type of state
        return static_cast<T *>(this);
    }

public:
    /**
     * @brief getNewState
     * @return
     */
    State* getNewState() const;

    /**
     * @brief removeState
     * @param state
     */
    void removeState(State* state) const;

    /**
     * @brief copyState
     * @param state
     * @return
     */
    State* copyState(const State *state) const;

    /**
     * @brief removeStates
     * @param states
     */
    void removeStates(std::vector<State*> states) const;

public:
    void setStateValidityCheck(const AbstractStateValidityCheckPtr &stateChecker);

    void setMotionValidityCheck(const AbstractMotionValidityCheckPtr &motionChecker);

    double getTraversalCost(const State *begin, const State *end, const bool &neighbor = false) const;


private:
    /**
     * @brief isSetup
     */
    bool isSetup;

protected:
    StateSpacePtr m_stateSpace;

    AbstractStateValidityCheckPtr m_stateValidCheck;
    AbstractMotionValidityCheckPtr m_motionValidCheck;
    StateSamplerPtr m_stateSampler;
};

} //end of namespace state_space
} //end of namespace mace

#endif // SPACE_INFORMATION_H
