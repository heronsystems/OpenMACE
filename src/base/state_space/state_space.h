#ifndef STATE_SPACE_H
#define STATE_SPACE_H

#include <vector>
#include <string>

#include "base/base_global.h"

#include "common/class_forward.h"
#include "state_space_types.h"
#include "state.h"

namespace mace {
namespace state_space {

MACE_CLASS_FORWARD(StateSpace);

class BASESHARED_EXPORT StateSpace
{
public:
    StateSpace();

    virtual ~StateSpace();

    StateSpace(const StateSpace& copy) = delete;
    StateSpace &operator =(const StateSpace &copy) = delete;


    template <class T>
    T *as()
    {
        return static_cast<T*>(this);
    }

    template <class T>
    const T *as() const
    {
        return static_cast<const T*>(this);
    }

public:
    const std::string &getName() const
    {
        return m_name;
    }

    void setName(const std::string &name)
    {
        this->m_name = name;
    }

    StateSpaceTypes getType() const
    {
        return m_type;
    }

public:
    virtual double distanceBetween(const State* lhs, const State* rhs) const = 0;


    /** The purpose of the following functions is to handle the state definition
        from the state space is that sampled. We allow for the space to handle
        the memory allocation in the event there are specifics unaware to the
        class implementors. It should be that this class shall handle the deletion
        of the state.
    */
public:
    //!
    //! \brief getNewState
    //! \return
    //!
    virtual State* getNewState() const;

    //!
    //! \brief removeState
    //! \param state
    //!
    virtual void removeState(State* state) const;

    //!
    //! \brief copyState
    //! \param state
    //! \return
    //!
    virtual State* copyState(const State* state) const;

    //!
    //! \brief removeStates
    //! \param states
    //!
    virtual void removeStates(std::vector<State*> states) const;

    virtual bool interpolateStates(const State* begin, const State* end, const double & percentage, State** interState);

    virtual double traversalCost(const State* begin, const State* end);

    virtual std::vector<State*> getNeighboringStates(const State *currentState) const;

protected:
    StateSpaceTypes m_type;
    std::string m_name;

};

} //end of namespace state_space
} //end of namespace mace

#endif // STATE_SPACE_H
