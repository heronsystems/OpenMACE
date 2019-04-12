#ifndef RRT_NODE_H
#define RRT_NODE_H

#include "base/state_space/state_space.h"
namespace mace {
namespace planners_sampling{

//!
//! \brief A Node in a tree
//! The node contains a state and a pointer to a parent.
//!
class RootNode
{
public:

    //!
    //! \brief Constructor that creates a new State
    //! \param stateSpace Space that states can exists in
    //!
    RootNode(const state_space::StateSpacePtr &stateSpace):
        currentState(stateSpace->getNewState()), parentNode(nullptr)
    {

    }

    //!
    //! \brief Contructor that copies an existing state
    //! \param state State to copy
    //!
    RootNode(const state_space::State &state):
        currentState(state.getClone()), parentNode(nullptr)
    {

    }

    ~RootNode()
    {
        delete currentState;
    }

public:

    //!
    //! \brief Sets a new current state for this node.
    //! \param state State to copy and assign
    //!
    void setCurrentState(const state_space::State &state) {

        state_space::State* tmp = this->currentState;
        this->currentState = state.getClone();
        delete tmp;
    }
    void setParentNode(RootNode* node) { this->parentNode = node; }

    state_space::State* getCurrentState()const { return this->currentState; }
    RootNode* getParentNode()const { return this->parentNode; }

private:
    state_space::State* currentState;
    RootNode* parentNode;
};

} //end of namespace planners_sampling
} //end of namespace mace
#endif // RRT_NODE_H
