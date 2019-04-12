#ifndef GRAPH_PLANNING_NODE_H
#define GRAPH_PLANNING_NODE_H

#include "base/state_space/state_space.h"

namespace mace {
namespace planners_graph{

enum class GRelationship
{
    G_TOTAL,
    G_PARENT,
    G_CHILD
};

class GraphNode
{
public:
    GraphNode();

    GraphNode(const GraphNode &copy);


    ~GraphNode() = default;

public:
    void setCurrentState(const state_space::State* state) { this->currentState = state; }
    const state_space::State* getCurrentState()const { return this->currentState; }

public:
    double updateGValue(const GRelationship &obj, const double &newValue);

    double updateGValue(const double &valueParent, const double &valueChild);

    double updateHValue(const double &newValue);

    void setParentNode(GraphNode *parent);

    void hasBeenOpened(const bool &value);

    void hasBeenClosed(const bool &value);

public:
    double getFValue() const;

    double getGValue(const GRelationship &obj = GRelationship::G_TOTAL) const;

    double getHValue() const;

    bool isOpen() const;

    bool isClosed() const;

    GraphNode *getParentNode() const;

private:
    void updateFValue();

public:
    /** Relational Operators */
public:

    //!
    //! \brief operator <
    //! \param rhs
    //! \return
    //!
    bool operator < (const GraphNode &rhs) const
    {
        if(this->fValue >= rhs.fValue)
            return false;
        return true;
    }

    //!
    //! \brief operator >=
    //! \param rhs
    //! \return
    //!
    bool operator >= (const GraphNode &rhs) const
    {
        return !(*this < rhs);
    }

    //!
    //! \brief operator >
    //! \param rhs
    //! \return
    //!
    bool operator > (const GraphNode &rhs) const
    {
        if(this->fValue <= rhs.fValue)
            return false;
        return true;
    }

    //!
    //! \brief operator <=
    //! \param rhs
    //! \return
    //!
    bool operator <= (const GraphNode &rhs) const
    {
        return !(*this > rhs);
    }

    //!
    //! \brief operator ==
    //! \param rhs
    //! \return
    //!
    bool operator == (const GraphNode &rhs) const
    {
        if(this->fValue != rhs.fValue){
            return false;
        }
        if(this->gValueParent != rhs.gValueParent){
            return false;
        }
        if(this->gValueChild != rhs.gValueChild){
            return false;
        }
        if(this->gValue != rhs.gValue){
            return false;
        }
        if(this->hValue != rhs.hValue){
            return false;
        }
        if(this->open != rhs.open){
            return false;
        }
        if(this->closed != rhs.closed){
            return false;
        }
        if(this->parentNode != rhs.parentNode){
            return false;
        }
        if(this->currentState != rhs.currentState){
            return false;
        }
        return true;
    }

    //!
    //! \brief operator !=
    //! \param rhs
    //! \return
    //!
    bool operator != (const GraphNode &rhs) const{
        return !(*this == rhs);
    }

private:
    double fValue = 0.0; /**< Member containing the summation of gValue and hValue .*/
    double gValueParent = 0.0; /**< Member containing the current cost up through the parent node. This should be
                               equal to that of the parents total gCost.*/
    double gValueChild = 0.0; /**< Member containing the current to go from the parent node to this node.*/
    double gValue = 0.0; /**< Member holding the summation of g parent and child.*/
    double hValue = 0.0; /**< Member containing an estimate of the remaining cost to achieve the goal.*/

    bool open = false; /**< Member boolean denoting whether this node as already been visited and expanded.*/
    bool closed = false; /**< Member boolean denoting whether this node as already been visited and expanded.*/

    GraphNode* parentNode; /**< Member variable that contains a pointer to the parent node connecting
                             to this node currently offering the shortest known path. */

    const state_space::State* currentState; /**< Member variable containing the current state of the node. The planner is
                                           relevantly oblivious to this state. Rather acting through the state space
                                            information. */

};


} //end of namespace planners_graph
} //end of namespace mace

#endif // GRAPH_PLANNING_NODE_H
