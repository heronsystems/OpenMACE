#include "graph_planning_node.h"

namespace mace {
namespace planners_graph{
GraphNode::GraphNode():
    parentNode(nullptr), currentState(nullptr)
{

}

GraphNode::GraphNode(const GraphNode &copy)
{
    this->fValue = copy.fValue;
    this->gValueParent = copy.gValueParent;
    this->gValueChild = copy.gValueChild;
    this->gValue = copy.gValue;
    this->hValue = copy.hValue;
    this->open = copy.open;
    this->closed = copy.closed;
    this->parentNode = copy.parentNode;
    this->currentState = copy.currentState;

}

double GraphNode::updateGValue(const GRelationship &obj, const double &newValue)
{
    switch (obj) {
    case GRelationship::G_TOTAL:
        gValue = newValue;
        break;
    case GRelationship::G_PARENT:
        gValueParent = newValue;
        gValue = gValueParent + gValueChild;
        break;
    case GRelationship::G_CHILD:
        gValueChild = newValue;
        gValue = gValueParent + gValueChild;
        break;
    default:
        break;
    }

    return gValue;
}

double GraphNode::updateGValue(const double &valueParent, const double &valueChild)
{
    gValueParent = valueParent;
    gValueChild = valueChild;
    gValue = gValueParent + gValueChild;
    return gValue;
}

double GraphNode::updateHValue(const double &newValue)
{
    this->hValue = newValue;
    this->updateFValue();
    return this->fValue;
}

void GraphNode::setParentNode(GraphNode* parent)
{
    this->parentNode = parent;
}

void GraphNode::hasBeenOpened(const bool &value)
{
    this->open = value;
    this->closed = !value;
}

void GraphNode::hasBeenClosed(const bool &value)
{
    this->closed = value;
    this->open = !value;
}

void GraphNode::updateFValue()
{
    this->fValue = this->gValue + this->hValue;
}

double  GraphNode::getFValue() const
{
    return fValue;
}

double  GraphNode::getGValue(const GRelationship &obj) const
{
    switch (obj) {
    case GRelationship::G_TOTAL:
        return gValue;
        break;
    case GRelationship::G_PARENT:
        return gValueParent;
        break;
    case GRelationship::G_CHILD:
        return gValueChild;
        break;
    default:
        return gValue;
        break;
    }
}

double  GraphNode::getHValue() const
{
    return hValue;
}

bool  GraphNode::isOpen() const
{
    return open;
}

bool  GraphNode::isClosed() const
{
    return closed;
}

GraphNode* GraphNode::getParentNode() const
{
    return this->parentNode;
}

} //end of namespace planners_graph
} //end of namespace mace
