#include "a_star_base.h"

namespace mace {
namespace planners_graph{

void AStarBase::retracePath(const GraphNode* start, const GraphNode* end, std::vector<state_space::State*> &path)
{
    std::vector<const state_space::State*> backwardsPath;
    const GraphNode* currentNode = end;
    while(currentNode != start)
    {
        backwardsPath.push_back(currentNode->getCurrentState());
        currentNode = currentNode->getParentNode();
    }

    path.clear();
    for(int i = backwardsPath.size() - 1; i >= 0; i--)
    {
        path.push_back(backwardsPath.at(i)->getStateClone());
        std::cout<<"The position here is: "<<backwardsPath.at(i)->printInfo()<<std::endl;
    }
}


std::vector<state_space::State *> AStarBase::solve(maps::Data2DGrid<GraphNode> &stateGridData)
{
    std::vector<state_space::State*> solutionVector;

    GraphNode* startNode = stateGridData.getCellByIndex(0);

    GraphNode* endNode = stateGridData.getCellByIndex(stateGridData.getSize() - 1);

    std::multiset<GraphNode*> openSet;
    std::multiset<GraphNode*>::iterator openSetIT;

    std::unordered_set<GraphNode*> closedSet;

    openSet.insert(startNode);
    startNode->hasBeenOpened(true);

    while(!openSet.empty())
    {
        openSetIT = openSet.begin();
        GraphNode* currentNode = *openSetIT;
        ++openSetIT;
        for(;openSetIT != openSet.end(); openSetIT++)
        {
            GraphNode* evalNode = *openSetIT;
            if((evalNode->getFValue() < currentNode->getFValue()) || ((evalNode->getFValue() == currentNode->getFValue()) &&
                                                                      evalNode->getHValue() < currentNode->getHValue()))
            {
                currentNode = evalNode;
            }
        }

        currentNode->hasBeenClosed(true);
        openSet.erase(currentNode);
        closedSet.insert(currentNode);

        if(currentNode == endNode)
        {
            retracePath(startNode, endNode, solutionVector);
            return solutionVector;
        }

        //get the appropriate neighbors
        unsigned int currentNodeIndex = 0;
//        bool found = stateGridData.findIndex(currentNode, currentNodeIndex);
        std::vector<int> neighbors = stateGridData.getCellNeighbors(currentNodeIndex);

        for(size_t i = 0; i < neighbors.size(); i++)
        {
            GraphNode* neighborNode = stateGridData.getCellByIndex(neighbors.at(i));

            //An edge valid check ensures that the current state is valid, and the connection between states is valid
            if(!m_spaceInfo->isEdgeValid(neighborNode->getCurrentState(),currentNode->getCurrentState()) || neighborNode->isClosed())
                continue;

            //movement cost to go from the current node to the neighboring node
            double movementCost = m_spaceInfo->getTraversalCost(currentNode->getCurrentState(),neighborNode->getCurrentState());
            //the total movement cost is therefore equal to the previous nodes cost plus the cost to move
            double parentCost = currentNode->getGValue(planners_graph::GRelationship::G_PARENT);
            double totalCost = parentCost + movementCost;

            //if the new path to neighbor is shorter || neighbor is not in OPEN
            if(totalCost < neighborNode->getGValue() || !neighborNode->isOpen())
            {
                neighborNode->updateGValue(parentCost,movementCost);
                neighborNode->updateHValue(m_spaceInfo->getTraversalCost(neighborNode->getCurrentState(),m_stateEnd->getState(),false));
                neighborNode->setParentNode(currentNode);
                if(!neighborNode->isOpen())
                {
                    neighborNode->hasBeenOpened(true);
                    openSet.insert(neighborNode);
                }
            }
        }
    }

    return solutionVector;
}

std::vector<state_space::State*> AStarBase::solve()
{
    std::vector<state_space::State*> solutionVector;
    return solutionVector;

    /*
    GraphNode startNode;
    startNode.setCurrentState(m_stateBegin->getState()->getClone());

    GraphNode endNode;
    endNode.setCurrentState(m_stateEnd->getState()->getClone());

    std::multiset<GraphNode> openSet;
    std::multiset<GraphNode>::iterator openSetIT;

    std::unordered_set<const state_space::State*> closedSet;
    openSet.insert(startNode);

    GraphNode beginNode = *openSet.begin();

    while(!openSet.empty())
    {
        openSetIT = openSet.begin();
        GraphNode currentNode = *openSetIT;
        ++openSetIT;
        for(;openSetIT != openSet.end(); openSetIT++)
        {
            GraphNode evalNode = *openSetIT;
            if((evalNode.getFValue() < currentNode.getFValue()) || ((evalNode.getFValue() == currentNode.getFValue()) &&
                                                                  evalNode.getHValue() < currentNode.getHValue()))
            {
                currentNode = evalNode;
            }
        }

        openSet.erase(currentNode);
        closedSet.insert(currentNode.getCurrentState());

        if(currentNode.getCurrentState() == m_stateEnd->getState())
            return solutionVector;

        //get the appropriate neighbors
        std::vector<state_space::State*> neighbors = m_spaceInfo->getStateSpace()->getNeighboringStates(currentNode.getCurrentState());
        for(int i = 0; i < neighbors.size(); i++)
        {
            GraphNode neighborNode;
            state_space::State* neighborState = neighbors.at(i);
            neighborNode.setCurrentState(neighborState);
            //An edge valid check ensures that the current state is valid, and the connection between states is valid
            if(!m_spaceInfo->isEdgeValid(neighborState,currentNode.getCurrentState()) || closedSet.count(neighborState))
                continue;

            //movement cost to go from the current node to the neighboring node
            double movementCost = m_spaceInfo->getTraversalCost(currentNode.getCurrentState(),neighborNode.getCurrentState());
            //the total movement cost is therefore equal to the previous nodes cost plus the cost to move
            double parentCost = currentNode.getGValue(planners_graph::GRelationship::G_PARENT);
            double totalCost = parentCost + movementCost;

            //if the new path to neighbor is shorter || neighbor is not in OPEN
            if(totalCost < neighborNode.getGValue() || openSet.count(neighborNode) == 0)
            {
                neighborNode.updateGValue(parentCost,movementCost);
                neighborNode.updateHValue(m_spaceInfo->getTraversalCost(neighborNode.getCurrentState(),m_stateEnd->getState(),false));
                neighborNode.setParentNode(&currentNode);
                if(!neighborNode.isVisted())
                {
                    neighborNode.hasBeenVisited(true);
                    openSet.insert(neighborNode);
                }
                }
        }
    }
    */
}

void AStarBase::setPlanningParameters(state_space::GoalState* begin, state_space::GoalState* end)
{
    m_stateBegin = begin;
    m_stateEnd = end;
}

} //end of namespace planners_graph
} //end of namespace mace
