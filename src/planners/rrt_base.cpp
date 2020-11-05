#include "rrt_base.h"

namespace mace {
namespace planners_sampling{

void RRTBase::setPlanningParameters(state_space::GoalState *begin, state_space::GoalState *end)
{
    m_stateBegin = begin;
    m_stateEnd = end;
}

//!
//! \brief RRTBase::solve function that initiates the solve routine of the planner
//!
std::vector<state_space::State*> RRTBase::solve()
{

    RootNode* finalNode = nullptr;
    bool solutionFound = false;
    std::vector<state_space::State*> path;

    /**
     * 1. Create the root node of the search based on the starting state and
     * insert into the allocated tree structure.
     */
    RootNode* start = new RootNode(*m_stateBegin->getState());
    m_nnStrategy->add(start);


    while(true){
        //std::this_thread::sleep_for(std::chrono::milliseconds(10));

        RootNode* sampleNode = new RootNode(m_spaceInfo->getStateSpace());
        //get the state from the node so that we can update this in memory when sampling
        state_space::State* sampleState = sampleNode->getCurrentState();

        // 2. Sample a state from the state space
        if((m_stateEnd != nullptr) && (m_RNG.uniform01() < goalProbability))
            sampleState = m_stateEnd->getState(); //m_stateEnd->sampleGoal(sampleState); //KEN FIX THIS SAMPLING IDEA
        else
            m_samplingStrategy->sampleUniform(sampleState);

        //
        // 3. Find the closet other node in the tree and determine the distance to the node
        //
        RootNode* closestNode = m_nnStrategy->nearest(sampleNode);
        state_space::State* closestState = closestNode->getCurrentState();
        double distance = m_spaceInfo->distanceBetween(closestState,sampleState);

        //
        // 4. Determine if the sampled point is within the appropriate distance threshold.
        // If not, interpolate between the states if possible. If not, drop the sample and
        // move on. If so, continue.
        //
        if(distance > maxBranchLength)
        {
            //do the interpretation
            state_space::State* interpolatedState;
//            bool validity = m_spaceInfo->getStateSpace()->interpolateStates(closestState,sampleState,maxBranchLength/distance, &interpolatedState);
            m_spaceInfo->getStateSpace()->interpolateStates(closestState,sampleState,maxBranchLength/distance, &interpolatedState);

            //Set the samplestate to newly interpolated and set in node.
            sampleNode->setCurrentState(*interpolatedState);
            sampleState = sampleNode->getCurrentState();
            delete interpolatedState;

        }

        if(m_CB)
            m_CB->cbiPlanner_SampledState(sampleState);

        //
        // 5. Check that 1)State is valid and collision free, 2)Path edge is valid sampled at desired intervals
        // related to the aircraft size
        //
        if(m_spaceInfo->isEdgeValid(closestState,sampleState))
        {
            if(m_CB)
                m_CB->cbiPlanner_NewConnection(sampleState, closestState);

            //
            // 5a. At this point the sampled state is clearly valid
            //


            sampleNode->setParentNode(closestNode);
            m_nnStrategy->add(sampleNode);

            //
            // 5b. Check if this satisfies the goal criteria
            //
            solutionFound = m_stateEnd->isGoalSatisfied(sampleState);
            if(solutionFound == true)
            {
                finalNode = sampleNode;
                break;
            }
        }
        else
        {
            delete sampleNode;
        }
    } //end of while loop

    if(finalNode != nullptr)
    {
        path.push_back(m_spaceInfo->copyState(m_stateEnd->getState()));
        while(finalNode != nullptr)
        {
            path.push_back(m_spaceInfo->copyState(finalNode->getCurrentState()));
            finalNode = finalNode->getParentNode();
        }
        std::reverse(path.begin(),path.end());
    }

    for(unsigned int i = 0; i < path.size(); i++)
    {
        std::string str = std::to_string(path.at(i)->stateAs<mace::pose::CartesianPosition_2D>()->getXPosition()) + "," + std::to_string(path.at(i)->stateAs<mace::pose::CartesianPosition_2D>()->getYPosition());
//        mLog->debug(str);
    }

    PathReduction newReduction(m_spaceInfo);
    newReduction.recursiveShortening(path);

    for(unsigned int i = 0; i < path.size(); i++)
    {
        std::string str = std::to_string(path.at(i)->stateAs<mace::pose::CartesianPosition_2D>()->getXPosition()) + "," + std::to_string(path.at(i)->stateAs<mace::pose::CartesianPosition_2D>()->getYPosition());
//        mLog->debug(str);
    }


    std::vector<RootNode*> vec = m_nnStrategy->getData();
    for(unsigned int i = 0; i < vec.size(); i++)
    {
        delete vec.at(i);
    }
    m_nnStrategy->clear();

    return path;
}

void RRTBase::setGoalProbability(const double &probability)
{
    goalProbability = probability;
}

double RRTBase::getGoalProbability() const{
    return this->goalProbability;
}

void RRTBase::setMaxBranchLength(const double &length)
{
    maxBranchLength = length;
}

double RRTBase::getMaxBranchLength() const
{
    return this->maxBranchLength;
}

} //end of namespace planners_sampling
} //end of namespace mace
