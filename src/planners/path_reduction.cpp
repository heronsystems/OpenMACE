#include "path_reduction.h"

namespace mace {
namespace planners_sampling{

PathReduction::PathReduction(const state_space::SpaceInformationPtr &spaceInfo):
    m_spaceInfo(spaceInfo)
{

}

void PathReduction::recursiveShortening(std::vector<state_space::State*> &path)
{

    uint outerCount = 0;
    std::vector<state_space::State*>::iterator it;

    while(outerCount < path.size()-1)
    {
        std::vector<double> dV;
        dV.push_back(0.0);

        for(uint i = 1; i < path.size(); i++)
        {
            double value = m_spaceInfo->distanceBetween(path[i-1],path[i]);
            dV.push_back(value);
        }

        uint innerCount = outerCount + 2;
        while(innerCount < path.size())
        {
            double distance = 0.0;
            for(uint j = 0; j <= innerCount; j++)
                distance += dV[j];

            double newDistance = m_spaceInfo->distanceBetween(path[outerCount],path[innerCount]);
            if((newDistance <= distance) && m_spaceInfo->isEdgeValid(path[outerCount],path[innerCount]))
            {
                dV[innerCount] = newDistance;
                dV.erase(dV.begin() + outerCount + 1, dV.begin() + innerCount);
                for(it = path.begin() + outerCount + 1; it != path.begin() + innerCount; ++it)
                {
                    delete *it;
                    path.erase(it);
                }
            }
            else
            {
                innerCount++;
            }
        }
        outerCount++;
    }
}

} //end of namespace planners_sampling
} //end of namespace mace

