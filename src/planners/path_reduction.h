#ifndef PATH_REDUCTION_H
#define PATH_REDUCTION_H

#include <vector>
#include <list>
#include <iostream>

#include "base/state_space/space_information.h"
#include "base/pose/cartesian_position_2D.h"

namespace mace {
namespace planners_sampling{

class PathReduction
{
public:
    PathReduction(const state_space::SpaceInformationPtr &spaceInfo);
    void recursiveShortening(std::vector<state_space::State *> &path);

private:
    state_space::SpaceInformationPtr m_spaceInfo;
};

} //end of namespace mace
} //end of namespace planners_sampling
#endif // PATH_REDUCTION_H
