#ifndef A_STAR_BASE_H
#define A_STAR_BASE_H

#include <queue>
#include <list>
#include <set>
#include <unordered_set>
#include <iostream>

#include "planners.h"
#include "graph_planning_node.h"
#include "maps/data_2d_grid.h"

namespace mace {
namespace planners_graph{

class AStarBase : public planners::Planners
{
public:
    AStarBase(const state_space::SpaceInformationPtr &spaceInfo):
        Planners(spaceInfo)
    {

    }
    virtual ~AStarBase() = default;

public:
    std::vector<state_space::State*> solve(maps::Data2DGrid<GraphNode> &stateGridData);

public:
    std::vector<state_space::State*> solve() override;

    void setPlanningParameters(state_space::GoalState* begin, state_space::GoalState* end) override;

    /*Requirements of the open/closed set data structures
     * 1) Add nodes to the set
     * 2) Remove nodes from the set
     * 3) Open set requires an ability to search it for the lowest F cost
     */
private:
    void retracePath(const GraphNode* start, const GraphNode* end, std::vector<state_space::State *> &path);

};

template<typename T, typename priority_t>
struct PriorityQueue {
  typedef std::pair<priority_t, T> PQElement;
  std::priority_queue<PQElement, std::vector<PQElement>,
                 std::greater<PQElement>> elements;

  inline bool empty() const {
     return elements.empty();
  }

  inline void put(T item, priority_t priority) {
    elements.emplace(priority, item);
  }

  T get() {
    T best_item = elements.top().second;
    elements.pop();
    return best_item;
  }
};

} //end of namespace planners_graph
} //end of namespace mace

#endif // A_STAR_BASE_H
