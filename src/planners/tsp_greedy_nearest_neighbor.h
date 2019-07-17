#ifndef TSP_GREEDY_NEAREST_NEIGHBOR_H
#define TSP_GREEDY_NEAREST_NEIGHBOR_H

#include <iostream>
#include <vector>
#include <limits>

#include "planners_global.h"
#include "planners.h"
#include "base/pose/position_interface.h"
#include "base/pose/cartesian_position_2D.h"

namespace mace {
namespace planners{

template <class T>
class PLANNERSSHARED_EXPORT TSP_GreedyNearestNeighbor : public Planners
{
    //need to make sure it is of a base type that we can understand so that we may call the overloaded virtual
    //distance function
    //static_assert(std::is_base_of<pose::AbstractPosition, T>::value,"T must be a descendant of AbstractPosition");
public:
    TSP_GreedyNearestNeighbor();

    void updateSites(const std::vector<T> &sites);

    void clearSites();

    double computeTourLength(const std::vector<T*> tour);

    void logTour(const std::vector<T*> tour);

    virtual double executeTSP(const T &start, std::vector<T> &tour);
protected:
    std::vector<T*> copy_sites();
    std::vector<T*> copy_sites(std::vector<T> &tour);

private:
    std::vector<T> m_siteNodes;
};

} //end of namespace planners
} //end of namespace mace

#endif // TSP_GREEDY_NEAREST_NEIGHBOR_H
