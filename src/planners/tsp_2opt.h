#ifndef TSP_2OPT_H
#define TSP_2OPT_H

#include <cmath>

#include "planners_global.h"
#include "tsp_greedy_nearest_neighbor.h"

namespace mace {
namespace planners{

template <class T>
class PLANNERSSHARED_EXPORT TSP_2OPT : public TSP_GreedyNearestNeighbor<T>
{
public:
    TSP_2OPT();

public:
    double execute2OPT(const T &start, std::vector<T> &tour, const bool &greedyFirst = true);

    double executeTSP(const T &start, std::vector<T> &tour) override;

private:
    void performSwap(const int &start, const int &end, std::vector<T*> &tour);

};

} //end of namespace planners
} //end of namespace mace
#endif // TSP_2OPT_H
