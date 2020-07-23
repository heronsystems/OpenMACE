#include "tsp_greedy_nearest_neighbor.h"

namespace mace {
namespace planners {

template <class T>
TSP_GreedyNearestNeighbor<T>::TSP_GreedyNearestNeighbor():
    Planners()
{

}

template <class T>
void TSP_GreedyNearestNeighbor<T>::updateSites(const std::vector<T> &sites)
{
    m_siteNodes = sites;
}

template <class T>
void TSP_GreedyNearestNeighbor<T>::clearSites()
{
    m_siteNodes.clear();
    m_siteNodes.shrink_to_fit();
}

template <class T>
double TSP_GreedyNearestNeighbor<T>::computeTourLength(const std::vector<T*> tour)
{
    double tourCost = 0.0;
    size_t tourSize = tour.size();
    if(tourSize > 1){
        for(size_t i = 0; i < (tourSize - 1); i++)
        {
            tourCost += tour[i]->distanceTo(tour[i+1]);
        }
    }
    return tourCost;
}

template <class T>
double TSP_GreedyNearestNeighbor<T>::executeTSP(const T &start, std::vector<T> &tour)
{
    double tourCost = 0.0;

    size_t siteSize = m_siteNodes.size();

    tour.clear();
    tour.reserve(siteSize + 1);
    tour.push_back(start);

    std::vector<T*> siteList = copy_sites();

    while(siteList.size() != 0)
    {
        int baseIndex = tour.size() - 1;
        double currentMinCost = std::numeric_limits<double>::max();
        int currentMinIndex = 0;
        size_t searchSize = siteList.size();

        for(size_t i = 0; i < searchSize; i++)
        {
            //compute the cost to go from the current point to all the remaining points in the list
            //in this case the cost is just the euclidian distance as defined by the distance
            //function from the abstract position class.
            double distanceCost = tour[baseIndex].distanceTo(siteList[i]);
            if(distanceCost < currentMinCost)
            {
                currentMinCost = distanceCost;
                currentMinIndex = i;
            }
        }
        tourCost += currentMinCost;
        tour.push_back(*siteList[currentMinIndex]);
        siteList.erase(siteList.begin() + currentMinIndex);
    }
    return tourCost;
}


template <class T>
void TSP_GreedyNearestNeighbor<T>::logTour(const std::vector<T*> tour)
{
//    mLog->debug("Logging the tour");

    size_t size = tour.size();
    for(unsigned int i = 0; i < size; i++)
    {
        std::stringstream buffer;
        buffer << tour[i];
//        mLog->info(buffer.str());
    }
}

template <class T>
std::vector<T*> TSP_GreedyNearestNeighbor<T>::copy_sites()
{

    int size = m_siteNodes.size();
    std::vector<T*> copy;
    copy.reserve(size);

    for (int i = 0; i < size; i++)
    {
        copy.push_back(&m_siteNodes[i]);
    }

    return copy;
}

template <class T>
std::vector<T*> TSP_GreedyNearestNeighbor<T>::copy_sites(std::vector<T> &tour)
{

    int size = tour.size();
    std::vector<T*> copy;
    copy.reserve(size);

    for (int i = 0; i < size; i++)
    {
        copy.push_back(&tour[i]);
    }

    return copy;
}


template class TSP_GreedyNearestNeighbor<pose::CartesianPosition_2D>;


} //end of namespace planners
} //end of namespace mace
