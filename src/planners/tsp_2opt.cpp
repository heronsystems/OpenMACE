#include "tsp_2opt.h"

namespace mace{
namespace planners {

template <class T>
TSP_2OPT<T>::TSP_2OPT():
    TSP_GreedyNearestNeighbor<T>()
{

}

template <class T>

void TSP_2OPT<T>::performSwap(const int &start, const int &end, std::vector<T*> &tour)
{
    std::vector<T*> oldValues;
    for(int i = start; i <= end; i++)
    {
        oldValues.push_back(tour[i]);
    }

    size_t size = oldValues.size();
    for(unsigned int i = 0; i < size; i++)
    {
        tour[end - i] = oldValues[i];
    }
}

template <class T>
double TSP_2OPT<T>::execute2OPT(const T &start, std::vector<T> &tour, const bool &greedyFirst)
{
    double currentLength, adjustedLength, oldLength;
    bool swapPerformed = true;
    std::vector<T*> data;

    if(greedyFirst)
    {
        currentLength = TSP_GreedyNearestNeighbor<T>::executeTSP(start, tour);
        data = TSP_GreedyNearestNeighbor<T>::copy_sites(tour);
    }
    else{
        data = TSP_GreedyNearestNeighbor<T>::copy_sites();
        currentLength = TSP_GreedyNearestNeighbor<T>::computeTourLength(data);
    }

    TSP_GreedyNearestNeighbor<T>::logTour(data);

    size_t size = data.size();

    //we can optimize this routine but for now lets just leave it
    while(swapPerformed)
    {
        swapPerformed = false;
        for (unsigned int i = 0; i < size ; i++)
        {
            //std::cout<<"Evalutaing points: "<<i<<","<<i+1<<std::endl;
            for (unsigned int j = i+2; (j+1) < size ; j++)
            {
                //std::cout<<"Sub-evalutaing points: "<<j<<","<<(j+1)<<std::endl;

                //We can do this because of the triangle inequality
                oldLength = data[i]->distanceTo(data[i+1]) + data[j]->distanceTo(data[j+1]);
                adjustedLength = data[i]->distanceTo(data[j]) + data[i+1]->distanceTo(data[j+1]);

                if(adjustedLength < oldLength)
                {
                    swapPerformed = true;
                    performSwap(i+1,j,data);
                    currentLength -= (oldLength - adjustedLength);
                    std::cout<<"The current length is: "<<currentLength<<std::endl;
                }
            }
        }
    }

    TSP_GreedyNearestNeighbor<T>::logTour(data);
    return currentLength;
}

template <class T>
double TSP_2OPT<T>::executeTSP(const T &start, std::vector<T> &tour)
{
    return execute2OPT(start,tour);
}

template class TSP_2OPT<pose::CartesianPosition_2D>;

} //end of namespace planners
} //end of namespace mace
