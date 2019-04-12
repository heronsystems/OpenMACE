#ifndef NEAREST_NEIGHBOR_ABSTRACT_H
#define NEAREST_NEIGHBOR_ABSTRACT_H

#include <vector>
#include <functional>

namespace mace{
namespace nn{

template <class T>
class NearestNeighborAbstract {

public:

    typedef std::function<double(const T &, const T &)> DistanceFunction;

    NearestNeighborAbstract() = default;

    virtual ~NearestNeighborAbstract() = default;

    virtual void setDistanceFunction(const DistanceFunction &distFun) {
        m_distanceFunction = distFun;
    }

    const DistanceFunction &getDistanceFunction() const {
        return m_distanceFunction;
    }

    virtual bool reportsSortedResults() const = 0;

    virtual void clear() = 0;

    virtual void add(const T &data) = 0;

    virtual void add(const std::vector<T> &data)

    {

        for (auto it = data.begin(); it != data.end(); ++it)

            add(*it);

    }

    virtual bool remove(const T &data) = 0;

    virtual T nearest(const T &data) const = 0;

    virtual void nearestK(const T &data, const std::size_t k, std::vector<T> &nn) const = 0;

    virtual void nearestR(const T &data, const double &radius, std::vector<T> &nn) const = 0;

    virtual std::size_t size() const = 0;

    virtual void list(std::vector<T> &data) const = 0;

    virtual std::vector<T> getData() const = 0;


protected:
    DistanceFunction m_distanceFunction;

};

} //end of namespace nn
} //end of namespace mace

#endif // NEAREST_NEIGHBOR_ABSTRACT_H
