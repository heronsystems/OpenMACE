#ifndef NEAREST_NEIGHBOR_FLANN_H
#define NEAREST_NEIGHBOR_FLANN_H

#include <memory>
#include <utility>

#include "flann/flann.hpp"
#include "nearest_neighbor_abstract.h"
#include "rrt_node.h"

#include "base/state_space/state.h"

namespace mace {
namespace nn{

class TestState
{
public:
    TestState()
    {

    }

    void setValue(const double &v)
    {
        value = v;
    }

    double getValue() const
    {
        return value;
    }

    bool operator == (const TestState &rhs) const
    {
        if(this->value != rhs.value){
            return false;
        }
        return true;
    }

private:
    double value = 0.0;
};

template <typename T>
class DistanceFunctionFLANN
{
public:
    //this is only being done until type can be figured out
    using ElementType = T;
    using ResultType = double;

    DistanceFunctionFLANN(const typename NearestNeighborAbstract<T>::DistanceFunction &func):
        m_distanceFunction(func)
    {

    }

    template <typename IT1, typename IT2>
    double operator ()(IT1 a, IT2 b, size_t, ResultType = -1) const
    {
        return m_distanceFunction(*a,*b);
    }
protected:
    const typename NearestNeighborAbstract<T>::DistanceFunction &m_distanceFunction;
};


template <typename T, typename F = DistanceFunctionFLANN<T>>
class NearestNeighbor_FLANN : public NearestNeighborAbstract<T>
{
public:
    NearestNeighbor_FLANN(std::shared_ptr<flann::IndexParams> params):
        index_(nullptr), params_(std::move(params)), searchParams_(32, 0., true), dimension_(1)
    {

    }

    ~NearestNeighbor_FLANN() override
    {
        if(index_)
            delete index_;
    }

    void clear() override
    {
        if (index_)
        {
            delete index_;
            index_ = nullptr;
        }
        data_.clear();
    }

    bool reportsSortedResults() const override
    {
        return searchParams_.sorted;
    }

    void setDistanceFunction(const typename NearestNeighborAbstract<T>::DistanceFunction &distFun) override
    {
        NearestNeighborAbstract<T>::setDistanceFunction(distFun);
        rebuildIndex();
    }

    void add(const T &data) override
    {
        bool rebuild = index_ && (data_.size() + 1 > data_.capacity());

        if (rebuild)
            rebuildIndex(2 * data_.capacity());

        data_.push_back(data);
        const flann::Matrix<T> mat(&data_.back(), 1, dimension_);

        if (index_)
            index_->addPoints(mat, std::numeric_limits<float>::max() / size());
        else
            createIndex(mat);
    }
    void add(const std::vector<T> &data) override
    {
        unsigned int oldSize = data_.size();
        unsigned int newSize = oldSize + data.size();
        bool rebuild = index_ && (newSize > data_.capacity());

        if (rebuild)
            rebuildIndex(std::max(2 * oldSize, newSize));

        if (index_)
        {
            std::copy(data.begin(), data.end(), data_.begin() + oldSize);
            const flann::Matrix<T> mat(&data_[oldSize], data.size(), dimension_);
            index_->addPoints(mat, std::numeric_limits<float>::max() / size());
        }
        else
        {
            data_ = data;
            const flann::Matrix<T> mat(&data_[0], data_.size(), dimension_);
            createIndex(mat);
        }
    }
    bool remove(const T &data) override
    {
        if (!index_)
            return false;
        auto &it = const_cast<T &>(data);
        const flann::Matrix<T> query(&it, 1, dimension_);
        std::vector<std::vector<size_t>> indices(1);
        std::vector<std::vector<double>> dists(1);
        index_->knnSearch(query, indices, dists, 1, searchParams_);
        if (*index_->getPoint(indices[0][0]) == data)
        {
            index_->removePoint(indices[0][0]);
            rebuildIndex();
            return true;
        }
        return false;
    }

    std::size_t size() const override
    {
        return index_ ? index_->size() : 0;
    }


    T nearest(const T &data) const override
    {
        if (size() > 0)
        {
            auto &cast = const_cast<T &>(data);
            const flann::Matrix<T> query(&cast, 1, dimension_);
            std::vector<std::vector<size_t>> indices(1);
            std::vector<std::vector<double>> dists(1);
            index_->knnSearch(query, indices, dists, 1, searchParams_);
            return *index_->getPoint(indices[0][0]);
        }
    }

    void nearestK(const T &data, const std::size_t k, std::vector<T> &nn) const override
    {
        auto &cast = const_cast<T &>(data);
        const flann::Matrix<T> query(&cast, 1, dimension_);
        std::vector<std::vector<size_t>> indices;
        std::vector<std::vector<double>> dists;
        std::size_t numNN = index_ ? index_->knnSearch(query, indices, dists, k, searchParams_) : 0;
        nn.resize(numNN);
        for (std::size_t i = 0; i < k; ++i)
            nn[i] = *index_->getPoint(indices[0][i]);
    }

    void nearestR(const T &data, const double &radius, std::vector<T> &nn) const override
    {
        auto &cast = const_cast<T &>(data);
        flann::Matrix<T> query(&cast, 1, dimension_);
        std::vector<std::vector<size_t>> indices;
        std::vector<std::vector<double>> dists;
        int k = index_ ? index_->radiusSearch(query, indices, dists, radius, searchParams_) : 0;
        nn.resize(k);
        for (int i = 0; i < k; ++i)
            nn[i] = *index_->getPoint(indices[0][i]);
    }

    //!
    //! \brief list function is override from base NearestNeighbor that retrieves all of the elements
    //! stored in the KD-Tree of FLANN and returns.
    //! \param data object that is used to rethrn the information contained in the tree.
    //!
    void list(std::vector<T> &data) const override
    {
        std::size_t sz = size();
        if (sz == 0)
        {
            data.resize(0);
            return;
        }
        const T &obj = *index_->getPoint(0);
        int checks = searchParams_.checks;
        searchParams_.checks = size();
        nearestK(obj, sz, data);
        searchParams_.checks = checks;
    }

    std::vector<T> getData() const override
    {
        return this->data_;
    }

protected:

    void createIndex(const flann::Matrix<T> &mat)
    {
        index_ = new flann::Index<F>(mat, *params_, F(NearestNeighborAbstract<T>::m_distanceFunction));
        index_->buildIndex();
    }

    void rebuildIndex(unsigned int capacity = 0)
    {
        if (index_)
        {
            std::vector<T> data;
            list(data);
            clear();
            if (capacity != 0u)
                data_.reserve(capacity);
            add(data);
        }
    }
protected:
    std::vector<T> data_;

    flann::Index<F> *index_;

    std::shared_ptr<flann::IndexParams> params_;

    mutable flann::SearchParams searchParams_;

    unsigned int dimension_;
};


template <typename T, typename F = DistanceFunctionFLANN<T>>
class NearestNeighbor_FLANNLinear : public NearestNeighbor_FLANN<T, F>
{
public:
    NearestNeighbor_FLANNLinear():
        NearestNeighbor_FLANN<T,F>(std::shared_ptr<flann::LinearIndexParams>(new flann::LinearIndexParams()))
    {

    }
};

} //end of namespace nn
} //end of namespace mace


#endif // NEAREST_NEIGHBOR_FLANN_H
