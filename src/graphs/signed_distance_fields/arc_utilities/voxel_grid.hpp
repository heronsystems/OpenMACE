#ifndef VOXEL_GRID_HPP
#define VOXEL_GRID_HPP

#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <Eigen/Geometry>
#include <stdexcept>

#include "arc_helpers.hpp"

namespace VoxelGrid {

struct GRID_INDEX
{
    int64_t x;
    int64_t y;
    int64_t z;

    GRID_INDEX() : x(-1), y(-1), z(-1) {}

    GRID_INDEX(const int64_t in_x, const int64_t in_y, const int64_t in_z)
        : x(in_x), y(in_y), z(in_z) {}

    bool operator==(const GRID_INDEX& other) const
    {
        return (x == other.x && y == other.y && z == other.z);
    }

    friend std::ostream& operator<<(std::ostream& strm, const GRID_INDEX& index)
    {
        strm << "GRID_INDEX: (" << index.x << "," << index.y << "," << index.z << ")";
        return strm;
    }
};

template<typename T, typename BackingStore=std::vector<T>>
class VoxelGrid
{
protected:

    Eigen::Isometry3d origin_transform_;
    Eigen::Isometry3d inverse_origin_transform_;
    T default_value_;
    T oob_value_;
    BackingStore data_;
    double cell_x_size_;
    double cell_y_size_;
    double cell_z_size_;
    double inv_cell_x_size_;
    double inv_cell_y_size_;
    double inv_cell_z_size_;
    double x_size_;
    double y_size_;
    double z_size_;
    int64_t stride1_;
    int64_t stride2_;
    int64_t num_x_cells_;
    int64_t num_y_cells_;
    int64_t num_z_cells_;
    bool initialized_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    protected:

        int64_t GetDataIndex(const int64_t x_index,
                             const int64_t y_index,
                             const int64_t z_index) const
    {
        return (x_index * stride1_) + (y_index * stride2_) + z_index;
    }

    int64_t GetDataIndex(const GRID_INDEX& index) const
    {
        return (index.x * stride1_) + (index.y * stride2_) + index.z;
    }

    T& AccessIndex(const int64_t& data_index)
    {
        if ((data_index >= 0) && (data_index < (int64_t)data_.size()))
        {
            return data_[data_index];
        }
        else
        {
            throw std::out_of_range("data_index out of range");
        }
    }

    const T& AccessIndex(const int64_t& data_index) const
    {
        if ((data_index >= 0) && (data_index < (int64_t)data_.size()))
        {
            return data_[data_index];
        }
        else
        {
            throw std::out_of_range("data_index out of range");
        }
    }

    void SetContents(const T& value)
    {
        data_.clear();
        data_.resize(num_x_cells_ * num_y_cells_ * num_z_cells_, value);
    }

    void SafetyCheckSizes(const double cell_x_size,
                          const double cell_y_size,
                          const double cell_z_size,
                          const double x_size,
                          const double y_size,
                          const double z_size) const
    {
        if (cell_x_size <= 0.0)
        {
            throw std::invalid_argument("cell_x_size must be positive and non-zero");
        }
        if (std::isnan(cell_x_size))
        {
            throw std::invalid_argument("cell_x_size must not be NaN");
        }
        if (std::isinf(cell_x_size) != 0)
        {
            throw std::invalid_argument("cell_x_size must not be INF");
        }
        if (cell_y_size <= 0.0)
        {
            throw std::invalid_argument("cell_y_size must be positive and non-zero");
        }
        if (std::isnan(cell_y_size))
        {
            throw std::invalid_argument("cell_y_size must not be NaN");
        }
        if (std::isinf(cell_y_size) != 0)
        {
            throw std::invalid_argument("cell_y_size must not be INF");
        }
        if (cell_z_size <= 0.0)
        {
            throw std::invalid_argument("cell_z_size must be positive and non-zero");
        }
        if (std::isnan(cell_z_size))
        {
            throw std::invalid_argument("cell_z_size must not be NaN");
        }
        if (std::isinf(cell_z_size) != 0)
        {
            throw std::invalid_argument("cell_z_size must not be INF");
        }
        if (x_size <= 0.0)
        {
            throw std::invalid_argument("x_size must be positive and non-zero");
        }
        if (y_size <= 0.0)
        {
            throw std::invalid_argument("y_size must be positive and non-zero");
        }
        if (z_size <= 0.0)
        {
            throw std::invalid_argument("z_size must be positive and non-zero");
        }
        if (std::isnan(x_size))
        {
            throw std::invalid_argument("x_size must not be NaN");
        }
        if (std::isnan(y_size))
        {
            throw std::invalid_argument("y_size must not be NaN");
        }
        if (std::isnan(z_size))
        {
            throw std::invalid_argument("z_size must not be NaN");
        }
        if (std::isinf(x_size) != 0)
        {
            throw std::invalid_argument("x_size must not be INF");
        }
        if (std::isinf(y_size) != 0)
        {
            throw std::invalid_argument("y_size must not be INF");
        }
        if (std::isinf(z_size) != 0)
        {
            throw std::invalid_argument("z_size must not be INF");
        }
    }

    void SafetyCheckSizes(const double cell_x_size,
                          const double cell_y_size,
                          const double cell_z_size,
                          const int64_t num_x_cells,
                          const int64_t num_y_cells,
                          const int64_t num_z_cells) const
    {
        if (cell_x_size <= 0.0)
        {
            throw std::invalid_argument("cell_x_size must be positive and non-zero");
        }
        if (std::isnan(cell_x_size))
        {
            throw std::invalid_argument("cell_x_size must not be NaN");
        }
        if (std::isinf(cell_x_size) != 0)
        {
            throw std::invalid_argument("cell_x_size must not be INF");
        }
        if (cell_y_size <= 0.0)
        {
            throw std::invalid_argument("cell_y_size must be positive and non-zero");
        }
        if (std::isnan(cell_y_size))
        {
            throw std::invalid_argument("cell_y_size must not be NaN");
        }
        if (std::isinf(cell_y_size) != 0)
        {
            throw std::invalid_argument("cell_y_size must not be INF");
        }
        if (cell_z_size <= 0.0)
        {
            throw std::invalid_argument("cell_z_size must be positive and non-zero");
        }
        if (std::isnan(cell_z_size))
        {
            throw std::invalid_argument("cell_z_size must not be NaN");
        }
        if (std::isinf(cell_z_size) != 0)
        {
            throw std::invalid_argument("cell_z_size must not be INF");
        }
        if (num_x_cells <= 0)
        {
            throw std::invalid_argument("num_x_cells must be positive and non-zero");
        }
        if (num_y_cells <= 0)
        {
            throw std::invalid_argument("num_y_cells must be positive and non-zero");
        }
        if (num_z_cells <= 0)
        {
            throw std::invalid_argument("num_z_cells must be positive and non-zero");
        }
    }

    void CoreInitialize(const double cell_x_size,
                        const double cell_y_size,
                        const double cell_z_size,
                        const int64_t num_x_cells,
                        const int64_t num_y_cells,
                        const int64_t num_z_cells,
                        const T& default_value,
                        const T& oob_value)
    {
        SafetyCheckSizes(cell_x_size, cell_y_size, cell_z_size,
                         num_x_cells, num_y_cells, num_z_cells);
        cell_x_size_ = std::abs(cell_x_size);
        cell_y_size_ = std::abs(cell_y_size);
        cell_z_size_ = std::abs(cell_z_size);
        inv_cell_x_size_ = 1.0 / cell_x_size_;
        inv_cell_y_size_ = 1.0 / cell_y_size_;
        inv_cell_z_size_ = 1.0 / cell_z_size_;
        num_x_cells_ = num_x_cells;
        num_y_cells_ = num_y_cells;
        num_z_cells_ = num_z_cells;
        x_size_ = (double)num_x_cells_ * cell_x_size_;
        y_size_ = (double)num_y_cells_ * cell_y_size_;
        z_size_ = (double)num_z_cells_ * cell_z_size_;
        default_value_ = default_value;
        oob_value_ = oob_value;
        stride1_ = num_y_cells_ * num_z_cells_;
        stride2_ = num_z_cells_;
        SetContents(default_value_);
    }

public:

    VoxelGrid(const Eigen::Isometry3d& origin_transform,
              const double cell_size,
              const double x_size,
              const double y_size,
              double const z_size,
              const T& default_value)
    {
        Initialize(origin_transform, cell_size, cell_size, cell_size,
                   x_size, y_size, z_size, default_value, default_value);
    }

    VoxelGrid(const Eigen::Isometry3d& origin_transform,
              const double cell_size,
              const double x_size,
              const double y_size,
              const double z_size,
              const T& default_value,
              const T& oob_value)
    {
        Initialize(origin_transform, cell_size, cell_size, cell_size,
                   x_size, y_size, z_size, default_value, oob_value);
    }

    VoxelGrid(const Eigen::Isometry3d& origin_transform,
              const double cell_x_size,
              const double cell_y_size,
              const double cell_z_size,
              const double x_size,
              const double y_size,
              double const z_size,
              const T& default_value)
    {
        Initialize(origin_transform, cell_x_size, cell_y_size, cell_z_size,
                   x_size, y_size, z_size, default_value, default_value);
    }

    VoxelGrid(const Eigen::Isometry3d& origin_transform,
              const double cell_x_size,
              const double cell_y_size,
              const double cell_z_size,
              const double x_size,
              const double y_size,
              const double z_size,
              const T& default_value,
              const T& oob_value)
    {
        Initialize(origin_transform, cell_x_size, cell_y_size, cell_z_size,
                   x_size, y_size, z_size, default_value, oob_value);
    }

    VoxelGrid(const Eigen::Isometry3d& origin_transform,
              const double cell_size,
              const int64_t num_x_cells,
              const int64_t num_y_cells,
              const int64_t num_z_cells,
              const T& default_value)
    {
        Initialize(origin_transform, cell_size, cell_size, cell_size,
                   num_x_cells, num_y_cells, num_z_cells,
                   default_value, default_value);
    }

    VoxelGrid(const Eigen::Isometry3d& origin_transform,
              const double cell_size,
              const int64_t num_x_cells,
              const int64_t num_y_cells,
              const int64_t num_z_cells,
              const T& default_value,
              const T& oob_value)
    {
        Initialize(origin_transform, cell_size, cell_size, cell_size,
                   num_x_cells, num_y_cells, num_z_cells,
                   default_value, oob_value);
    }

    VoxelGrid(const Eigen::Isometry3d& origin_transform,
              const double cell_x_size,
              const double cell_y_size,
              const double cell_z_size,
              const int64_t num_x_cells,
              const int64_t num_y_cells,
              const int64_t num_z_cells,
              const T& default_value)
    {
        Initialize(origin_transform, cell_x_size, cell_y_size, cell_z_size,
                   num_x_cells, num_y_cells, num_z_cells,
                   default_value, default_value);
    }

    VoxelGrid(const Eigen::Isometry3d& origin_transform,
              const double cell_x_size,
              const double cell_y_size,
              const double cell_z_size,
              const int64_t num_x_cells,
              const int64_t num_y_cells,
              const int64_t num_z_cells,
              const T& default_value,
              const T& oob_value)
    {
        Initialize(origin_transform, cell_x_size, cell_y_size, cell_z_size,
                   num_x_cells, num_y_cells, num_z_cells, default_value, oob_value);
    }

    VoxelGrid(const double cell_size,
              const double x_size,
              const double y_size,
              const double z_size,
              const T& default_value)
    {
        Initialize(cell_size, cell_size, cell_size,
                   x_size, y_size, z_size, default_value, default_value);
    }

    VoxelGrid(const double cell_size,
              const double x_size,
              const double y_size,
              const double z_size,
              const T& default_value,
              const T& oob_value)
    {
        Initialize(cell_size, cell_size, cell_size,
                   x_size, y_size, z_size, default_value, oob_value);
    }

    VoxelGrid(const double cell_x_size,
              const double cell_y_size,
              const double cell_z_size,
              const double x_size,
              const double y_size,
              const double z_size,
              const T& default_value)
    {
        Initialize(cell_x_size, cell_y_size, cell_z_size,
                   x_size, y_size, z_size, default_value, default_value);
    }

    VoxelGrid(const double cell_x_size,
              const double cell_y_size,
              const double cell_z_size,
              const double x_size,
              const double y_size,
              const double z_size,
              const T& default_value,
              const T& oob_value)
    {
        Initialize(cell_x_size, cell_y_size, cell_z_size,
                   x_size, y_size, z_size, default_value, oob_value);
    }

    VoxelGrid(const double cell_size,
              const int64_t num_x_cells,
              const int64_t num_y_cells,
              const int64_t num_z_cells,
              const T& default_value)
    {
        Initialize(cell_size, cell_size, cell_size,
                   num_x_cells, num_y_cells, num_z_cells,
                   default_value, default_value);
    }

    VoxelGrid(const double cell_size,
              const int64_t num_x_cells,
              const int64_t num_y_cells,
              const int64_t num_z_cells,
              const T& default_value,
              const T& oob_value)
    {
        Initialize(cell_size, cell_size, cell_size,
                   num_x_cells, num_y_cells, num_z_cells,
                   default_value, oob_value);
    }

    VoxelGrid(const double cell_x_size,
              const double cell_y_size,
              const double cell_z_size,
              const int64_t num_x_cells,
              const int64_t num_y_cells,
              const int64_t num_z_cells,
              const T& default_value)
    {
        Initialize(cell_x_size, cell_y_size, cell_z_size,
                   num_x_cells, num_y_cells, num_z_cells,
                   default_value, default_value);
    }

    VoxelGrid(const double cell_x_size,
              const double cell_y_size,
              const double cell_z_size,
              const int64_t num_x_cells,
              const int64_t num_y_cells,
              const int64_t num_z_cells,
              const T& default_value,
              const T& oob_value)
    {
        Initialize(cell_x_size, cell_y_size, cell_z_size,
                   num_x_cells, num_y_cells, num_z_cells,
                   default_value, oob_value);
    }

    VoxelGrid()
    {
        origin_transform_ = Eigen::Isometry3d::Identity();
        inverse_origin_transform_ = origin_transform_.inverse();
        arc_helpers::RequireAlignment(origin_transform_, 16u);
        arc_helpers::RequireAlignment(inverse_origin_transform_, 16u);
        cell_x_size_ = 0.0;
        cell_y_size_ = 0.0;
        cell_z_size_ = 0.0;
        inv_cell_x_size_ = 0.0;
        inv_cell_y_size_ = 0.0;
        inv_cell_z_size_ = 0.0;
        x_size_ = 0.0;
        y_size_ = 0.0;
        z_size_ = 0.0;
        num_x_cells_ = 0;
        num_y_cells_ = 0;
        num_z_cells_ = 0;
        stride1_ = num_y_cells_ * num_z_cells_;
        stride2_ = num_z_cells_;
        data_.clear();
        initialized_ = false;
    }

    virtual ~VoxelGrid() {}

    virtual VoxelGrid<T, BackingStore>* Clone() const
    {
        return new VoxelGrid<T, BackingStore>(
                    static_cast<const VoxelGrid<T, BackingStore>&>(*this));
    }

    void Initialize(const Eigen::Isometry3d& origin_transform,
                    const double cell_x_size,
                    const double cell_y_size,
                    const double cell_z_size,
                    const double x_size,
                    const double y_size,
                    double const z_size,
                    const T& default_value,
                    const T& oob_value)
    {
        SafetyCheckSizes(cell_x_size, cell_y_size, cell_z_size,
                         x_size, y_size, z_size);
        const int64_t num_x_cells
                = (int64_t)(ceil(std::abs(x_size) / std::abs(cell_x_size)));
        const int64_t num_y_cells
                = (int64_t)(ceil(std::abs(y_size) / std::abs(cell_y_size)));
        const int64_t num_z_cells
                = (int64_t)(ceil(std::abs(z_size) / std::abs(cell_z_size)));
        Initialize(origin_transform, cell_x_size, cell_y_size, cell_z_size,
                   num_x_cells, num_y_cells, num_z_cells, default_value, oob_value);
    }

    void Initialize(const Eigen::Isometry3d& origin_transform,
                    const double cell_x_size,
                    const double cell_y_size,
                    const double cell_z_size,
                    const int64_t num_x_cells,
                    const int64_t num_y_cells,
                    const int64_t num_z_cells,
                    const T& default_value,
                    const T& oob_value)
    {
        SafetyCheckSizes(cell_x_size, cell_y_size, cell_z_size,
                         num_x_cells, num_y_cells, num_z_cells);
        CoreInitialize(cell_x_size, cell_y_size, cell_z_size,
                       num_x_cells, num_y_cells, num_z_cells,
                       default_value, oob_value);
        origin_transform_ = origin_transform;
        inverse_origin_transform_ = origin_transform_.inverse();
        //Ken disabling the eigen alignment requirements for the time being
        //        arc_helpers::RequireAlignment(origin_transform_, 16u);
        //        arc_helpers::RequireAlignment(inverse_origin_transform_, 16u);
        initialized_ = true;
    }

    void Initialize(const double cell_x_size,
                    const double cell_y_size,
                    const double cell_z_size,
                    const double x_size,
                    const double y_size,
                    double const z_size,
                    const T& default_value,
                    const T& oob_value)
    {
        SafetyCheckSizes(cell_x_size, cell_y_size, cell_z_size,
                         x_size, y_size, z_size);
        const int64_t num_x_cells
                = (int64_t)(ceil(std::abs(x_size) / std::abs(cell_x_size)));
        const int64_t num_y_cells
                = (int64_t)(ceil(std::abs(y_size) / std::abs(cell_y_size)));
        const int64_t num_z_cells
                = (int64_t)(ceil(std::abs(z_size) / std::abs(cell_z_size)));
        Initialize(cell_x_size, cell_y_size, cell_z_size,
                   num_x_cells, num_y_cells, num_z_cells,
                   default_value, oob_value);
    }

    void Initialize(const double cell_x_size,
                    const double cell_y_size,
                    const double cell_z_size,
                    const int64_t num_x_cells,
                    const int64_t num_y_cells,
                    const int64_t num_z_cells,
                    const T& default_value,
                    const T& oob_value)
    {
        SafetyCheckSizes(cell_x_size, cell_y_size, cell_z_size,
                         num_x_cells, num_y_cells, num_z_cells);
        CoreInitialize(cell_x_size, cell_y_size, cell_z_size,
                       num_x_cells, num_y_cells, num_z_cells,
                       default_value, oob_value);
        const Eigen::Translation3d origin_translation(
                    -x_size_ * 0.5, -y_size_ * 0.5, -z_size_ * 0.5);
        const Eigen::Isometry3d origin_transform
                = origin_translation * Eigen::Quaterniond::Identity();
        origin_transform_ = origin_transform;
        inverse_origin_transform_ = origin_transform_.inverse();
        //Ken disabling the eigen alignment requirements for the time being
        //        arc_helpers::RequireAlignment(origin_transform_, 16u);
        //        arc_helpers::RequireAlignment(inverse_origin_transform_, 16u);
        initialized_ = true;
    }

    bool IsInitialized() const
    {
        return initialized_;
    }

    void ResetWithDefault()
    {
        SetContents(default_value_);
    }

    void ResetWithNewValue(const T& new_value)
    {
        SetContents(new_value);
    }

    void ResetWithNewDefault(const T& new_default)
    {
        default_value_ = new_default;
        SetContents(default_value_);
    }

    bool IndexInBounds(const int64_t x_index,
                       const int64_t y_index,
                       const int64_t z_index) const
    {
        if (x_index >= 0 && y_index >= 0 && z_index >= 0 && x_index < num_x_cells_
                && y_index < num_y_cells_ && z_index < num_z_cells_)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    bool IndexInBounds(const GRID_INDEX& index) const
    {
        if (index.x >= 0 && index.y >= 0 && index.z >= 0 && index.x < num_x_cells_
                && index.y < num_y_cells_ && index.z < num_z_cells_)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    bool LocationInBounds(const double x,
                          const double y,
                          const double z) const
    {
        const GRID_INDEX index = LocationToGridIndex(x, y, z);
        return IndexInBounds(index);
    }

    bool LocationInBounds3d(const Eigen::Vector3d& location) const
    {
        const GRID_INDEX index = LocationToGridIndex3d(location);
        return IndexInBounds(index);
    }

    bool LocationInBounds4d(const Eigen::Vector4d& location) const
    {
        const GRID_INDEX index = LocationToGridIndex4d(location);
        return IndexInBounds(index);
    }

    std::pair<const T&, bool> GetImmutable3d(
            const Eigen::Vector3d& location) const
    {
        const GRID_INDEX index = LocationToGridIndex3d(location);
        if (IndexInBounds(index))
        {
            return GetImmutable(index);
        }
        else
        {
            return std::pair<const T&, bool>(oob_value_, false);
        }
    }

    std::pair<const T&, bool> GetImmutable4d(
            const Eigen::Vector4d& location) const
    {
        const GRID_INDEX index = LocationToGridIndex4d(location);
        if (IndexInBounds(index))
        {
            return GetImmutable(index);
        }
        else
        {
            return std::pair<const T&, bool>(oob_value_, false);
        }
    }

    std::pair<const T&, bool> GetImmutable(const double x,
                                           const double y,
                                           const double z) const
    {
        const Eigen::Vector4d location(x, y, z, 1.0);
        return GetImmutable4d(location);
    }

    std::pair<const T&, bool> GetImmutable(const GRID_INDEX& index) const
    {
        if (IndexInBounds(index))
        {
            return std::pair<const T&, bool>(AccessIndex(GetDataIndex(index)), true);
        }
        else
        {
            return std::pair<const T&, bool>(oob_value_, false);
        }
    }

    std::pair<const T&, bool> GetImmutable(const int64_t x_index,
                                           const int64_t y_index,
                                           const int64_t z_index) const
    {
        if (IndexInBounds(x_index, y_index, z_index))
        {
            return std::pair<const T&, bool>(
                        AccessIndex(GetDataIndex(x_index, y_index, z_index)), true);
        }
        else
        {
            return std::pair<const T&, bool>(oob_value_, false);
        }
    }

    virtual std::pair<T&, bool> GetMutable3d(const Eigen::Vector3d& location)
    {
        const GRID_INDEX index = LocationToGridIndex3d(location);
        if (IndexInBounds(index))
        {
            return GetMutable(index);
        }
        else
        {
            return std::pair<T&, bool>(oob_value_, false);
        }
    }

    virtual std::pair<T&, bool> GetMutable4d(const Eigen::Vector4d& location)
    {
        const GRID_INDEX index = LocationToGridIndex4d(location);
        if (IndexInBounds(index))
        {
            return GetMutable(index);
        }
        else
        {
            return std::pair<T&, bool>(oob_value_, false);
        }
    }

    virtual std::pair<T&, bool> GetMutable(const double x,
                                           const double y,
                                           const double z)
    {
        const Eigen::Vector4d location(x, y, z, 1.0);
        return GetMutable4d(location);
    }

    virtual std::pair<T&, bool> GetMutable(const GRID_INDEX& index)
    {
        if (IndexInBounds(index))
        {
            return std::pair<T&, bool>(AccessIndex(GetDataIndex(index)), true);
        }
        else
        {
            return std::pair<T&, bool>(oob_value_, false);
        }
    }

    virtual std::pair<T&, bool> GetMutable(const int64_t x_index,
                                           const int64_t y_index,
                                           const int64_t z_index)
    {
        if (IndexInBounds(x_index, y_index, z_index))
        {
            return std::pair<T&, bool>(
                        AccessIndex(GetDataIndex(x_index, y_index, z_index)), true);
        }
        else
        {
            return std::pair<T&, bool>(oob_value_, false);
        }
    }

    virtual bool SetValue3d(const Eigen::Vector3d& location,
                            const T& value)
    {
        const GRID_INDEX index = LocationToGridIndex3d(location);
        if (IndexInBounds(index))
        {
            return SetValue(index, value);
        }
        else
        {
            return false;
        }
    }

    virtual bool SetValue4d(const Eigen::Vector4d& location,
                            const T& value)
    {
        const GRID_INDEX index = LocationToGridIndex4d(location);
        if (IndexInBounds(index))
        {
            return SetValue(index, value);
        }
        else
        {
            return false;
        }
    }

    virtual bool SetValue(const double &x,
                          const double &y,
                          const double &z,
                          const T& value)
    {
        const Eigen::Vector4d location(x, y, z, 1.0);
        return SetValue4d(location, value);
    }

    virtual bool SetValue(const GRID_INDEX& index, const T& value)
    {
        if (IndexInBounds(index))
        {
            AccessIndex(GetDataIndex(index)) = value;
            return true;
        }
        else
        {
            return false;
        }
    }

    virtual bool SetValue(const int64_t x_index,
                          const int64_t y_index,
                          const int64_t z_index,
                          const T& value)
    {
        if (IndexInBounds(x_index, y_index, z_index))
        {
            AccessIndex(GetDataIndex(x_index, y_index, z_index)) = value;
            return true;
        }
        else
        {
            return false;
        }
    }

    virtual bool SetValue3d(const Eigen::Vector3d& location, T&& value)
    {
        const GRID_INDEX index = LocationToGridIndex3d(location);
        if (IndexInBounds(index))
        {
            return SetValue(index, value);
        }
        else
        {
            return false;
        }
    }

    virtual bool SetValue4d(const Eigen::Vector4d& location, T&& value)
    {
        const GRID_INDEX index = LocationToGridIndex4d(location);
        if (IndexInBounds(index))
        {
            return SetValue(index, value);
        }
        else
        {
            return false;
        }
    }

    virtual bool SetValue(const double &x,
                          const double &y,
                          const double &z,
                          T&& value)
    {
        const Eigen::Vector4d location(x, y, z, 1.0);
        return SetValue4d(location, value);
    }

    virtual bool SetValue(const GRID_INDEX& index, T&& value)
    {
        if (IndexInBounds(index))
        {
            AccessIndex(GetDataIndex(index)) = value;
            return true;
        }
        else
        {
            return false;
        }
    }

    virtual bool SetValue(const int64_t x_index,
                          const int64_t y_index,
                          const int64_t z_index,
                          T&& value)
    {
        if (IndexInBounds(x_index, y_index, z_index))
        {
            AccessIndex(GetDataIndex(x_index, y_index, z_index)) = value;
            return true;
        }
        else
        {
            return false;
        }
    }

    double GetXSize() const
    {
        return x_size_;
    }

    double GetYSize() const
    {
        return y_size_;
    }

    double GetZSize() const
    {
        return z_size_;
    }

    Eigen::Vector3d GetCellSizes() const
    {
        return Eigen::Vector3d(cell_x_size_, cell_y_size_, cell_z_size_);
    }

    T GetDefaultValue() const
    {
        return default_value_;
    }

    T GetOOBValue() const
    {
        return oob_value_;
    }

    void SetDefaultValue(const T& default_value)
    {
        default_value_ = default_value;
    }

    void SetOOBValue(const T& oob_value)
    {
        oob_value_ = oob_value;
    }

    int64_t GetNumXCells() const
    {
        return num_x_cells_;
    }

    int64_t GetNumYCells() const
    {
        return num_y_cells_;
    }

    int64_t GetNumZCells() const
    {
        return num_z_cells_;
    }

    const Eigen::Isometry3d& GetOriginTransform() const
    {
        return origin_transform_;
    }

    const Eigen::Isometry3d& GetInverseOriginTransform() const
    {
        return inverse_origin_transform_;
    }

    void UpdateOriginTransform(const Eigen::Isometry3d& origin_transform)
    {
        origin_transform_ = origin_transform;
        inverse_origin_transform_ = origin_transform_.inverse();
    }

    GRID_INDEX PointInFrameToGridIndex(const double x,
                                       const double y,
                                       const double z) const
    {
        const int64_t x_cell = (int64_t)(x * inv_cell_x_size_);
        const int64_t y_cell = (int64_t)(y * inv_cell_y_size_);
        const int64_t z_cell = (int64_t)(z * inv_cell_z_size_);
        return GRID_INDEX(x_cell, y_cell, z_cell);
    }

    GRID_INDEX PointInFrameToGridIndex3d(
            const Eigen::Vector3d& point_in_grid_frame) const
    {
        const int64_t x_cell = (int64_t)(point_in_grid_frame(0) * inv_cell_x_size_);
        const int64_t y_cell = (int64_t)(point_in_grid_frame(1) * inv_cell_y_size_);
        const int64_t z_cell = (int64_t)(point_in_grid_frame(2) * inv_cell_z_size_);
        return GRID_INDEX(x_cell, y_cell, z_cell);
    }

    GRID_INDEX PointInFrameToGridIndex4d(
            const Eigen::Vector4d& point_in_grid_frame) const
    {
        const int64_t x_cell = (int64_t)(point_in_grid_frame(0) * inv_cell_x_size_);
        const int64_t y_cell = (int64_t)(point_in_grid_frame(1) * inv_cell_y_size_);
        const int64_t z_cell = (int64_t)(point_in_grid_frame(2) * inv_cell_z_size_);
        return GRID_INDEX(x_cell, y_cell, z_cell);
    }

    GRID_INDEX LocationToGridIndex(const double x,
                                   const double y,
                                   const double z) const
    {
        const Eigen::Vector4d point(x, y, z, 1.0);
        return LocationToGridIndex4d(point);
    }

    GRID_INDEX LocationToGridIndex3d(const Eigen::Vector3d& location) const
    {
        const Eigen::Vector3d point_in_grid_frame
                = inverse_origin_transform_ * location;
        return PointInFrameToGridIndex(point_in_grid_frame.x(),
                                       point_in_grid_frame.y(),
                                       point_in_grid_frame.z());
    }

    GRID_INDEX LocationToGridIndex4d(const Eigen::Vector4d& location) const
    {
        const Eigen::Vector4d point_in_grid_frame
                = inverse_origin_transform_ * location;
        return PointInFrameToGridIndex(point_in_grid_frame(0),
                                       point_in_grid_frame(1),
                                       point_in_grid_frame(2));
    }

    Eigen::Vector4d GridIndexToLocation(const GRID_INDEX& index) const
    {
        const Eigen::Vector4d point_in_grid_frame(
                    cell_x_size_ * ((double)index.x + 0.5),
                    cell_y_size_ * ((double)index.y + 0.5),
                    cell_z_size_ * ((double)index.z + 0.5),
                    1.0);
        const Eigen::Vector4d point = origin_transform_ * point_in_grid_frame;
        return point;
    }

    Eigen::Vector4d GridIndexToLocation(const int64_t x_index,
                                        const int64_t y_index,
                                        const int64_t z_index) const
    {
        const Eigen::Vector4d point_in_grid_frame(
                    cell_x_size_ * ((double)x_index + 0.5),
                    cell_y_size_ * ((double)y_index + 0.5),
                    cell_z_size_ * ((double)z_index + 0.5),
                    1.0);
        const Eigen::Vector4d point = origin_transform_ * point_in_grid_frame;
        return point;
    }

    Eigen::Vector4d GridIndexToLocationGridFrame(
            const GRID_INDEX& index) const
    {
        const Eigen::Vector4d point_in_grid_frame(
                    cell_x_size_ * ((double)index.x + 0.5),
                    cell_y_size_ * ((double)index.y + 0.5),
                    cell_z_size_ * ((double)index.z + 0.5),
                    1.0);
        return point_in_grid_frame;
    }

    Eigen::Vector4d GridIndexToLocationGridFrame(
            const int64_t x_index, const int64_t y_index, const int64_t z_index) const
    {
        const Eigen::Vector4d point_in_grid_frame(
                    cell_x_size_ * ((double)x_index + 0.5),
                    cell_y_size_ * ((double)y_index + 0.5),
                    cell_z_size_ * ((double)z_index + 0.5),
                    1.0);
        return point_in_grid_frame;
    }

    BackingStore& GetMutableRawData()
    {
        return data_;
    }

    const BackingStore& GetImmutableRawData() const
    {
        return data_;
    }

    bool SetRawData(const BackingStore& data)
    {
        const int64_t expected_length
                = num_x_cells_ * num_y_cells_ * num_z_cells_;
        if ((int64_t)data.size() == expected_length)
        {
            data_ = data;
            return true;
        }
        else
        {
            std::cerr << "Failed to load internal data - expected "
                      << expected_length << " got " << data.size() << std::endl;
            return false;
        }
    }

    uint64_t HashDataIndex(const int64_t x_index,
                           const int64_t y_index,
                           const int64_t z_index) const
    {
        return (x_index * stride1_) + (y_index * stride2_) + z_index;
    }
};

} //end of namespace VoxelGrid

namespace std
{
template <>
struct hash<VoxelGrid::GRID_INDEX>
{
    std::size_t operator()(const VoxelGrid::GRID_INDEX& index) const
    {
        using std::size_t;
        using std::hash;
        return ((std::hash<int64_t>()(index.x)
                 ^ (std::hash<int64_t>()(index.y) << 1) >> 1)
                ^ (std::hash<int64_t>()(index.z) << 1));
    }
};
} //end of namespace std

#endif // VOXEL_GRID_HPP
