#include "costmap_base_layer.h"

namespace mace{
namespace costmap{
Costmap_BaseLayer::Costmap_BaseLayer(const std::string &layerName):
    Costmap2D(NO_INFORMATION)
{

}

Costmap_BaseLayer::Costmap_BaseLayer(const std::string &layerName, const uint8_t &fill_value, const double &x_length, const double &y_length, const double &resolution,
                                     const pose::CartesianPosition_2D &position, const double &rotation):
    Costmap2D(fill_value, x_length, y_length, resolution, position, rotation)
{

}

void Costmap_BaseLayer::touch(double x, double y, double *min_x, double *min_y, double *max_x, double *max_y)
{
    *min_x = std::min(x, *min_x);
    *min_y = std::min(y, *min_y);
    *max_x = std::max(x, *max_x);
    *max_y = std::max(y, *max_y);
}

void Costmap_BaseLayer::useExtraBounds(double *min_x, double *min_y, double *max_x, double *max_y)
{
    if (!has_extra_bounds_)
        return;

    *min_x = std::min(extra_min_x_, *min_x);
    *min_y = std::min(extra_min_y_, *min_y);
    *max_x = std::max(extra_max_x_, *max_x);
    *max_y = std::max(extra_max_y_, *max_y);
    extra_min_x_ = 1e6;
    extra_min_y_ = 1e6;
    extra_max_x_ = -1e6;
    extra_max_y_ = -1e6;
    has_extra_bounds_ = false;
}

void Costmap_BaseLayer::updateWithMax(Costmap2D& costGrid, const unsigned int &minXIndex, const unsigned int &minYIndex, const unsigned int &maxXIndex, const unsigned int &maxYIndex)
{
    if (!isLayerEnabled())
      return;

    unsigned int startX = std::max<unsigned int>(0,minXIndex), startY = std::max<unsigned int>(0,minYIndex);
    unsigned int endX = std::min<unsigned int>((this->xSize-1),maxXIndex), endY = std::min<unsigned int>((this->ySize-1),maxYIndex);

    for (unsigned int j = startY; j <= endY; j++)
    {
      for (unsigned int i = startX; i <= endX; i++)
      {
        uint8_t* data = costGrid.getCellByPosIndex(i,j);

        if (*data == NO_INFORMATION){
          continue;
        }
        if (*data == NO_INFORMATION || *data < *this->getCellByPosIndex(i,j))
            *data = *this->getCellByPosIndex(i,j);
      }
    }
}

void Costmap_BaseLayer::updateWithTrueOverwrite(Costmap2D& costGrid, const unsigned int &minXIndex, const unsigned int &minYIndex, const unsigned int &maxXIndex, const unsigned int &maxYIndex)
{
    if (!isLayerEnabled())
      return;

    for (size_t j = minYIndex; j < maxYIndex; j++)
    {
      for (size_t i = minXIndex; i < maxXIndex; i++)
      {
          uint8_t* data = costGrid.getCellByPosIndex(static_cast<unsigned int>(i),static_cast<unsigned int>(j));
          *data = *this->getCellByPosIndex(static_cast<unsigned int>(i),static_cast<unsigned int>(j));
      }
    }
}

void Costmap_BaseLayer::updateWithOverwrite(Costmap2D& costGrid, const unsigned int &minXIndex, const unsigned int &minYIndex, const unsigned int &maxXIndex, const unsigned int &maxYIndex)
{
    if (!isLayerEnabled())
      return;

    /*
    for (size_t j = minYIndex; j < maxYIndex; j++)
    {
      for (size_t i = minXIndex; i < maxXIndex; i++)
      {
        if (costmap_[it] != NO_INFORMATION)
          master[it] = costmap_[it];
      }
    }
    */
}

void Costmap_BaseLayer::updateWithAddition(Costmap2D& costGrid, const unsigned int &minXIndex, const unsigned int &minYIndex, const unsigned int &maxXIndex, const unsigned int &maxYIndex)
{
    if (!isLayerEnabled())
      return;

    /*
    unsigned char* master_array = master_grid.getCharMap();
    unsigned int span = master_grid.getSizeInCellsX();

    for (size_t j = minYIndex; j < maxYIndex; j++)
    {
      unsigned int it = j * span + min_i;
      for (size_t i = minXIndex; i < maxXIndex; i++)
      {
        if (costmap_[it] == NO_INFORMATION){
          it++;
          continue;
        }

        unsigned char old_cost = master_array[it];
        if (old_cost == NO_INFORMATION)
          master_array[it] = costmap_[it];
        else
        {
          int sum = old_cost + costmap_[it];
          if (sum >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
              master_array[it] = costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1;
          else
              master_array[it] = sum;
        }
        it++;
      }
    }
    */
}

} //end of namespace maps
} //end of namespace mace
