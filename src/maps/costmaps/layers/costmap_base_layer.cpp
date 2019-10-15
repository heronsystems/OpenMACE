#include "costmap_base_layer.h"

namespace mace{
namespace costmap{

Costmap_BaseLayer::Costmap_BaseLayer(const std::string &layerName):
    Costmap2D(NO_INFORMATION)
{

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

}

void Costmap_BaseLayer::updateWithOverwrite(Costmap2D& costGrid, const unsigned int &minXIndex, const unsigned int &minYIndex, const unsigned int &maxXIndex, const unsigned int &maxYIndex)
{

}

void Costmap_BaseLayer::updateWithAddition(Costmap2D& costGrid, const unsigned int &minXIndex, const unsigned int &minYIndex, const unsigned int &maxXIndex, const unsigned int &maxYIndex)
{

}

} //end of namespace maps
} //end of namespace mace
