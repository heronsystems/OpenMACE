#ifndef COSTMAP_BASE_LAYER_H
#define COSTMAP_BASE_LAYER_H

#include <string>
#include "common/common.h"
#include "../costmap_2d.h"

#include "base/geometry/polygon_cartesian.h"
#include "../../iterators/line_map_iterator.h"

namespace mace{
namespace costmap{

class Costmap_BaseLayer : public Costmap2D
{
public:
    Costmap_BaseLayer(const std::string &layerName);

    Costmap_BaseLayer(const std::string &layerName, const uint8_t &fill_value, const double &x_length = 10.0, const double &y_length = 10.0, const double &resolution = 0.5,
                      const pose::CartesianPosition_2D &position = pose::CartesianPosition_2D(), const double &rotation = 0.0);

    Costmap_BaseLayer(const std::string &layerName, const uint8_t &fill_value,
               const double &x_min, const double &x_max,
               const double &y_min, const double &y_max,
               const double &resolution,
               const pose::CartesianPosition_2D &position = pose::CartesianPosition_2D(),
               const double &rotation = 0.0);

    virtual ~Costmap_BaseLayer() = default;

    void initialize(const std::string &layerName);

    virtual void updateCosts(Costmap2D& master_grid, unsigned int minXIndex, unsigned int minYIndex, unsigned int maxXIndex, unsigned int maxYIndex)
    {
        UNUSED(master_grid); UNUSED(minXIndex); UNUSED(minYIndex); UNUSED(maxXIndex); UNUSED(maxYIndex);
    }

public:
    virtual void outlineBoundary(const geometry::Polygon_Cartesian &boundary, const uint8_t &fill_value)
    {
        //for each line segmenet of the boundary
        if(boundary.isValidPolygon())
        {
            size_t vertexIndex = 1;
            uint8_t* value = nullptr;

            for(;vertexIndex < boundary.polygonSize(); vertexIndex++)
            {
                geometry::CartesianPosition_2D endVertex = boundary.at(vertexIndex);
                geometry::CartesianPosition_2D startVertex = boundary.at(vertexIndex-1);
                mace::maps::LineMapIterator newIterator(this,startVertex,endVertex);
//                std::cout<<"Start: "<<startVertex<<std::endl;
//                std::cout<<"End: "<<endVertex<<std::endl;

                for(;!newIterator.isPastEnd();++newIterator)
                {
                    value = (*this)[static_cast<unsigned int>(*newIterator)];
                    if(value != nullptr)
                        *value = fill_value;
                }
            }

            geometry::CartesianPosition_2D endVertex = boundary.at(0);
            geometry::CartesianPosition_2D startVertex = boundary.at(vertexIndex-1);
            mace::maps::LineMapIterator newIterator(this,startVertex,endVertex);
            for(;!newIterator.isPastEnd();++newIterator)
            {
                value = (*this)[static_cast<unsigned int>(*newIterator)];
                if(value != nullptr)
                    *value = fill_value;
            }
        }
    }

    void fillPolygon(const geometry::Polygon_Cartesian &polygon, const uint8_t &fill_value)
    {
        UNUSED(polygon); UNUSED(fill_value);
    }

protected:

    void updateWithTrueOverwrite(Costmap2D& costGrid, const unsigned int &minXIndex, const unsigned int &minYIndex, const unsigned int &maxXIndex, const unsigned int &maxYIndex);

    void updateWithOverwrite(Costmap2D& costGrid, const unsigned int &minXIndex, const unsigned int &minYIndex, const unsigned int &maxXIndex, const unsigned int &maxYIndex);

    void updateWithMax(Costmap2D& costGrid, const unsigned int &minXIndex, const unsigned int &minYIndex, const unsigned int &maxXIndex, const unsigned int &maxYIndex);

    void updateWithAddition(Costmap2D& costGrid, const unsigned int &minXIndex, const unsigned int &minYIndex, const unsigned int &maxXIndex, const unsigned int &maxYIndex);

    void touch(double x, double y, double* min_x, double* min_y, double* max_x, double* max_y);

    void useExtraBounds(double* min_x, double* min_y, double* max_x, double* max_y);

    bool has_extra_bounds_;

protected:
    double extra_min_x_, extra_max_x_, extra_min_y_, extra_max_y_;
};

} //end of namespace costmap
} //end of namespace mace

#endif // COSTMAP_BASE_LAYER_H
