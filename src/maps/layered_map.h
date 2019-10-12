#ifndef LAYERED_MAP_H
#define LAYERED_MAP_H

#include <map>
#include <string>
#include <unordered_map>

#include "base_grid_map.h"

#include "bounded_2D_grid.h"
#include "iterators/circle_map_iterator.h"
#include "iterators/polygon_map_iterator.h"
#include "iterators/grid_map_iterator.h"
#include "occupancy_definition.h"
#include "data_2d_grid.h"
#include "map_cell.h"

#include <common/logging/macelog.h>

namespace mace {
namespace maps {

class LayeredMap
{
public:

    //!
    //! \brief LayeredMap Constructor
    //!
    LayeredMap()
    {
        // TODO: Initialize layered map to something?
    }

    //!
    //! \brief LayeredMap Constructor
    //! \param map Map to initialize
    //!
    LayeredMap(const std::unordered_map<std::string, BaseGridMap*> map)
    {
        m_layeredMap = map;
    }

    // TODO: Set lambda functions to call when something changes in here


    void addMapLayer(const std::string &layerName);

    //!
    //! \brief updateMapLayer If a map layer exists, update with provided pointer. If not, insert the new pointer
    //! \param layerName Layer to update
    //! \param mapLayer Pointer to map layer
    //!
    void updateMapLayer(const std::string &layerName, BaseGridMap* mapLayer);

    //!
    //! \brief getMapLayer Get map layer corresponding to key
    //! \param layerName Layer name key
    //! \param mapLayer Reference to map layer pointer
    //! \return True for if map layer exists, false otherwise
    //!
    BaseGridMap* getMapLayer(const std::string &layerName);

    //!
    //! \brief getLayerSize Get the size of all the underlying map layers
    //! \param xLength Length in the x direction
    //! \param yLength Length in the y direction
    //! \param xGridResolution X grid resolution
    //! \param yGridResolution Y grid resolution
    //! \return True for success, false otherwise
    //!
    bool getLayerSize(double &xSize, double &ySize, double &xGridResolution, double &yGridResolution);

    //!
    //! \brief getRotationAngleDegrees Get the rotation of the grid from standard Cartesian x/y
    //! \return Rotation angle in degrees
    //!
    double getRotationAngleDegrees();

    //!
    //! \brief setRotationAngleDegrees Set the rotation angle for all underlying grid layers
    //! \param angle Rotation angle in degrees
    //!
    void setRotationAngleDegrees(const double &angle);

    //!
    //! \brief expandLayersToFit Given a polygonal boundary, expand the underlying grid layers to fit the given boundary
    //! \param boundary Boundary to expand to
    //!
//    void expandLayersToFit(const mace::geometry::Polygon_2DC &boundary);

    //!
    //! \brief updateGridSize Update the underlying map layers size to the given dimensions. NOTE: The values given are assumed to be in a standard cartesian coordinate frame (i.e. not rotated)
    //! \param minX Minimum x value
    //! \param maxX Maximum x value
    //! \param minY Minimum y value
    //! \param maxY Maximum y value
    //! \param xRes X resolution
    //! \param yRes Y Resolution
    //!
    bool updateGridSize(const double &minX, const double &maxX, const double &minY, const double &maxY, const double &x_res, const double &y_res);

    //!
    //! \brief updateGridSizeByLength Update the underlying map layers size to the given lengths
    //! \param xLength X length
    //! \param yLength Y length
    //! \param xRes X resoltion
    //! \param yRes Y resolution
    //!
    bool updateGridSizeByLength(const double &xLength, const double &yLength, const double &xRes, const double &yRes);

    //!
    //! \brief setXResolution Set x resolution for all underlying map layers
    //! \param xRes X resolution
    //!
    void setXResolution(const double &xRes);

    //!
    //! \brief setYResolution Set y resolution for all underlying map layers
    //! \param yRes Y resolution
    //!
    void setYResolution(const double &yRes);

    //!
    //! \brief updateResolution Set x and y resolution for all underlying map layers
    //! \param xRes X resolution
    //! \param yRes Y resolution
    //!
    void updateResolution(const double &xRes, const double &yRes);

    //!
    //! \brief getXResolution Get x resolution
    //! \return X resolution
    //!
    double getXResolution();

    //!
    //! \brief getYResolution Get y resolution
    //! \return Y resolution
    //!
    double getYResolution();

    //!
    //! \brief getLayeredMap
    //! \return
    //!
    std::unordered_map<std::string, BaseGridMap*> getLayeredMap() {
        return m_layeredMap;
    }

private:

    //!
    //! \brief staticCallbackFunction_UpdateGridSize Static callback method for grid resizing
    //! \param p Pointer to the callback interface
    //! \param mapSize New map size data
    //!
    static void staticCallbackFunction_UpdateGridSize(void *p, mace::maps::mapSize &mapSize)
    {
        ((LayeredMap*)p)->callbackUpdateGridSize(mapSize);
    }

    //!
    //! \brief callbackUpdateGridSize Private callback method for grid resizing
    //! \param mapSize New map size
    //!
    void callbackUpdateGridSize(const mace::maps::mapSize &mapSize) {
        for(auto layer : m_layeredMap) {
            layer.second->updateGridSize(mapSize.x_min, mapSize.x_max, mapSize.y_min, mapSize.y_max, mapSize.x_res, mapSize.y_res);
        }
    }

private:

    //!
    //! \brief m_layeredMap Container for map layers
    //!
    std::unordered_map<std::string, BaseGridMap*> m_layeredMap;
};

} //end of namespace maps
} //end of namespace mace

#endif // LAYERED_MAP_H
