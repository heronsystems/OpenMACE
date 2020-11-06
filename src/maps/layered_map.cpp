#include "layered_map.h"

namespace mace {
namespace maps {

void LayeredMap::addMapLayer(const std::string &layerName)
{
    BaseGridMap* tmpLayer = new BaseGridMap();
    double xSize, ySize, xRes, yRes;
    if(this->getLayerSize(xSize, ySize, xRes, yRes)) {
        tmpLayer->updateGridSizeByLength(xSize, ySize, xRes, yRes);
    }
    this->updateMapLayer(layerName, tmpLayer);
}

//!
//! \brief updateMapLayer If a map layer exists, update with provided pointer. If not, insert the new pointer
//! \param layerName Layer to update
//! \param mapLayer Pointer to map layer
//!
void LayeredMap::updateMapLayer(const std::string &layerName, BaseGridMap* mapLayer)
{
    mapLayer->connectUpdatedGridSizeCallback(LayeredMap::staticCallbackFunction_UpdateGridSize, this);

    if(m_layeredMap.find(layerName) == m_layeredMap.end()) {
        m_layeredMap.insert(std::make_pair(layerName, mapLayer));
    }
    else {
        MaceLog::Yellow("Layer already exists. Overwriting");
        m_layeredMap[layerName] = mapLayer;
    }
}



//!
//! \brief getMapLayer Get map layer corresponding to key
//! \param layerName Layer name key
//! \param mapLayer Reference to map layer pointer
//! \return True for if map layer exists, false otherwise
//!
BaseGridMap* LayeredMap::getMapLayer(const std::string &layerName)
{
    if(m_layeredMap.find(layerName) != m_layeredMap.end()) {
//        mapLayer = m_layeredMap[layerName];
        return m_layeredMap[layerName];
//        return true;
    }
    else {
        MaceLog::Yellow("Layer does not exist.");
        return nullptr;
    }    
}

//!
//! \brief getLayerSize Get the size of all the underlying map layers
//! \param xLength Length in the x direction
//! \param yLength Length in the y direction
//! \param xGridResolution X grid resolution
//! \param yGridResolution Y grid resolution
//! \return True for success, false otherwise
//!
bool LayeredMap::getLayerSize(double &xSize, double &ySize, double &xGridResolution, double &yGridResolution)
{
    xSize = 0;
    ySize = 0;
    xGridResolution = 0;
    yGridResolution = 0;
    if(m_layeredMap.size() != 0) {
        mace::maps::BaseGridMap* tmpMapLayer = m_layeredMap.begin()->second;
        xSize = tmpMapLayer->getSizeX();
        ySize = tmpMapLayer->getSizeY();
        xGridResolution = tmpMapLayer->getXResolution();
        yGridResolution = tmpMapLayer->getYResolution();
        return true;
    }

    return false;
}

//!
//! \brief getRotationAngleDegrees Get the rotation of the grid from standard Cartesian x/y
//! \return Rotation angle in degrees
//!
double LayeredMap::getRotationAngleDegrees()
{
    if(m_layeredMap.size() != 0) {
        mace::maps::BaseGridMap* tmpMapLayer = m_layeredMap.begin()->second;
        return tmpMapLayer->getRotationAngleDegrees();
    }

    return 0.0;
}

//!
//! \brief setRotationAngleDegrees Set the rotation angle for all underlying grid layers
//! \param angle Rotation angle in degrees
//!
void LayeredMap::setRotationAngleDegrees(const double &angle)
{
    for(auto layer : m_layeredMap) {
        layer.second->updateGridRotation(angle);
    }
}

//!
//! \brief expandLayersToFit Given a polygonal boundary, expand the underlying grid layers to fit the given boundary
//! \param boundary Boundary to expand to
//!
//void LayeredMap::expandLayersToFit(const mace::geometry::Polygon_2DC &boundary)
//{
//    double xMin, xMax, yMin, yMax;
//    boundary.getBoundingValues(xMin, xMax, yMin, yMax);
////    for(auto layer : m_layeredMap) {
//        m_layeredMap.begin()->second->updateGridSize(xMin, xMax, yMin, yMax, m_layeredMap.begin()->second->getXResolution(), m_layeredMap.begin()->second->getYResolution());
////    }
//}

//!
//! \brief updateGridSize Update the underlying map layers size to the given dimensions
//! \param minX Minimum x value
//! \param maxX Maximum x value
//! \param minY Minimum y value
//! \param maxY Maximum y value
//! \param xRes X resolution
//! \param yRes Y Resolution
//!
bool LayeredMap::updateGridSize(const double &minX, const double &maxX, const double &minY, const double &maxY, const double &x_res, const double &y_res) {
    m_layeredMap.begin()->second->updateGridSize(minX, maxX, minY, maxY, x_res, y_res);
    return true;
}

//!
//! \brief updateGridSizeByLength Update the underlying map layers size to the given lengths
//! \param xLength X length
//! \param yLength Y length
//! \param xRes X resoltion
//! \param yRes Y resolution
//!
bool LayeredMap::updateGridSizeByLength(const double &xLength, const double &yLength, const double &xRes, const double &yRes)
{
    m_layeredMap.begin()->second->updateGridSizeByLength(xLength, yLength, xRes, yRes);
    return true;
}

//!
//! \brief setXResolution Set x resolution for all underlying map layers
//! \param xRes X resolution
//!
void LayeredMap::setXResolution(const double &xRes)
{
//    for(auto layer : m_layeredMap) {
        m_layeredMap.begin()->second->setXResolution(xRes);
//    }
}

//!
//! \brief setYResolution Set y resolution for all underlying map layers
//! \param yRes Y resolution
//!
void LayeredMap::setYResolution(const double &yRes)
{
//    for(auto layer : m_layeredMap) {
        m_layeredMap.begin()->second->setYResolution(yRes);
//    }
}

//!
//! \brief updateResolution Set x and y resolution for all underlying map layers
//! \param xRes X resolution
//! \param yRes Y resolution
//!
void LayeredMap::updateResolution(const double &xRes, const double &yRes)
{
//    for(auto layer : m_layeredMap) {
        m_layeredMap.begin()->second->updateResolution(xRes, yRes);
//    }
}

//!
//! \brief getXResolution Get x resolution
//! \return X resolution
//!
double LayeredMap::getXResolution()
{
    if(m_layeredMap.size() != 0) {
        mace::maps::BaseGridMap* tmpMapLayer = m_layeredMap.begin()->second;
        return tmpMapLayer->getXResolution();
    }

    // Return a negative value if there is no layers in the map:
    MaceLog::Yellow("No layers in map. No resolution available");
    return -1.0;
}

//!
//! \brief getYResolution Get y resolution
//! \return Y resolution
//!
double LayeredMap::getYResolution()
{
    if(m_layeredMap.size() != 0) {
        mace::maps::BaseGridMap* tmpMapLayer = m_layeredMap.begin()->second;
        return tmpMapLayer->getYResolution();
    }

    // Return a negative value if there is no layers in the map:
    MaceLog::Yellow("No layers in map. No resolution available");
    return -1.0;
}


} //end of namespace maps
} //end of namepsace mace
