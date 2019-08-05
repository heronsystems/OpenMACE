#ifndef OCTOMAP_WRAPPER_H
#define OCTOMAP_WRAPPER_H

#include <iostream>

#include "octomap_sensor_definition.h"
#include "octomap_2d_projection_definition.h"

#include "octomap/OcTree.h"

#include "occupancy_definition.h"

#include "octomap/OcTreeIterator.hxx"


#include "data_2d_grid.h"

#include "base/pose/cartesian_position_3D.h"
#include "base/pose/rotation_3D.h"

#include "occupancy_definition.h"


namespace mace{
namespace maps{

class OctomapWrapper
{
public:
    OctomapWrapper(const double &treeResolution = 0.05, const OctomapSensorDefinition &sensorProperties = OctomapSensorDefinition());

    ~OctomapWrapper();

    bool is2DProjectionEnabled() const;

    bool is2DTrackingChanges() const;

    void set2DProjection(const bool enable);

    void set2DTrackingChanges(const bool enable);

public:
    bool loadOctreeFromBT(const std::string &path);

    void getTreeDimensions(double &minX, double &maxX, double &minY, double &maxY, double &minZ, double &maxZ);

    bool updateSensorProperties(const OctomapSensorDefinition &sensorProperties);

    bool updateProjectionProperties(const Octomap2DProjectionDefinition &projectionProperties);

    void updateFromPointCloud(octomap::Pointcloud *pc, const pose::CartesianPosition_3D &position);

    void updateFromPointCloud(octomap::Pointcloud *pc, const pose::CartesianPosition_3D &position, const pose::Rotation_3D &orientation);

    void updateFromLaserScan(octomap::Pointcloud* pc, const pose::CartesianPosition_3D &position = pose::CartesianPosition_3D(), const pose::Rotation_3D &orientation = mace::pose::Rotation_3D());

    OctomapSensorDefinition getCurrentOctomapProperies() const;

public:
    maps::Data2DGrid<OccupiedResult>* get2DOccupancyMap();
    octomap::OcTree* get3DOccupancyMap();

    std::vector<unsigned int> getChanged2DIndices() const;
    void reset2DChanges();

private:
    void updateMapContinuity();

    void updateEntireMapFromTree();

    void updateMapOccupancy(const octomap::OcTreeKey &key, const bool &occupancy);

    void updateMapOccupancyRecursiveCheck(const double &xPos, const double &yPos, const unsigned int &depth, const bool &occupancy);

    void updateMapOccupancyRecursiveCheck(const octomap::OcTree::iterator &it, const bool &occupancy);

private:
    //void filterGroundPlane(const octomap::Pointcloud& pc, octomap::Pointcloud& ground, octomap::Pointcloud& nonground);
    void updateOccupiedNode(const octomap::OcTree::iterator &it);
    void updateFreeNode(const octomap::OcTree::iterator &it);

private:
    inline void mapIndex(const octomap::OcTreeKey& key, unsigned int &rows, unsigned int &colums) const {
        rows = key[0] - paddedMinKey[0];
        colums = key[1] - paddedMinKey[1];
    }

private:
    bool enabled2DProjection = true;
    bool enabled2DTrackingChanges = false;
    bool enabledIndependentMapResolution = false;

    std::vector<unsigned int> changesIn2DMap;

    double treeResolution = 0.05;
    unsigned int treeDepth = 0;
    unsigned int maxTreeDepth = 0;

    unsigned int mapScaling = 0;

    octomap::OcTree* m_Tree;
    octomap::OcTreeKey keyBBXMin;
    octomap::OcTreeKey keyBBXMax;
    octomap::OcTreeKey paddedMinKey;
    octomap::OcTreeKey paddedMaxKey;

    maps::Data2DGrid<OccupiedResult>* m_Map;
    maps::Data2DGrid<unsigned int>* m_InflatedMap;

    OctomapSensorDefinition* m_sensorProperties;
    Octomap2DProjectionDefinition* m_projectionProperties;

};

} //end of namespace maps
} //end of namespace mace


#endif // OCTOMAP_WRAPPER_H
