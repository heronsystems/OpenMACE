#include "octomap_wrapper.h"
namespace mace{
namespace maps{

OctomapWrapper::OctomapWrapper(const double &treeResolution, const OctomapSensorDefinition &sensorProperties):
    treeResolution(treeResolution),
    m_Tree(nullptr),
    m_Map(nullptr),
    m_sensorProperties(nullptr),
    m_projectionProperties(nullptr)
{
    m_sensorProperties = new OctomapSensorDefinition();
    m_projectionProperties = new Octomap2DProjectionDefinition();

    m_Tree = new octomap::OcTree(sensorProperties.getTreeResolution());
    m_Tree->enableChangeDetection(true);

    OccupiedResult fillValue = OccupiedResult::UNKNOWN;
    m_Map = new maps::Data2DGrid<OccupiedResult>(&fillValue);

    updateSensorProperties(*m_sensorProperties);
}


OctomapWrapper::~OctomapWrapper()
{
    delete m_sensorProperties;
    delete m_projectionProperties;

    delete m_Tree;
    delete m_Map;
}

bool OctomapWrapper::updateSensorProperties(const OctomapSensorDefinition &sensorProperties)
{
    m_Tree->setResolution(sensorProperties.getTreeResolution());
    treeResolution = m_Tree->getResolution();

    m_Tree->setProbHit(sensorProperties.getProbHit());
    m_Tree->setProbMiss(sensorProperties.getProbMiss());
    m_Tree->setClampingThresMax(sensorProperties.getThreshMax());
    m_Tree->setClampingThresMin(sensorProperties.getThreshMin());

    std::string oldLoad = m_sensorProperties->getInitialLoadFile();

    m_sensorProperties->updateProperties(sensorProperties);

    if((sensorProperties.getInitialLoadFile() != oldLoad) && (!sensorProperties.getInitialLoadFile().empty()))
    {
        //this implies that there is a different file we have been told to load, let us handle that
        loadOctreeFromBT(m_sensorProperties->getInitialLoadFile());
        return true;
    }

    return false;
}

bool OctomapWrapper::updateProjectionProperties(const Octomap2DProjectionDefinition &projectionProperties)
{
    UNUSED(projectionProperties);
    return false;
}

void OctomapWrapper::updateFromPointCloud(octomap::Pointcloud *pc, const pose::CartesianPosition_3D &position)
{
    octomap::point3d sensorOrigin(static_cast<float>(position.getXPosition()),
                                  static_cast<float>(position.getYPosition()),
                                  static_cast<float>(position.getZPosition()));

    m_Tree->insertPointCloud(*pc,sensorOrigin);

    //Update the depth of the tree
    treeDepth = m_Tree->getTreeDepth();
    maxTreeDepth = treeDepth;

    //The following function is already called when calling the insertPointCloud function, this would be inefficient
    //As it would perform the ray trace operation twice. It may be better at some future date to modify the octomap
    //library to merely deliver the changed values and keys on insertion and/or store as members.
    //m_Tree->computeUpdate(*pc,origin.trans(),freeKeySet,occupiedKeySet,m_sensorProperties->getMaxRange());
    if(m_Tree->numChangesDetected() > 0)
    {
        if(enabled2DProjection)
        {
            updateMapContinuity();
            for (octomap::KeyBoolMap::const_iterator it = m_Tree->changedKeysBegin(), end = m_Tree->changedKeysEnd(); it != end; ++it)
            {
                octomap::OcTreeKey key = it->first;
                octomap::point3d point = m_Tree->keyToCoord(key,maxTreeDepth);
                bool occupied = m_Tree->isNodeOccupied(m_Tree->search(key,m_Tree->getTreeDepth()));
                updateMapOccupancyRecursiveCheck(point.x(),point.y(),maxTreeDepth,occupied);
            }
        }
        m_Tree->resetChangeDetection();
    }
}

void OctomapWrapper::updateFromPointCloud(octomap::Pointcloud *pc, const pose::CartesianPosition_3D &position, const pose::Rotation_3D &orientation)
{
    octomap::pose6d origin(position.getXPosition(),position.getYPosition(),position.getZPosition(),
                           orientation.getRoll(),orientation.getPitch(),orientation.getYaw());

    octomap::point3d sensorOrigin;

    m_Tree->insertPointCloud(*pc,sensorOrigin,origin);
    //m_Tree->insertPointCloud(*pc,sensorOrigin,origin,m_sensorProperties->getMaxRange());

    //Update the depth of the tree
    treeDepth = m_Tree->getTreeDepth();
    maxTreeDepth = treeDepth;

    //The following function is already called when calling the insertPointCloud function, this would be inefficient
    //As it would perform the ray trace operation twice. It may be better at some future date to modify the octomap
    //library to merely deliver the changed values and keys on insertion and/or store as members.
    //m_Tree->computeUpdate(*pc,origin.trans(),freeKeySet,occupiedKeySet,m_sensorProperties->getMaxRange());
    if(m_Tree->numChangesDetected() > 0)
    {
        if(enabled2DProjection)
        {
            updateMapContinuity();
            for (octomap::KeyBoolMap::const_iterator it = m_Tree->changedKeysBegin(), end = m_Tree->changedKeysEnd(); it != end; ++it)
            {
                octomap::OcTreeKey key = it->first;
                octomap::point3d point = m_Tree->keyToCoord(key,maxTreeDepth);
                bool occupied = m_Tree->isNodeOccupied(m_Tree->search(key,m_Tree->getTreeDepth()));
                updateMapOccupancyRecursiveCheck(point.x(),point.y(),maxTreeDepth,occupied);
            }
        }
        m_Tree->resetChangeDetection();
    }
}

void OctomapWrapper::updateFromLaserScan(octomap::Pointcloud *pc, const pose::CartesianPosition_3D &position, const pose::Rotation_3D &orientation)
{
    //Update the depth of the tree
    treeDepth = m_Tree->getTreeDepth();
    maxTreeDepth = treeDepth;

    if(pc->size() > 0)
    {
        octomap::pose6d origin(position.getXPosition(),position.getYPosition(),position.getZPosition(),
                               orientation.getRoll(),orientation.getPitch(),orientation.getYaw());

        octomap::Pointcloud* copy = new octomap::Pointcloud(*pc);
        octomap::ScanGraph scan;
        scan.addNode(copy,origin);
        octomap::ScanGraph::iterator it;
        for (it = scan.begin(); it != scan.end(); it++) {
            m_Tree->insertPointCloud(**it, m_sensorProperties->getMaxRange());
        }

        //The following function is already called when calling the insertPointCloud function, this would be inefficient
        //As it would perform the ray trace operation twice. It may be better at some future date to modify the octomap
        //library to merely deliver the changed values and keys on insertion and/or store as members.
        //m_Tree->computeUpdate(*pc,origin.trans(),freeKeySet,occupiedKeySet,m_sensorProperties->getMaxRange());
        if(m_Tree->numChangesDetected() > 0)
        {
            if(enabled2DProjection)
            {
                updateMapContinuity();
                for (octomap::KeyBoolMap::const_iterator it = m_Tree->changedKeysBegin(), end = m_Tree->changedKeysEnd(); it != end; ++it)
                {
                    octomap::OcTreeKey key = it->first;
                    octomap::point3d point = m_Tree->keyToCoord(key,maxTreeDepth);
                    bool occupied = m_Tree->isNodeOccupied(m_Tree->search(key,m_Tree->getTreeDepth()));
                    updateMapOccupancyRecursiveCheck(point.x(),point.y(),maxTreeDepth,occupied);

                }
            }
            m_Tree->resetChangeDetection();
        }
    }
}

bool OctomapWrapper::is2DProjectionEnabled() const
{
    return this->enabled2DProjection;
}

bool OctomapWrapper::is2DTrackingChanges() const
{
    return this->enabled2DTrackingChanges;
}

void OctomapWrapper::set2DProjection(const bool enable)
{
    this->enabled2DProjection = enable;
    if(this->enabled2DProjection)
    {
        updateMapContinuity();
        updateEntireMapFromTree();
    }
}

void OctomapWrapper::set2DTrackingChanges(const bool enable)
{
    this->enabled2DTrackingChanges = enable;
    if(!enable) //if we are no longer interested in tracking changes, clear the queue
        this->changesIn2DMap.clear();
}

bool OctomapWrapper::loadOctreeFromBT(const std::string &path)
{
    std::string suffix = path.substr(path.length()-3, 3);
    if (suffix== ".bt"){
        if (!m_Tree->readBinary(path)){
            return false;
        }
    } else if (suffix == ".ot"){
        octomap::AbstractOcTree* tree = octomap::AbstractOcTree::read(path);
        if (!tree){
            return false;
        }
        if (m_Tree){
            delete m_Tree;
            m_Tree = nullptr;
        }
        m_Tree = dynamic_cast<octomap::OcTree*>(tree);
        if (!m_Tree){
            throw std::runtime_error("Could not read the supplied file.");
            return false;
        }

    } else{
        return false;
    }

    double minX, minY, minZ, maxX, maxY, maxZ;

    m_Tree->getMetricMin(minX, minY, minZ);
    m_Tree->getMetricMax(maxX, maxY, maxZ);

    treeDepth = m_Tree->getTreeDepth();
    maxTreeDepth = treeDepth;
    treeResolution = m_Tree->getResolution();
    keyBBXMin[0] = m_Tree->coordToKey(minX);
    keyBBXMin[1] = m_Tree->coordToKey(minY);
    keyBBXMin[2] = m_Tree->coordToKey(minZ);

    keyBBXMax[0] = m_Tree->coordToKey(maxX);
    keyBBXMax[1] = m_Tree->coordToKey(maxY);
    keyBBXMax[2] = m_Tree->coordToKey(maxZ);

    if(enabled2DProjection){
        this->updateMapContinuity();
        this->updateEntireMapFromTree();
    }

    return true;
}

void OctomapWrapper::getTreeDimensions(double &minX, double &maxX, double &minY, double &maxY, double &minZ, double &maxZ)
{
    m_Tree->getMetricMin(minX, minY, minZ);
    m_Tree->getMetricMax(maxX, maxY, maxZ);
}

void OctomapWrapper::updateMapContinuity()
{
    double minX, minY, minZ, maxX, maxY, maxZ;

    m_Tree->getMetricMin(minX, minY, minZ);
    m_Tree->getMetricMax(maxX, maxY, maxZ);

    double halfPaddedX = 0.5*m_sensorProperties->getMinSizeX();
    double halfPaddedY = 0.5*m_sensorProperties->getMinSizeY();
    minX = std::min(minX, -halfPaddedX);
    maxX = std::max(maxX, halfPaddedX);
    minY = std::min(minY, -halfPaddedY);
    maxY = std::max(maxY, halfPaddedY);
    octomap::point3d minPt(minX, minY, minZ);
    octomap::point3d maxPt(maxX, maxY, maxZ);
    octomap::OcTreeKey minKey = m_Tree->coordToKey(minPt, maxTreeDepth);
    octomap::OcTreeKey maxKey = m_Tree->coordToKey(maxPt, maxTreeDepth);
    UNUSED(maxKey);

    bool minKeyCheck = m_Tree->coordToKeyChecked(minPt, maxTreeDepth, paddedMinKey);
    bool maxKeycheck = m_Tree->coordToKeyChecked(maxPt, maxTreeDepth, paddedMaxKey);
    UNUSED(minKeyCheck); UNUSED(maxKeycheck);

    mapScaling = 1 << (treeDepth - maxTreeDepth);
    unsigned int width = (paddedMaxKey[0] - paddedMinKey[0])/mapScaling;
    unsigned int height = (paddedMaxKey[0] - paddedMinKey[0])/mapScaling;
    UNUSED(width); UNUSED(height);

    double gridRes = m_Tree->getNodeSize(m_Tree->getTreeDepth());

    bool resolutionChanged = false;
    if(m_projectionProperties->isMapLayerResolutionIndependent())
        resolutionChanged = m_Map->updateGridSize(minX,maxX,minY,maxY,m_Map->getXResolution(),m_Map->getYResolution());
    else
        resolutionChanged = m_Map->updateGridSize(minX,maxX,minY,maxY,gridRes,gridRes);

    if(resolutionChanged)
        this->updateEntireMapFromTree();

    int mapOriginX = minKey[0] - paddedMinKey[0];
    int mapOriginY = minKey[1] - paddedMinKey[1];
    UNUSED(mapOriginX); UNUSED(mapOriginY);

    // might not exactly be min / max of octree:
    octomap::point3d origin = m_Tree->keyToCoord(paddedMinKey, treeDepth);
    //    m_projectCompleteMap = (!m_incrementalUpdate || (std::abs(gridRes-m_gridmap.info.resolution) > 1e-6));
    //    m_gridmap.info.resolution = gridRes;
    pose::CartesianPosition_2D transformedOrigin(origin.x() - gridRes*0.5, origin.y() - gridRes*0.5);
    m_Map->updateOriginPosition(transformedOrigin);

    //if (maxTreeDepth != treeDepth){
    //    std::cout<<"Was this true"<<std::endl;
    //        m_gridmap.info.origin.position.x -= m_res/2.0;
    //        m_gridmap.info.origin.position.y -= m_res/2.0;
    //}
}

void OctomapWrapper::updateEntireMapFromTree()
{
    this->m_Map->clear();

    for (octomap::OcTree::iterator it = m_Tree->begin(maxTreeDepth), end = m_Tree->end(); it != end; ++it)
    {
        if(it.getZ() > 0.5){ //should filter for the ground here
            if(m_Tree->isNodeOccupied(*it))
            {
                updateMapOccupancyRecursiveCheck(it,true);
            }
            else
            {
                updateMapOccupancyRecursiveCheck(it,false);
            }
        }
    }
}

void OctomapWrapper::updateMapOccupancy(const octomap::OcTreeKey &key, const bool &occupancy)
{
    octomap::point3d point = m_Tree->keyToCoord(key,maxTreeDepth);
    OccupiedResult* ptr = m_Map->getCellByPos(point.x(),point.y());
    if(occupancy)
        *ptr = OccupiedResult::OCCUPIED;
    else
        *ptr = OccupiedResult::NOT_OCCUPIED;
}

void OctomapWrapper::updateMapOccupancyRecursiveCheck(const octomap::OcTree::iterator &it, const bool &occupancy)
{
    updateMapOccupancyRecursiveCheck(it.getX(),it.getY(),it.getDepth(),occupancy);
}

void OctomapWrapper::updateMapOccupancyRecursiveCheck(const double &xPos, const double &yPos, const unsigned int &depth, const bool &occupancy)
{
    if(depth == maxTreeDepth)
    {
        unsigned int currentIndex = m_Map->indexFromPos(xPos,yPos);
        OccupiedResult* newPtr = m_Map->getCellByPos(xPos,yPos);

        if(occupancy)
        {
            if(*newPtr != OccupiedResult::OCCUPIED) //means the value wasn't already previously occupied
            {
                *newPtr = OccupiedResult::OCCUPIED;
                if(enabled2DTrackingChanges)
                    this->changesIn2DMap.push_back(currentIndex);
            }
        }
        else //means we now want to mark the data as unoccupied
        {
            if((newPtr == nullptr) || (*newPtr == OccupiedResult::UNKNOWN))  //if we had no data before or the cell was null we can go ahead and immediately mark the space as unoccupied
            {
                if(enabled2DTrackingChanges)
                    this->changesIn2DMap.push_back(currentIndex);
                *newPtr = OccupiedResult::NOT_OCCUPIED;
            }
            else if(*newPtr == OccupiedResult::NOT_OCCUPIED)
            {
                //the cell was previously marked as unoccupied and it shall remain that way
                //we have this condition wrapped in the else if so it doesnt track changes
                //and doesn't perform the recursive search in the else condition
            }
            else
            {
                //this means this cell was previously occupied and now we are saying it is unoccuppied
                //we cannot necessarily just update the cell, we need to create a bbx and check all the z
                //conditions to see if it holds true

                //first we need to define a bbx
                double minX, minY, minZ, maxX, maxY, maxZ;

                m_Tree->getMetricMin(minX, minY, minZ);
                m_Tree->getMetricMax(maxX, maxY, maxZ);
                double width = m_Tree->getNodeSize(depth)/2.0;
                octomap::point3d minPt(xPos - width, yPos - width, minZ);
                octomap::point3d maxPt(xPos + width, yPos + width, maxZ);
                for(octomap::OcTree::leaf_bbx_iterator it = m_Tree->begin_leafs_bbx(minPt,maxPt,maxTreeDepth), end=m_Tree->end_leafs_bbx();
                    it!=end;++it)
                {
                    if(m_Tree->isNodeOccupied(*it)) //if another node is occupied in the tree somewhere different in height, we cannot mark the point in the map as unoccupied
                        return;
                }
                //if we have reached here, there are no nodes in the bbx that are occupied and therefore the node can be marked as free
                //and therefore as a result of this case condition we must mark the cell as having been changed
                if(enabled2DTrackingChanges)
                    this->changesIn2DMap.push_back(currentIndex);
                *newPtr = OccupiedResult::NOT_OCCUPIED;
            }

        }
    }else
    {
        //std::cout<<"Somehow we made it into here where the depth does not equal maxTreeDepth of the updateMapOccupancyRecursiveCheck."<<std::endl;
    }
}

maps::Data2DGrid<OccupiedResult>* OctomapWrapper::get2DOccupancyMap()
{
    return this->m_Map;
}

octomap::OcTree* OctomapWrapper::get3DOccupancyMap()
{
    return this->m_Tree;
}

std::vector<unsigned int> OctomapWrapper::getChanged2DIndices() const
{
    return this->changesIn2DMap;
}

void OctomapWrapper::reset2DChanges()
{
    this->changesIn2DMap.clear();
}

void OctomapWrapper::updateFreeNode(const octomap::OcTree::iterator &it)
{
    updateMapOccupancyRecursiveCheck(it,false);
}

void OctomapWrapper::updateOccupiedNode(const octomap::OcTree::iterator &it)
{
    updateMapOccupancyRecursiveCheck(it,true);
}

//void OctomapWrapper::filterGroundPlane(const octomap::Pointcloud& pc, octomap::Pointcloud& ground, octomap::Pointcloud& nonground) const
//{
//  ground.header = pc.header;
//  nonground.header = pc.header;

//  if (pc.size() < 50){
//    ROS_WARN("Pointcloud in OctomapServer too small, skipping ground plane extraction");
//    nonground = pc;
//  } else {
//    // plane detection for ground plane removal:
//    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

//    // Create the segmentation object and set up:
//    pcl::SACSegmentation<PCLPoint> seg;
//    seg.setOptimizeCoefficients (true);
//    // TODO: maybe a filtering based on the surface normals might be more robust / accurate?
//    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
//    seg.setMethodType(pcl::SAC_RANSAC);
//    seg.setMaxIterations(200);
//    seg.setDistanceThreshold (m_groundFilterDistance);
//    seg.setAxis(Eigen::Vector3f(0,0,1));
//    seg.setEpsAngle(m_groundFilterAngle);


//    PCLPointCloud cloud_filtered(pc);
//    // Create the filtering object
//    pcl::ExtractIndices<PCLPoint> extract;
//    bool groundPlaneFound = false;

//    while(cloud_filtered.size() > 10 && !groundPlaneFound){
//      seg.setInputCloud(cloud_filtered.makeShared());
//      seg.segment (*inliers, *coefficients);
//      if (inliers->indices.size () == 0){
//        break;
//      }

//      extract.setInputCloud(cloud_filtered.makeShared());
//      extract.setIndices(inliers);

//      if (std::abs(coefficients->values.at(3)) < m_groundFilterPlaneDistance){
//        extract.setNegative (false);
//        extract.filter (ground);

//        // remove ground points from full pointcloud:
//        // workaround for PCL bug:
//        if(inliers->indices.size() != cloud_filtered.size()){
//          extract.setNegative(true);
//          PCLPointCloud cloud_out;
//          extract.filter(cloud_out);
//          nonground += cloud_out;
//          cloud_filtered = cloud_out;
//        }

//        groundPlaneFound = true;
//      } else{
//        pcl::PointCloud<PCLPoint> cloud_out;
//        extract.setNegative (false);
//        extract.filter(cloud_out);
//        nonground +=cloud_out;
//        // debug
//        //            pcl::PCDWriter writer;
//        //            writer.write<PCLPoint>("nonground_plane.pcd",cloud_out, false);

//        // remove current plane from scan for next iteration:
//        // workaround for PCL bug:
//        if(inliers->indices.size() != cloud_filtered.size()){
//          extract.setNegative(true);
//          cloud_out.points.clear();
//          extract.filter(cloud_out);
//          cloud_filtered = cloud_out;
//        } else{
//          cloud_filtered.points.clear();
//        }
//      }

//    }
//    // TODO: also do this if overall starting pointcloud too small?
//    if (!groundPlaneFound){ // no plane found or remaining points too small

//      // do a rough fitlering on height to prevent spurious obstacles
//      pcl::PassThrough<PCLPoint> second_pass;
//      second_pass.setFilterFieldName("z");
//      second_pass.setFilterLimits(-m_groundFilterPlaneDistance, m_groundFilterPlaneDistance);
//      second_pass.setInputCloud(pc.makeShared());
//      second_pass.filter(ground);

//      second_pass.setFilterLimitsNegative (true);
//      second_pass.filter(nonground);
//    }

//  }
//}


OctomapSensorDefinition OctomapWrapper::getCurrentOctomapProperies() const
{
    return *this->m_sensorProperties;
}
} //end of namespace maps
} //end of namespace mace
