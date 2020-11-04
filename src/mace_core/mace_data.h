#ifndef I_MACE_DATA_H
#define I_MACE_DATA_H

#include <iostream>
#include <string>
#include <map>
#include <unordered_map>
#include <stdexcept>
#include <functional>
#include <mutex>
#include <list>
#include <vector>

#include <Eigen/StdVector>

#include "vehicle_data.h"

#include "observation_history.h"

#include "matrix_operations.h"

#include "topic.h"

#include "data_generic_item/data_generic_item_components.h"
#include "data_generic_command_item/command_item_components.h"

#include "data/system_description.h"
#include "data/mission_execution_state.h"

#include "maps/octomap_wrapper.h"
#include "maps/data_2d_grid.h"
#include "maps/layered_map.h"

#include "octomap/octomap.h"
#include "octomap/OcTree.h"

#include "base/pose/cartesian_position_3D.h"
#include "base/pose/rotation_3D.h"
#include "base/geometry/cell_2DC.h"

#include "data_generic_command_item/boundary_items/boundary_type.h"

#include "data/environment_time.h"

#include "common/logging/macelog.h"

namespace MaceCore
{

using BoundaryIdentifierType = uint8_t;

class MaceCore;

//!
//! \brief Object that is to hold all data that MACE modules may need
//!
//! It it intended that a single MaceData object will be created and passed to each module.
//! Therefore all of this objects methods are thread safe as they may be stimulated from multiple module's threads.
//!
//! The reason for a single data storage object over simply passing data in command methods was done for three reasons:
//!     1) Some modules may require the same data, so a centrailized container is simplier over distrubtiting idential data.
//!     2) Some data may be updated at a high rate, so changing one and calling a notify modules of new data is prefered over sending new data
//!     3) Commands may be marsheled at a different rate than they are generated, so a centralize data bin gives the module full access to data without over-invoking
//!
//! Only MaceCore object will be able of manipulating data inside this object, while any module can read data out of it.
//! If a module must write to this object it must issue an event to MaceCore first.
//!
//! Use of the MaceData object requires implementaiton of various Fuse_* methods.
//! These methods describe how to combine multiple observations and used to expose a continuous-time data-space with descrite-time observations.
//! The exact method in how this interpolation is done (pick closests, linear, more advanced methods) is left to the user of MaceData.
//!
class MACE_CORESHARED_EXPORT MaceData
{
    friend class MaceCore;

    static const uint64_t DEFAULT_MS_RECORD_TO_KEEP = 1000;

private:
    uint64_t m_MSTOKEEP;

public:


public:
    MaceData() :
        m_MSTOKEEP(DEFAULT_MS_RECORD_TO_KEEP), flagBoundaryVerts(false)
    {
        m_OctomapWrapper = new mace::maps::OctomapWrapper();
        m_LayeredMap_Local = std::make_shared<mace::maps::LayeredMap>();
    }

    MaceData(uint64_t historyToKeepInms) :
        m_MSTOKEEP(historyToKeepInms), flagBoundaryVerts(false)
    {
        m_OctomapWrapper = new mace::maps::OctomapWrapper();
        m_LayeredMap_Local = std::make_shared<mace::maps::LayeredMap>();
    }

    virtual ~MaceData()
    {
        //delete m_OctomapWrapper;
    }

    /////////////////////////////////////////////////////////
    /// DATA FUSION METHODS
    /////////////////////////////////////////////////////////


    //!
    //! \brief Abstract method to interpolate vehicle dynamics
    //!
    //! This method is to be implemented by the instantiator of MaceData
    //! \param time Time to interpolate to
    //! \param v0 Value0
    //! \param t0 Time0
    //! \param v1 Value1
    //! \param t1 Time1
    //! \return Interpolated vehicle dynamics
    //!
    virtual VectorDynamics Fuse_VehicleDynamics(const TIME &time, const VectorDynamics &v0, const TIME &t0, const VectorDynamics &v1, const TIME &t1) const = 0;


    //!
    //! \brief Abstract method to interpolate vehicle life
    //!
    //! This method is to be implemented by the instantiator of MaceData
    //! \param time Time to interpolate to
    //! \param v0 Value0
    //! \param t0 Time0
    //! \param v1 Value1
    //! \param t1 Time1
    //! \return Interpolated vehicle life
    //!
    virtual VehicleLife Fuse_VehicleLife(const TIME &time, const VehicleLife &v0, const TIME &t0, const VehicleLife &v1, const TIME &t1) const = 0;


    /////////////////////////////////////////////////////////
    /// VEHICLE DATA
    /////////////////////////////////////////////////////////

public:

    //!
    //! \brief GetAvailableVehicles Get a list of vehicle IDs this instance has knowledge of
    //! \param vehicleIDs Vector of vehicle IDs
    //!
    void GetAvailableVehicles(std::vector<unsigned int> &vehicleIDs) const
    {
        std::lock_guard<std::mutex> guard(m_AvailableVehicleMutex);
        vehicleIDs = m_AvailableVehicles;        
    }

    void GetVehicleFlightMode(const int &vehicleID, std::string &vehicleMode) const
    {
//        ModuleCharacteristic module = GetVehicleFromMAVLINKID(vehicleID);
//        std::unordered_map<std::string, TopicDatagram> topicMap = getAllLatestTopics(module);
//        for(auto it = topicMap.cbegin() ; it != topicMap.cend() ; ++it) {
//            MaceLog::Alert("Topic name: " + it->first);
////            std::vector<std::string> components = it->second.ListNonTerminals();
////            ModuleBase* base = (ModuleBase*)sender;
////            //base->NewTopic(it->first,targetID,components);
////    //        throw std::runtime_error("Requesting Data Sync Not Implemented");
////            MaceLog::Debug("Requesting Data Sync not implemented");
//        }

        std::lock_guard<std::mutex> guard(m_VehicleFlightModeMutex);
        if(m_VehicleFlightModeMap.find(vehicleID) != m_VehicleFlightModeMap.end()) {
            vehicleMode = m_VehicleFlightModeMap.at(vehicleID);
        }
        else {
            vehicleMode = "UNKNOWN"; // TODO: Actually grab this from an available enum?
        }
    }


    //!
    //! \brief GetLocalVehicles Get a list of local vehicle IDs this instance has knowledge of
    //! \param vehicleIDs Vector of vehicle IDs
    //!
    void GetLocalVehicles(std::vector<unsigned int> &vehicleIDs) const
    {
        std::lock_guard<std::mutex> guard(m_AvailableVehicleMutex);
        vehicleIDs = m_LocalVehicles;
    }


    std::vector<unsigned int> GetLocalVehicles() const
    {
        std::lock_guard<std::mutex> guard(m_AvailableVehicleMutex);
        return m_LocalVehicles;
    }

    bool HasMavlinkID(const unsigned int MAVLINKID) const
    {
        if(m_MAVLINKIDtoModule.find(MAVLINKID) == m_MAVLINKIDtoModule.cend())
        {
            return false;
        }
        return true;
    }

    //!
    //! \brief GetVehicleFromMAVLINKID
    //! \param MAVLINKID
    //! \return
    //!
    ModuleCharacteristic GetVehicleFromMAVLINKID(const unsigned int MAVLINKID) const
    {
        return m_MAVLINKIDtoModule.at(MAVLINKID);
    }


    bool getMavlinkIDFromModule(const ModuleCharacteristic &characterstic, uint8_t &vehicleID) const
    {
        for(auto it = m_MAVLINKIDtoModule.cbegin() ; it != m_MAVLINKIDtoModule.cend() ; ++it)
        {
            if(it->second == characterstic)
            {
                vehicleID = it->first;
                return true;
            }
        }

        MaceLog::Alert("Unknown module given to get key of -- Mace Instance: " + std::to_string(characterstic.MaceInstance) + " / Module ID: " + std::to_string(characterstic.ModuleID));
        return false;
    }

    //!
    //! \brief Determine if the given module has an associated ID on the MAVLINK network
    //! \param characterstic Module to check
    //! \return True if there is an ID on the MAVLINK network. Can be retreived with getMavlinkIDFromModule
    //!
    bool HasModuleAsMavlinkID(const ModuleCharacteristic &characterstic) const
    {
        for(auto it = m_MAVLINKIDtoModule.cbegin() ; it != m_MAVLINKIDtoModule.cend() ; ++it)
        {
            if(it->second == characterstic)
            {
                return true;
            }
        }

        return false;
    }

    //!
    //! \brief GetVehicleHomePostion Get the home position of the specified vehicle
    //! \param vehicleID Vehicle ID
    //! \return Vehicle home position
    //!
    command_item::SpatialHome GetVehicleHomePostion(const unsigned int &vehicleID) const
    {
        std::lock_guard<std::mutex> guard(m_VehicleHomeMutex);
        command_item::SpatialHome vehicleHome = m_VehicleHomeMap.at(vehicleID);
        return vehicleHome;
    }

    //!
    //! \brief GetGlobalOrigin Get the system wide global origin
    //! \return Global origin
    //!
    mace::pose::GeodeticPosition_3D GetGlobalOrigin() const
    {
        std::lock_guard<std::mutex> guard(m_VehicleHomeMutex);
        return m_GlobalOrigin;
    }

    //!
    //! \brief GetGridSpacing Get the grid spacing (used for RTA grid generation)
    //! \return Grid spacing
    //!
    double GetGridSpacing() const
    {
        std::lock_guard<std::mutex> guard(m_VehicleHomeMutex);
        double gridSpacing = m_GridSpacing;
        return gridSpacing;
    }

    //!
    //! \brief EnvironmentBoundarySet Determine if an Environment boundary has been set
    //! \return environment boundary flag
    //!
    bool EnvironmentBoundarySet() const
    {
        std::lock_guard<std::mutex> guard(m_EnvironmentalBoundaryMutex);
        bool boundarySet = flagBoundaryVerts;
        return boundarySet;
    }
    //!
    //! \brief GetEnvironmentBoundary Get the environment boundary
    //! \return Vector of positions that make up the boundary
    //!
    BoundaryItem::EnvironmentBoundary GetEnvironmentBoundary() const
    {
        std::lock_guard<std::mutex> guard(m_EnvironmentalBoundaryMutex);
        BoundaryItem::EnvironmentBoundary boundaryVerts = m_environmentBoundary;
        return boundaryVerts;
    }


    //!
    //! \brief GetVehicleBoundaryList Get vehicle boundary list
    //! \return Boundary list
    //!
    std::vector<BoundaryItem::BoundaryList> GetVehicleBoundaryList() const
    {
        std::lock_guard<std::mutex> guard(m_VehicleBoundaryMutex);
        std::vector<BoundaryItem::BoundaryList> boundaryList = m_vehicleBoundaryList;
        return boundaryList;
    }

private:


    //!
    //! \brief AddAvailableVehicle Add a new vehicle to the list of available vehicles
    //! \param vehicleID Vehicle ID to add
    //! \param internal If internal, the vehicle is attached to this MACE instance
    //!
    void AddAvailableVehicle(const int &vehicleID, bool internal, const ModuleCharacteristic &module)
    {
        std::lock_guard<std::mutex> guard(m_AvailableVehicleMutex);
        m_AvailableVehicles.push_back(vehicleID);
        std::sort( m_AvailableVehicles.begin(), m_AvailableVehicles.end());
        m_AvailableVehicles.erase( unique( m_AvailableVehicles.begin(), m_AvailableVehicles.end() ), m_AvailableVehicles.end() );

        m_MAVLINKIDtoModule.insert({vehicleID, module});

        if(internal == true)
        {
            m_LocalVehicles.push_back(vehicleID);
            m_LocalVehicles.erase( unique( m_LocalVehicles.begin(), m_LocalVehicles.end() ), m_LocalVehicles.end() );
        }
    }


    //!
    //! \brief UpdateVehicleHomePosition Update the vehicle home position
    //! \param vehicleHome New vehicle home
    //!
    void UpdateVehicleHomePosition(const uint8_t vehicleID, const command_item::SpatialHome &vehicleHome)
    {
        std::lock_guard<std::mutex> guard(m_VehicleHomeMutex);
        m_VehicleHomeMap[vehicleID] = vehicleHome;
    }

    //!
    //! \brief UpdateGlobalOrigin Update the global origin
    //! \param globalOrigin New global origin
    //!
    void UpdateGlobalOrigin(const mace::pose::GeodeticPosition_3D &globalOrigin)
    {
        std::lock_guard<std::mutex> guard(m_VehicleHomeMutex);
        double bearingTo = m_GlobalOrigin.polarBearingTo(&globalOrigin);
        double distanceTo = m_GlobalOrigin.distanceBetween2D(&globalOrigin); //in this case we need the 2D value since we are going to update only 2D position of boundaries
        m_GlobalOrigin = globalOrigin;
        updateBoundariesNewOrigin(distanceTo, bearingTo);
    }

    //!
    //! \brief UpdateGridSpacing Update the grid spacing
    //! \param gridSpacing New grid spacing value
    //!
    void UpdateGridSpacing(const double &gridSpacing)
    {
        std::lock_guard<std::mutex> guard(m_VehicleHomeMutex);
        m_GridSpacing = gridSpacing;
    }


    //!
    //! \brief UpdateEnvironmentBoundary Update environment boundary from the global parameters
    //! \param boundaryVerts New boundary vertices vector
    //! \param name New boundary name
    //! \param type New boundary type
    //!
    void UpdateEnvironmentBoundary(const std::vector<mace::pose::GeodeticPosition_2D> &boundaryVerts, const std::string &name, const std::string &type) {
        std::lock_guard<std::mutex> guard(m_EnvironmentalBoundaryMutex);
        m_environmentBoundary.setValues(boundaryVerts,type,name);
        flagBoundaryVerts = true;
    }



    //This is only called via RTA
    //!
    //! \brief UpdateVehicleCellMap Update the map of paritioned vehicle cells (from RTA module)
    //! \param vehicleMap Map of new vehicle cells
    //!
    void UpdateVehicleCellMap(const std::map<int, mace::geometry::Cell_2DC> &vehicleMap) {
        std::lock_guard<std::mutex> gaurd(m_VehicleBoundaryMutex);
        m_vehicleCellMap = vehicleMap;
    }

    //This is only called via RTA
    //!
    //! \brief UpdateVehicleBoundaryList Update the list vehicle of boundary lists
    //! \param boundaryList Vector of vehicle boundary lists
    //!
    void UpdateVehicleBoundaryList(const std::vector<BoundaryItem::BoundaryList> &boundaryList) {
        std::lock_guard<std::mutex> gaurd(m_VehicleBoundaryMutex);
        m_vehicleBoundaryList = boundaryList;
    }

    //!
    //! \brief RemoveVehicle Remove vehicle
    //! \param rn Resource/vehicle name
    //!
    void RemoveVehicle(const std::string &rn)
    {
        if(m_PositionDynamicsHistory.find(rn) == m_PositionDynamicsHistory.cend())
            throw std::runtime_error("resource name does not exists");

        m_PositionDynamicsHistory.erase(rn);
        m_AttitudeDynamicsHistory.erase(rn);
        m_VehicleLifeHistory.erase(rn);
    }

    //!
    //! \brief AddVehicleLife Add vehicle life data
    //! \param rn Resource name
    //! \param time Timestamp
    //! \param life Vehicle life
    //!
    void AddVehicleLife(const std::string &rn, const TIME &time, const VehicleLife &life)
    {
        std::lock_guard<std::mutex> guard(m_VehicleDataMutex);

        m_VehicleLifeHistory.at(rn).InsertObservation(time, life);
    }

    //!
    //! \brief set the dynamics for a specific vehicle.
    //!
    //! Commands are micro-level list of dynamics the vehicle is to physically achieve to reach a target.
    //! \param vehicleID ID of vehicle whose commands are being modified on
    //! \param locationalCommands List of commands to asign to vehicle
    //!
    void setVehicleDynamicsCommands(const std::string vehicleID, std::vector<FullVehicleDynamics> commands)
    {
        std::lock_guard<std::mutex> guard(m_VehicleDataMutex);

        m_VehicleCommandDynamicsList[vehicleID] = commands;
    }

public:


    //!
    //! \brief setTopicDatagram Set topic datagram
    //! \param topicName Topic name
    //! \param senderID Sender ID
    //! \param time Timestamp
    //! \param value Topic datagram value
    //!
    void setTopicDatagram(const std::string &topicName, const ModuleCharacteristic sender, const TIME &time, const TopicDatagram &value)
    {

        std::lock_guard<std::mutex> guard(m_TopicMutex);

        if(m_LatestTopic.find(topicName) == m_LatestTopic.cend()) {
            m_LatestTopic.insert({topicName, {}});
        }
        if(m_LatestTopic[topicName].find(sender) == m_LatestTopic[topicName].cend()) {
            m_LatestTopic[topicName].insert({sender, TopicDatagram()});
        }
        m_LatestTopic[topicName][sender].MergeDatagram(value);

        std::vector<std::string> terminalNames = value.ListTerminals();
        std::vector<std::string> nonTerminalNames = value.ListNonTerminals();
        for(size_t i = 0 ; i < terminalNames.size() ; i++) {
            m_LatestTopicComponentUpdateTime[topicName][sender][terminalNames.at(i)] = time;
        }
        for(size_t i = 0 ; i < nonTerminalNames.size() ; i++) {
            m_LatestTopicComponentUpdateTime[topicName][sender][nonTerminalNames.at(i)] = time;
        }
    }


    //!
    //! \brief getAllLatestTopics Get all latest topics from the specified target
    //! \param targetID Target ID
    //! \return Map of the latest topic datagrams
    //!
    std::unordered_map<std::string, TopicDatagram> getAllLatestTopics(const ModuleCharacteristic sender)
    {
        std::lock_guard<std::mutex> guard(m_TopicMutex);
        std::unordered_map<std::string, TopicDatagram> topicMap;
        for(auto it = m_LatestTopic.cbegin() ; it != m_LatestTopic.cend() ; ++it) {
            for(auto local_it = m_LatestTopic[it->first].cbegin(); local_it != m_LatestTopic[it->first].cend(); ++local_it)
            {
                if(local_it->first == sender)
                {
                    topicMap[it->first] = local_it->second;
                }
            }
        }
        return topicMap;

    }


    //!
    //! \brief GetCurrentTopicDatagram Get the current topic datagram of a specified name and sender ID
    //! \param topicName Topic name
    //! \param senderID Sender ID
    //! \return Current topic datagram
    //!
    TopicDatagram GetCurrentTopicDatagram(const std::string &topicName, const ModuleCharacteristic sender) const
    {
        std::lock_guard<std::mutex> guard(m_TopicMutex);
        return m_LatestTopic.at(topicName).at(sender);
    }


public:

    //!
    //! \brief GetPositionDynamics Get position and velocity of a specified resource at a specified time
    //! \param rn Resource name
    //! \param time Time to query
    //! \param pos Position container
    //! \param velocity Velocity container
    //! \return True if successful
    //!
    bool GetPositionDynamics(const std::string rn, const TIME &time, Eigen::Vector3d &pos, Eigen::Vector3d &velocity) const
    {
        UNUSED(pos);
        UNUSED(velocity);
        std::lock_guard<std::mutex> guard(m_VehicleDataMutex);

        VectorDynamics vec;
        bool success = GetObservation<VectorDynamics>(m_PositionDynamicsHistory.at(rn), time, vec, [&](const TIME& t, const VectorDynamics& d0, const TIME& t0, const VectorDynamics& d1, const TIME& t1){ return Fuse_VehicleDynamics(t, d0, t0, d1, t1);});

        if(success == false)
            return false;
        return true;
    }

    //!
    //! \brief GetAttitudeDynamics Get attitude and attitude rates of a specified resource at a specified time
    //! \param rn Resource name
    //! \param time Time to query
    //! \param att Attiude container
    //! \param att_rates Attitude rates conatiner
    //! \return True if successful
    //!
    bool GetAttitudeDynamics(const std::string rn, const TIME &time, Eigen::Vector3d &att, Eigen::Vector3d &att_rates) const
    {
        UNUSED(att);
        UNUSED(att_rates);
        std::lock_guard<std::mutex> guard(m_VehicleDataMutex);

        VectorDynamics vec;
        bool success = GetObservation<VectorDynamics>(m_AttitudeDynamicsHistory.at(rn), time, vec, [&](const TIME& t, const VectorDynamics& d0, const TIME& t0, const VectorDynamics& d1, const TIME& t1){ return Fuse_VehicleDynamics(t, d0, t0, d1, t1);});

        if(success == false)
            return false;
        return true;
    }


    //!
    //! \brief get the command dynamics for a specific vehicle.
    //!
    //! Commands are micro-level list of positions/attitude the vehicle is to physically achieve to reach a target.
    //! Will return empty array if no attitudes are commanded for vehicle
    //! \param vehicleID ID of Vehicle
    //! \return list of commanded dynamics
    //!
    std::vector<FullVehicleDynamics> getVehicleDynamicsCommands(const std::string vehicleID) const
    {
        std::lock_guard<std::mutex> guard(m_VehicleDataMutex);

        if(m_VehicleCommandDynamicsList.find(vehicleID) == m_VehicleCommandDynamicsList.cend())
            return {};

        return m_VehicleCommandDynamicsList.at(vehicleID);
    }


    /////////////////////////////////////////////////////////
    /// PATH PLANNING DATA
    /////////////////////////////////////////////////////////

private:


    //!
    //! \brief Entirely replaces the stored Resource map with given matrix
    //! \param occupancy map to replace with
    //!
    void ResourceMap_ReplaceMatrix(const Eigen::MatrixXd &newResourceMap)
    {
        std::lock_guard<std::mutex> guard(m_Mutex_ResourceMap);

        m_ResourceMap = newResourceMap;
    }


    //!
    //! \brief Replace a discrete set of cells in the resource map
    //!
    //! Thread Safe
    //! May be faster than ResourceMap_ReplaceMatrix if operations are sparse
    //! \param cells Vector of cells to replace in the resourse map
    //!
    void ResourceMap_ReplaceCells(const std::vector<MatrixCellData<double>> &cells)
    {
        std::lock_guard<std::mutex> guard(m_Mutex_ResourceMap);

        ReplaceCellsInMatrix(m_ResourceMap, cells);
    }


    //!
    //! \brief Call lambda function to modify components of Resource map
    //!
    //! Thread Safe
    //! May be faster than ResourceMap_ReplaceMatrix if operations are sparse
    //! \param func Lambda function to modify map
    //!
    void ResourceMap_GenericOperation(const std::function<void(Eigen::MatrixXd &)> &func)
    {
        std::lock_guard<std::mutex> guard(m_Mutex_ResourceMap);

        func(m_ResourceMap);
    }





    //!
    //! \brief Entirely replaces the stored occupancy map with given matrix
    //! \param occupancy map to replace with
    //!
    void OccupancyMap_ReplaceMatrix(const Eigen::MatrixXd &newOccupancyMap)
    {
        UNUSED(newOccupancyMap);
        //m_OccupancyMap = newOccupancyMap;
    }


    //!
    //! \brief Replace a discrete set of cells in the occupancy map
    //!
    //! Thread Safe
    //! May be faster than OccupancyMap_ReplaceMatrix if operations are sparse
    //! \param cells Vector of cells to replace in the resourse map
    //!
    void OccupanyMap_ReplaceCells(const std::vector<MatrixCellData<double>> &cells)
    {
        UNUSED(cells);
        //ReplaceCellsInMatrix(m_OccupancyMap, cells);
    }


    //!
    //! \brief Call lambda function to modify components of Occupancy map
    //!
    //! Thread Safe
    //! May be faster than OccupancyMap_ReplaceMatrix if operations are sparse
    //! \param func Lambda function to modify map
    //!
    void OccupancyMap_GenericOperation(const std::function<void(Eigen::MatrixXd &)> &func)
    {
        UNUSED(func);
        //func(m_OccupancyMap);
    }

    //!
    //! \brief Entirely replaces the stored Probility map with given matrix
    //! \param occupancy map to replace with
    //!
    void ProbabilityMap_ReplaceMatrix(const Eigen::MatrixXd &newProbabilityMap)
    {
        std::lock_guard<std::mutex> guard(m_Mutex_ProbabilityMap);

        m_ProbabilityMap = newProbabilityMap;
    }


    //!
    //! \brief Replace a discrete set of cells in the probibility map
    //!
    //! Thread Safe
    //! May be faster than ProbabilityMap_ReplaceMatrix if operations are sparse
    //! \param cells Vector of cells to replace in the resourse map
    //!
    void ProbabilityMap_ReplaceCells(const std::vector<MatrixCellData<double>> &cells)
    {
        std::lock_guard<std::mutex> guard(m_Mutex_ProbabilityMap);

        ReplaceCellsInMatrix(m_ProbabilityMap, cells);
    }


    //!
    //! \brief Call lambda function to modify components of Probility map
    //!
    //! Thread Safe
    //! May be faster than ProbabilityMap_ReplaceMatrix if operations are sparse
    //! \param func Lambda function to modify map
    //!
    void ProbabilityMap_GenericOperation(const std::function<void(Eigen::MatrixXd &)> &func)
    {
        std::lock_guard<std::mutex> guard(m_Mutex_ProbabilityMap);

        func(m_ProbabilityMap);
    }

public:

    //!
    //! \brief Retreive a copy of the Resource map
    //!
    //! Thread safe
    //! \return Copy of Resource map
    //!
    Eigen::MatrixXd ResourceMap_GetCopy() const
    {
        std::lock_guard<std::mutex> guard(m_Mutex_ResourceMap);

        return m_ResourceMap;
    }


    //!
    //! \brief Read specific cells from resourse map
    //! \param cells Vector of cells to read
    //!
    void ResourceMap_ReadCells(std::vector<MatrixCellData<double>> &cells)
    {
        std::lock_guard<std::mutex> guard(m_Mutex_ResourceMap);

        ReadCellsInMatrix(m_ResourceMap, cells);
    }


    //!
    //! \brief Call lambda to perform a generic const operation on Resource map
    //!
    //! Thread Safe
    //! May be faster than GetResourceMapCopy if matrix is large.
    //! \param func Lambda function to read from map
    //!
    void ResourceMap_GenericConstOperation(std::function<void(const Eigen::MatrixXd &)> &func) const
    {
        std::lock_guard<std::mutex> guard(m_Mutex_ResourceMap);

        func(m_ResourceMap);
    }

    //!
    //! \brief insertGlobalObservation Insert a global point cloud into the current octomap
    //! \param obj Point cloud in the global/inertial frame
    //! \param position Position of the sensor taking the measurement
    //!
    void insertGlobalObservation(octomap::Pointcloud& obj, const pose::CartesianPosition_3D &position)
    {
        std::lock_guard<std::mutex> guard(m_Mutex_OccupancyMaps);
        m_OctomapWrapper->updateFromPointCloud(&obj, position);
    }

    //!
    //! \brief insertObservation Insert a point cloud into the current octomap
    //! \param obj Point cloud in the vehicle frame
    //! \param position Position of the sensor taking the measurement
    //! \param orientation Orientation of the sensor taking the measurement
    //!
    void insertObservation(octomap::Pointcloud& obj, const pose::CartesianPosition_3D &position, const mace::pose::Rotation_3D &orientation)
    {
        std::lock_guard<std::mutex> guard(m_Mutex_OccupancyMaps);
        m_OctomapWrapper->updateFromPointCloud(&obj, position, orientation);
        //m_OctomapWrapper->updateFromPointCloud(&obj);
        // TODO: Test insert and origin point. We'll have to ensure a (0,0,0) origin works, or we'll have to calculate the sensor origin as it moves
        //          - One option is instead of transforming to the world frame, we can pass in a sensor origin AND frame origin point, and the
        //              overloaded version of insertPointCloud() will transform for us. I'm partial to all sensor data being reported in the world
        //              frame in this case though

        //m_OccupancyMap.insertPointCloud(obj, octomap::point3d(0,0,0));
        //        m_OccupancyMap.insertPointCloudRays(obj, octomap::point3d(0,0,0)); // Less efficient than insertPointCloud() accordoing to docs
    }


    //!
    //! \brief checkForOccupancy rather than copying the entire occupancy map
    //! to the module, the check can be done right from the core
    //!
    bool checkForOccupancy()
    {
        return false;
    }

    //!
    //! \brief Read specific cells from occupancy map
    //! \param cells Vector of cells to read
    //!
    void OccupancyMap_ReadCells(std::vector<MatrixCellData<double>> &cells)
    {
        UNUSED(cells);
        //ReadCellsInMatrix(m_OccupancyMap, cells);
    }


    //!
    //! \brief Call lambda to perform a generic const operation on occupancy map
    //!
    //! Thread Safe
    //! May be faster than GetOccupancyMapCopy if matrix is large.
    //! \param func Lambda function to read from map
    //!
    void OccupancyMap_GenericConstOperation(std::function<void(const Eigen::MatrixXd &)> &func) const
    {
        UNUSED(func);
        //func(m_OccupancyMap);
    }


    //!
    //! \brief Retreive a copy of the Probibility map
    //!
    //! Thread safe
    //! \return Copy of Probibility map
    //!
    Eigen::MatrixXd ProbibilityMap_GetCopy() const
    {
        std::lock_guard<std::mutex> guard(m_Mutex_ProbabilityMap);

        return m_ProbabilityMap;
    }

    //!
    //! \brief Read specific cells from Probibility map
    //! \param cells Vector of cells to read
    //!
    void ProbibilityMap_ReadCells(std::vector<MatrixCellData<double>> &cells)
    {
        std::lock_guard<std::mutex> guard(m_Mutex_ProbabilityMap);

        ReadCellsInMatrix(m_ProbabilityMap, cells);
    }

    //!
    //! \brief Call lambda to perform a generic const operation on Probibility map
    //!
    //! Thread Safe
    //! May be faster than GetProbibilityMapCopy if matrix is large.
    //! \param func Lambda function to read from map
    //!
    void ProbibilityMap_GenericConstOperation(std::function<void(const Eigen::MatrixXd &)> &func) const
    {
        std::lock_guard<std::mutex> guard(m_Mutex_ProbabilityMap);

        func(m_ProbabilityMap);
    }



private:


    template <typename T>
    //!
    //! \brief GetObservation
    //! \param history
    //! \param time
    //! \param data
    //! \param reckoning
    //! \return
    //!
    bool GetObservation(const ObservationHistory<TIME, T> &history, const TIME time, T &data, const std::function<T(const TIME&, const T&, const TIME&, const T&, const TIME&)> &reckoning) const
    {
        std::vector<TIME> prevTimes;
        std::vector<T> prevData;
        history.GetClosestObservation(time, BACKWARD_EXCLUSION, 1, prevData, prevTimes);

        std::vector<TIME> nextTimes;
        std::vector<T> nextData;
        history.GetClosestObservation(time, FORWARD_INCLUSION, 1, nextData, nextTimes);

        //if neither vector has data then we cant get anything so return false.
        if(prevTimes.size() == 0 && nextTimes.size() == 0)
            return false;

        //if previous is empty then return what is in next
        if(prevTimes.size() == 0)
        {
            data = nextData.at(0);
            return true;
        }

        //if next is empty then return what is in prev
        if(nextTimes.size() == 0)
        {
            data = prevData.at(0);
            return true;
        }

        //neither is empty so call fusion method and return
        data = reckoning(time, prevData.at(0), prevTimes.at(0), nextData.at(0), nextTimes.at(0));
        return true;
    }

    //std::map<int, std::shared_ptr<VehicleObject>> m_VehicleData;

    std::unordered_map<std::string, std::unordered_map<ModuleCharacteristic, TopicDatagram>> m_LatestTopic;
    std::unordered_map<std::string, std::unordered_map<ModuleCharacteristic, std::unordered_map<std::string, TIME>>> m_LatestTopicComponentUpdateTime;

    mutable std::mutex m_AvailableVehicleMutex;
    std::vector<unsigned int> m_AvailableVehicles;
    std::vector<unsigned int> m_LocalVehicles;
    std::unordered_map<unsigned int, ModuleCharacteristic> m_MAVLINKIDtoModule;

    mutable std::mutex m_VehicleFlightModeMutex;
    std::unordered_map<unsigned int, std::string> m_VehicleFlightModeMap;

    mutable std::mutex m_VehicleHomeMutex;
    std::map<unsigned int, command_item::SpatialHome> m_VehicleHomeMap;
    mace::pose::GeodeticPosition_3D m_GlobalOrigin;
    double m_GridSpacing = -1;

    mutable std::mutex m_VehicleBoundaryMutex;
    std::map<int, mace::geometry::Cell_2DC> m_vehicleCellMap;
    std::vector<BoundaryItem::BoundaryList> m_vehicleBoundaryList;
    BoundaryItem::EnvironmentBoundary m_environmentBoundary;
    bool flagBoundaryVerts;


    std::map<std::string, ObservationHistory<TIME, VectorDynamics> > m_PositionDynamicsHistory;
    std::map<std::string, ObservationHistory<TIME, VectorDynamics> > m_AttitudeDynamicsHistory;
    std::map<std::string, ObservationHistory<TIME, VehicleLife> > m_VehicleLifeHistory;
    std::map<std::string, std::vector<FullVehicleDynamics> > m_VehicleCommandDynamicsList;


    Eigen::MatrixXd m_ResourceMap;
    Eigen::MatrixXd m_ProbabilityMap;


    mutable std::mutex m_VehicleDataMutex;

    mutable std::mutex m_Mutex_ResourceMap;
    mutable std::mutex m_Mutex_ProbabilityMap;
    mutable std::mutex m_TopicMutex;

    /////////////////////////////////////////////////////////
    /// PATH PLANNING DATA
    /////////////////////////////////////////////////////////

private:
    mutable std::mutex m_Mutex_LayeredMap;
    std::shared_ptr<mace::maps::LayeredMap> m_LayeredMap_Local;

public:

    //!
    //! \brief getLayeredMap Get a const pointer to the LayeredMap object in the core
    //! \return const Pointer to a LayeredMap object
    //!
    const std::shared_ptr<mace::maps::LayeredMap> getLayeredMap() const {
        return m_LayeredMap_Local;
    }

    // Add layer to layered map -- grabs grid size and adds default layer of specific type? (default fill value)
    template <class LayerClass>
    void addLayeredMapLayer(const std::string &layerName) const {
        std::lock_guard<std::mutex> guard(m_Mutex_LayeredMap);
        m_LayeredMap_Local->addMapLayer(layerName);
    }

    //!
    //! \brief updateLayeredMapLayer If a map layer exists, update with provided pointer. If not, insert the new pointer
    //! \param layerName Layer to update
    //! \param mapLayer Pointer to map layer
    //!
    void updateLayeredMapLayer(const std::string &layerName, mace::maps::BaseGridMap* mapLayer) const {
        std::lock_guard<std::mutex> guard(m_Mutex_LayeredMap);
        m_LayeredMap_Local->updateMapLayer(layerName, mapLayer);

        MaceLog::Alert("Update layered map layer");

        // TODO: Call lambda functions? (lambdas would be set with some setter method)
//        for(auto lambda : m_LayeredMapLambdas) {
//            lambda(layerName);
//        }
    }

    //!
    //! \brief updateLayeredMapGridSize Update the underlying map layers size to the given dimensions. NOTE: The values given are assumed to be in a standard cartesian coordinate frame (i.e. not rotated)
    //! \param minX Minimum x value
    //! \param maxX Maximum x value
    //! \param minY Minimum y value
    //! \param maxY Maximum y value
    //! \param xRes X resolution
    //! \param yRes Y Resolution
    //!
    void updateLayeredMapGridSize(const double &minX, const double &maxX, const double &minY, const double &maxY, const double &x_res, const double &y_res) const {
        std::lock_guard<std::mutex> guard(m_Mutex_LayeredMap);
        m_LayeredMap_Local->updateGridSize(minX, maxX, minY, maxY, x_res, y_res);
    }

    /////////////////////////////////////////////////////////
    /// PATH PLANNING DATA
    /////////////////////////////////////////////////////////

private:
    mutable std::mutex m_Mutex_OccupancyMaps;
    mace::maps::OctomapWrapper* m_OctomapWrapper;

public:

    //!
    //! \brief getOctomapDimensions Get the current octomap dimensions
    //! \param minX Minimum x value of the bounding box
    //! \param maxX Maximum x value of the bounding box
    //! \param minY Minimum y value of the bounding box
    //! \param maxY Maximum y value of the bounding box
    //! \param minZ Minimum z value of the bounding box
    //! \param maxZ Maximum z value of the bounding box
    //!
    void getOctomapDimensions(double &minX, double &maxX, double &minY, double &maxY, double &minZ, double &maxZ) const;

    //!
    //! \brief updateOctomapProperties Update the octomap properties
    //! \param properties New octomap properties
    //! \return True if successful
    //!
    bool updateOctomapProperties(const mace::maps::OctomapSensorDefinition &properties);

    //!
    //! \brief updateMappingProjectionProperties Update octomap projection properties
    //! \param properties New octomap projection properties
    //! \return True if successful
    //!
    bool updateMappingProjectionProperties(const mace::maps::Octomap2DProjectionDefinition &properties);

    //!
    //! \brief loadOccupancyEnvironment Load an occupancy map from a file
    //! \param filePath File path
    //! \return True if successful
    //!
    bool loadOccupancyEnvironment(const std::string &filePath);

    //!
    //! \brief getOccupancyGrid3D Get the current 3D occupancy map
    //! \return True if successful
    //!
    octomap::OcTree getOccupancyGrid3D() const;

    //!
    //! \brief getCompressedOccupancyGrid2D Get the current compressed 2D occupancy map
    //! \return True if successful
    //!
    mace::maps::Data2DGrid<mace::maps::OccupiedResult> getCompressedOccupancyGrid2D() const;


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// VEHICLE MISSION METHODS: The following methods are in support of accessing the mission items stored within MaceData.
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /*
    The following methods aid a MACE instance in assigning an appropriate missionID to the mission in the core data.
    The data class is responsible for reporting an updated missionKey to the calling agent attempting to update
    the core data structure.
    */

    //variables
private:
    mutable std::mutex MUTEXMissionID;
    //!
    //! \brief mapMissionID
    //! The map structure is broken down as systemID(the system for which the mission was created for or the mission
    //! is referencing and would be applied to), creatorID(the system for which created the actual mission), and lastly
    //! the missionID as the unique identifier associated with the actual mission.
    //!
    std::map<int,std::map<int,int>> mapMissionID;

    //methods
public:

    //!
    //! \brief appendAssociatedMissionMap Append a mission list
    //! \param missionList Mission list
    //! \return Key of new mission list
    //!
    MissionItem::MissionKey appendAssociatedMissionMap(const MissionItem::MissionList &missionList);

    //!
    //! \brief appendAssociatedMissionMap Append a mission list with a new system ID
    //! \param newSystemID New system ID
    //! \param missionList Mission list
    //! \return Key of new mission list
    //!
    MissionItem::MissionKey appendAssociatedMissionMap(const int &newSystemID, const MissionItem::MissionList &missionList);

private:
    //!
    //! \brief getAvailableMissionID Get mission ID based on mission key
    //! \param key Key to query
    //! \return Mission ID
    //!
    int getAvailableMissionID(const MissionItem::MissionKey &key);

    /*
    The following aids in handling mission reception to/from the core.
    */
    //variables
private:
    mutable std::mutex MUTEXMissions;
    std::map<MissionItem::MissionKey,MissionItem::MissionList> mapMissions;
    std::map<int,MissionItem::MissionKey> mapCurrentMission;

    //methods
public:
    /*
    The following methods aid in handling the reception of a new mission over the external link. The items handled
    in here will be partial lists and should not migrate into the main mission queue.
    */
    //!
    //! \brief updateCurrentMissionItem Update the current mission item
    //! \param current Current mission item
    //! \return
    //!
    bool updateCurrentMissionItem(const MissionItem::MissionItemCurrent &current);

    /*
    The following methods aid getting the mission list from the mace data class. The following methods aid getting
    the current mission object and keys.
    */
    //!
    //! \brief getMissionList Get mission list corresponding to the system ID, mission type, mission state
    //! \param systemID System ID
    //! \param type Mission type
    //! \param state Mission state
    //! \param missionList Container for the mission list
    //! \return True if mission list exists
    //!
    bool getMissionList(const int &systemID, const MissionItem::MISSIONTYPE &type, const MissionItem::MISSIONSTATE &state, MissionItem::MissionList &missionList) const;

    //!
    //! \brief getMissionList Get mission list based on mission key
    //! \param missionKey Mission key to query
    //! \param missionList Container for the mission list
    //! \return True if mission list exists
    //!
    bool getMissionList(const MissionItem::MissionKey &missionKey, MissionItem::MissionList &missionList) const;

    //!
    //! \brief getCurrentMissionKey Get current mission key for the specified system ID
    //! \param systemID System ID to query
    //! \param key Container for the mission key
    //! \return True if mission key exists
    //!
    bool getCurrentMissionKey(const int &systemID, MissionItem::MissionKey &key) const;

    //!
    //! \brief getCurrentMission Get current mission for the specified system ID
    //! \param systemID System ID to query
    //! \param cpyMission Container for the mission list
    //! \return  True if mission exists
    //!
    bool getCurrentMission(const int &systemID, MissionItem::MissionList &cpyMission) const;

    //!
    //! \brief getCurrentMissionValidity Get current mission validity for the specified system ID
    //! \param systemID System ID to query
    //! \return True if current mission exists
    //!
    bool getCurrentMissionValidity(const int &systemID) const;

    //!
    //! \brief getMissionKeyValidity Get current mission key validity
    //! \param key Key to query
    //! \return True if mission key exists
    //!
    bool getMissionKeyValidity(const MissionItem::MissionKey &key) const;


    /*
    The following methods aid getting the mission list from the mace data class. The following methods aid getting
    the current mission object and keys.
    */


    //!
    //! \brief getOnboardMissionKeys Get a list of mission keys for the specified system ID
    //! \param systemID System ID to query
    //! \return Vector of mission keys
    //!
    std::vector<MissionItem::MissionKey> getMissionKeysForVehicle(const int &systemID) const;


    //!
    //! \brief removeFromMissionMap Remove a mission with the corresponding key from the map
    //! \param missionKey Mission key to remove
    //!
    void removeFromMissionMap(const MissionItem::MissionKey &missionKey);

    //!
    //! \brief receivedMissionACKKey Handle a received mission ACK key
    //! \param key Key of the mission
    //! \param newState New misison state
    //! \return Key for the current mission in its current state
    //!
    MissionItem::MissionKey receivedMissionACKKey(const MissionItem::MissionKey &key, const MissionItem::MISSIONSTATE &newState);

    //!
    //! \brief receivedNewMission Received the full mission
    //! \param missionList Received mission list
    //!
    void receivedNewMission(const MissionItem::MissionList &missionList);

    /*
    The following methods update the mission type state of the appropriate mission items.
    */
    //!
    //! \brief updateMissionExeState Update the execution state of the mission with the corresponding key
    //! \param missionKey Mission key to update
    //! \param state New execution state
    //!
    void updateMissionExeState(const MissionItem::MissionKey &missionKey, const Data::MissionExecutionState &state);

    //!
    //! \brief updateOnboardMission Update the onboard mission with the new mission key
    //! \param missionKey New mission key
    //! \return True if update successful
    //!
    bool updateOnboardMission(const MissionItem::MissionKey &missionKey);

    //!
    //! \brief checkForCurrentMission Check for the current mission corresponding to the key
    //! \param missionKey Mission key to query
    //! \return True if current mission exists
    //!
    bool checkForCurrentMission(const MissionItem::MissionKey &missionKey);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// MACE BOUNDARY METHODS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /**
      * Methods for working with the boundaries associated with MACE.
      * Boundaries defined in these functions are generic to include
      * operational and resource boundaries. The only accepted data type
      * at the current time is a cartesian boundary. This simplifies
      * math operations when global origin and other updates are required.
      */

public:

    BoundaryIdentifierType setBoundaryByKey(const BoundaryItem::BoundaryCharacterisic &key, const BoundaryItem::BoundaryList &boundary);

    // TODO-PAT: Move to appropriate spot:
    void updateVehicleFlightMode(const int &vehicleID, const string vehicleMode);


    //!
    //! \brief Fetch all the boundaries of the given typ efor the given vehicle
    //! \param vehicleID ID of vehicle to get boundaries of
    //! \param type Type of boundaries to get
    //! \return List of boundaries, empty if no boundary exists for given criteria
    //!
    std::vector<uint8_t> getBoundaryForVehicle(const int &vehicleID, const BoundaryItem::BOUNDARYTYPE &type);


    //!
    //! \brief getBoundaryFromIdentifier Return the boundary corresponding to a specific ID
    //! \param ID Boundary identifier
    //! \param boundary Container for the requested boundary
    //! \return True if boundary is found, False otherwise
    //!
    bool getBoundaryFromIdentifier(const BoundaryIdentifierType &ID, BoundaryItem::BoundaryList &boundary) const;


    //!
    //! \brief getCharactersticFromIdentifier Get boundary characteristic given a specific boundary identifier
    //! \param ID Boundary identifier
    //! \param characteristic Container for boundary characteristic
    //! \return True if characteristic is found, False otherwise
    //!
    bool getCharactersticFromIdentifier(const BoundaryIdentifierType ID, BoundaryItem::BoundaryCharacterisic &characteristic) const;


private:
    //!
    //! \brief updateBoundariesNewOrigin Update the origin of the boundary points from a new distance/bearing relative to a new origin point
    //! \param distance Distance to new origin point
    //! \param bearing Bearing to new origin point
    //!
    void updateBoundariesNewOrigin(const double &distance, const double &bearing);


private:
    mutable std::mutex m_EnvironmentalBoundaryMutex;
    std::unordered_map<uint8_t, std::tuple<BoundaryItem::BoundaryCharacterisic, BoundaryItem::BoundaryList>> m_Boundaries;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// SYSTEM TIME METHODS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

public:

    //!
    //! \brief updateCurrentSystemTimeDelta Given the microseconds since epoch (from MAVLINK SYSTEM_TIME message), set the delta between epoch and SYSTEMCLOCK (in milliseconds)
    //! \param microsecondsSinceEpoch Microseconds since epoch (from MALINK SYSTEM_TIME message)
    //!
    void updateCurrentSystemTimeDelta(const uint64_t &microsecondsSinceEpoch);

    //!
    //! \brief setCurrentDeltaTime If the current offset time is known, set the current deltaT in milliseconds
    //! \param millseconds Value to set the member variable storing offset time to
    //!
    void setCurrentDeltaTime(const double &millseconds);

    //!
    //! \brief MaceData::getMAVLINKAdjustedTime Get the current time adjusted with the delta time
    //! \return EnvironmentTime container with time since epoch, adjusted with delta
    //!
    Data::EnvironmentTime getCurrentSystemTime_Adjusted() const;

    //!
    //! \brief getDeltaT_msec Get delta time between MAVLINK SYSTEM_TIME since epoch and SYSTEMCLOCK
    //! \return Delta in milliseconds
    //!
    double getDeltaT_msec() const
    {
        return deltaT_msec;
    }

private:
    double deltaT_msec = 0.0;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// LOCAL/REMOTE MODULE DEFINITION METHODS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

public:


    void AddLocalModule(const ModuleCharacteristic &characterstic, const ModuleClasses &type)
    {
        m_LocalModules.insert({characterstic, type});
    }


    bool AddRemoteModule(const ModuleCharacteristic &characterstic, const ModuleClasses &type)
    {
        if(m_RemoteModules.find(characterstic) == m_RemoteModules.cend())
        {
            m_RemoteModules.insert({characterstic, type});
            return true;
        }
        return false;
    }


    std::vector<ModuleCharacteristic> GetAllLocalModules() const
    {
        std::vector<ModuleCharacteristic> vec;
        for(auto it = m_LocalModules.cbegin() ; it != m_LocalModules.cend() ; ++it)
        {
            vec.push_back(it->first);
        }

        return vec;
    }


    std::vector<ModuleCharacteristic> GetAllRemoteModules() const
    {
        std::vector<ModuleCharacteristic> vec;
        for(auto it = m_RemoteModules.cbegin() ; it != m_RemoteModules.cend() ; ++it)
        {
            vec.push_back(it->first);
        }
        return vec;
    }


    std::vector<ModuleCharacteristic> GetAllModules() const
    {
        std::vector<ModuleCharacteristic> vec1 = GetAllLocalModules();
        std::vector<ModuleCharacteristic> vec2 = GetAllRemoteModules();

        vec1.insert(vec1.end(), vec2.begin(), vec2.end());

        return vec1;
    }


    ModuleClasses getModuleType(const ModuleCharacteristic &characterstic) const
    {
        if(m_LocalModules.find(characterstic) != m_LocalModules.cend())
        {
            return m_LocalModules.at(characterstic);
        }

        if(m_RemoteModules.find(characterstic) != m_RemoteModules.cend())
        {
            return m_RemoteModules.at(characterstic);
        }

        throw std::runtime_error("Unknown module given");
    }


    //!
    //! \brief Determine if the given module exists on the test environment
    //! \param characterstic Characterstic to check
    //! \return True if exists
    //!
    bool HasModule(const ModuleCharacteristic &characterstic) const
    {
        if(m_LocalModules.find(characterstic) != m_LocalModules.cend())
        {
            return true;
        }

        if(m_RemoteModules.find(characterstic) != m_RemoteModules.cend())
        {
            return true;
        }

        return false;
    }


    bool KnownMaceInstance(uint8_t MaceInstanceID) const
    {
        for(auto it = m_LocalModules.cbegin() ; it != m_LocalModules.cend() ; ++it)
        {
            if(it->first.MaceInstance == MaceInstanceID)
            {
                return true;
            }
        }

        for(auto it = m_RemoteModules.cbegin() ; it != m_RemoteModules.cend() ; ++it)
        {
            if(it->first.MaceInstance == MaceInstanceID)
            {
                return true;
            }
        }

        return false;
    }

private:

    std::unordered_map<ModuleCharacteristic, ModuleClasses> m_LocalModules;
    std::unordered_map<ModuleCharacteristic, ModuleClasses> m_RemoteModules;

};

} //END MaceCore Namespace

#endif // I_MACE_DATA_H
