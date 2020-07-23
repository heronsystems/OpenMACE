#ifndef MODULE_PATH_PLANNING_NASAPHASE2_H
#define MODULE_PATH_PLANNING_NASAPHASE2_H

#include "module_path_planning_nasaphase2_global.h"

#include "common/common.h"
#include "mace_core/i_module_command_path_planning.h"

#include "data_generic_mission_item_topic/mission_item_topic_components.h"

#include "data/topic_data_object_collection.h"
#include "base_topic/base_topic_components.h"

#include "base/state_space/cartesian_2D_space.h"

#include "planners/rrt_base.h"
#include "planners/nearest_neighbor_flann.h"

#include "base/state_space/discrete_motion_validity_check.h"
#include "base/state_space/special_validity_check.h"


#include "maps/octomap_wrapper.h"
#include "maps/map_topic_components.h"

using namespace octomap;

#include "base/pose/cartesian_position_2D.h"
#include "base/geometry/cell_2DC.h"

#include "maps/data_2d_grid.h"
#include "maps/occupancy_definition.h"

#include "base/geometry/base_line.h"
#include "base/pose/pose_components.h"
#include "base_topic/base_topic_components.h"

#include "planners/virtual_potential_fields/potential_fields.h"
#include "planners/virtual_potential_fields/virtual_force.h"

#include "data_generic_command_item/command_item_components.h"

using namespace mace ;
using namespace geometry;
using namespace maps;

class MODULE_PATH_PLANNING_NASAPHASE2SHARED_EXPORT ModulePathPlanningNASAPhase2 : public MaceCore::IModuleCommandPathPlanning, public mace::planners::Planner_Interface
{

public:
    ModulePathPlanningNASAPhase2();

public:

    //!
    //! \brief This module as been attached as a module
    //! \param ptr pointer to object that attached this instance to itself
    //!
    void AttachedAsModule(MaceCore::IModuleTopicEvents* ptr) override;


    //!
    //! \brief Describes the strucure of the parameters for this module
    //! \return Strucure
    //!
    std::shared_ptr<MaceCore::ModuleParameterStructure> ModuleConfigurationStructure() const override;


    //!
    //! \brief Provides object contains parameters values to configure module with
    //! \param params Parameters to configure
    //!
    void ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params) override;

    void OnModulesStarted() override;
    //!
    //! \brief New non-spooled topic given
    //!
    //! NonSpooled topics send their data immediatly.
    //! \param topicName Name of stopic
    //! \param sender Module that sent topic
    //! \param data Data for topic
    //! \param target Target module (or broadcasted)
    //!
    void NewTopicData(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const MaceCore::TopicDatagram &data,
                      const OptionalParameter<MaceCore::ModuleCharacteristic> &target) override;


    //!
    //! \brief New Spooled topic given
    //!
    //! Spooled topics are stored on the core's datafusion.
    //! This method is used to notify other modules that there exists new data for the given components on the given module.
    //! \param topicName Name of topic given
    //! \param sender Module that sent topic
    //! \param componentsUpdated Components in topic that where updated
    //! \param target Target moudle (or broadcast)
    //!
    void NewTopicSpooled(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const std::vector<std::string> &componentsUpdated,
                         const OptionalParameter<MaceCore::ModuleCharacteristic> &target = OptionalParameter<MaceCore::ModuleCharacteristic>()) override;


    //! Virtual functions as defined by IModuleCommandPathPlanning
public:

    // ============================================================================= //
    // ======== Virtual functions as defined by IModuleCommandGenericBoundaries ==== //
    // ============================================================================= //

    //!
    //! \brief NewlyAvailableBoundary Subscriber to a new boundary
    //! \param key Key corresponding to the updated boundary in the core
    //!
    void NewlyAvailableBoundary(const uint8_t &key, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender = OptionalParameter<MaceCore::ModuleCharacteristic>()) override;

    void NewlyAvailableVehicle(const int &vehicleID, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender) override;

    void NewlyLoadedOccupancyMap() override;

    void NewlyUpdatedOccupancyMap() override;

    void NewlyUpdatedGlobalOrigin(const mace::pose::GeodeticPosition_3D &position) override;

    void NewlyAvailableMission(const MissionItem::MissionList &mission) override;

    void NewlyAvailableGoalState(const mace::state_space::GoalState &goal) override;

public:
    void cbiPlanner_SampledState(const mace::state_space::State* sampleState) override;
    void cbiPlanner_NewConnection(const mace::state_space::State* beginState, const mace::state_space::State* secondState) override;

private:
    mace::state_space::GoalState m_goalState;
    mace::pose::CartesianPosition_2D m_castGoalState;

private: //variables explicit for the potential fields approach
    void setupF3StaticMap();
    void updateAgentAction();
    VPF_ResultingForce computeVirtualForce(double &vResponse);
    command_target::DynamicTarget_Kinematic computeDynamicTarget(const VPF_ResultingForce &apfObj, const double &vResponse);

    mace::maps::Data2DGrid<mace::maps::OccupiedResult>* staticMap;
    PotentialFields* m_Planner;

private:
    mace::state_space::Cartesian2DSpacePtr m_Space;
    mace::state_space::Cartesian2DSpace_SamplerPtr sampler;
    mace::state_space::SpecialValidityCheckPtr stateCheck;
    mace::state_space::SpaceInformationPtr spaceInfo;

private:
    mace::pose::Rotation_3D m_AgentRotation;
    mace::pose::CartesianPosition_3D m_AgentPosition;
    mace::pose::Velocity_Cartesian3D m_AgentVelocity;

private:
    //!
    //! \brief m_VehicleDataTopic Vehicle data topic collection
    //!
    Data::TopicDataObjectCollection<BASE_POSE_TOPICS> m_VehicleDataTopic;

    //!
    //! \brief m_MissionDataTopic Mission data topic collection
    //!
    Data::TopicDataObjectCollection<DATA_MISSION_GENERIC_TOPICS> m_MissionDataTopic;

    //!
    //! \brief m_PlanningStateTopic
    //!
    Data::TopicDataObjectCollection<BASE_GEOMETRY_TOPICS, BASE_POSE_TOPICS> m_PlanningStateTopic;

    //!
    //! \brief m_MapTopic
    //!
    Data::TopicDataObjectCollection<MAP_DATA_TOPICS> m_MapTopic;

    private:
    mace::state_space::Cartesian2DSpacePtr goalSpace;
    mace::state_space::Cartesian2DSpace_SamplerPtr m_goalSampler;

    std::ofstream myfile;


};
#endif // MODULE_PATH_PLANNING_NASAPHASE2_H
