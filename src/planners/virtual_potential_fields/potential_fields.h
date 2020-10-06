#ifndef POTENTIAL_FIELDS_H
#define POTENTIAL_FIELDS_H
#include <fstream>
#include "maps/layered_map.h"
#include "virtual_force.h"
#include "../planners.h"
#include "../planners_global.h"
#include "base/pose/pose_components.h"
#include "data_generic_command_item/target_items/dynamic_target_kinematic.h"

#include <base/state_space/cartesian_2D_space.h>

struct logicValues {
    double small;
    double medium;
    double large;
    double min;
    double max;
};

class VelocityFuzzyLogicController
{

public:
    VelocityFuzzyLogicController()
    {}

    void setRepulsiveLogic(double p_small, double p_medium, double p_large, double p_min, double p_max)
    {
        attractiveLogic.small = p_small;
        attractiveLogic.medium = p_medium;
        attractiveLogic.large = p_large;
        attractiveLogic.min = p_min;
        attractiveLogic.max = p_max;

    }

    logicValues getRepulsiveLogic()
    {
        return repulsiveLogic;
    }

    void setAttractiveLogic(double p_small, double p_medium, double p_large, double p_min, double p_max)
    {
        attractiveLogic.small = p_small;
        attractiveLogic.medium = p_medium;
        attractiveLogic.large = p_large;
        attractiveLogic.min = p_min;
        attractiveLogic.max = p_max;

    }

    logicValues getAttractiveLogic()
    {
        return attractiveLogic;
    }


private:
    logicValues repulsiveLogic;
    logicValues attractiveLogic;

};

namespace mace {

class PotentialFields: public planners::Planners
{
public:
    PotentialFields(const state_space::SpaceInformationPtr &spaceInfo, const mace::maps::Data2DGrid<maps::OccupiedResult>* staticMap = nullptr);

    virtual ~PotentialFields() = default;


    void setPlanningParameters(state_space::GoalState* begin, state_space::GoalState* end);

    virtual void assignMapLayerObject(mace::maps::LayeredMap* layeredMap)
    {
        m_currentMapObject = layeredMap;
    }

    std::vector<state_space::State*> solve() {
        std::vector<state_space::State*> solutionVector;
        return solutionVector;
    }


    void computeRepulsiveGradientMap(const mace::maps::BaseGridMap* &dynamicMap);

    void computeAttactiveGradientMap(const mace::pose::CartesianPosition_2D &targetPosition);


    void updateStaticMapLayer(const mace::maps::Data2DGrid<maps::OccupiedResult> *staticMap);

    void updateStaticObstacleGradient(const mace::maps::Data2DGrid<mace::maps::OccupiedResult>* staticMap);

    void updateStaticAttractionGradient(const mace::maps::Data2DGrid<mace::maps::OccupiedResult>* staticMap);


    VPF_ResultingForce retrieveRepulsiveSummation(const mace::pose::CartesianPosition_2D &currentPosition);

    VPF_ResultingForce retrieveStaticObstacleGradient(int);

    VPF_ResultingForce computeRepulsiveGradient(const mace::pose::Abstract_CartesianPosition* obstaclePosition, const mace::pose::Abstract_CartesianPosition* cellPosition, const double &incidentAngle = 0.0);

    VPF_ResultingForce computeAttractionGradient(const mace::pose::CartesianPosition_2D agentPose,const mace::pose::Rotation_2D &agentRotation,
                                                 const mace::pose::CartesianPosition_2D targetPosition);

    VPF_ResultingForce computeArtificialForceVector(const mace::pose::Abstract_CartesianPosition* agentPosition, const mace::pose::Velocity_Cartesian2D* agentVelocity,
                                                    const mace::pose::Abstract_CartesianPosition* targetPosition, double &vResponse);

    void computeVirtualPotentialField();

//    TargetItem::Cartesian3DDynamicTarget computeTotalGradient(mace::maps::BaseGridMap* dynamicMap,
//                                                              mace::pose::CartesianPosition_2D agentPose,
//                                                              mace::pose::CartesianPosition_2D targetPosition);

    double getRepulsionRadius() const;
    void setRepulsionRadius(double repulsionRadius);

    double getLinearAttractionRadius() const;
    void setLinearAttractionRadius(double linearAttractionRadius);

    double getPlanningRadius() const;
    void setPlanningRadius(double planningRadius);

    double getRepulsionGain() const;
    void setRepulsionGain(double repulsionGain);

    double getAttractionGain() const;
    void setAttractionGain(double attractionGain);

    double getRadialInfluence() const;
    void setRadialInfluence(double radialInfluence);

    void printGrid();

    double getGoalThreshold() const;
    void setGoalThreshold(double goalThreshold);

    double getGoalRadialInfluence() const;
    void setGoalRadialInfluence(double goalRadialInfluence);

    double getResultingSearchRadius() const;
    void setResultingSearchRadius(double radius);

    mace::pose::CartesianPosition_2D getAgentPose() const;
    void setAgentPose(const mace::pose::CartesianPosition_2D &value);

    mace::pose::CartesianPosition_2D getTargetPosition() const;
    void setTargetPosition(const mace::pose::CartesianPosition_2D &targetPosition);

private:

    double m_repulsionRadius;
    double m_linearAttractionRadius;
    double m_linearAttractionGain;
    double m_conicalAttractionGain;

    double m_planningRadius;
    double m_repulsionGain;
    double m_attractiveGoalThreshold;
    double m_attractiveLinearGoalThreshold;

    double m_resultingSearchRadius;
    double m_radialInfluence;
    double m_goalRadialInfluence;

    mace::pose::CartesianPosition_2D m_agentPose;
    mace::pose::CartesianPosition_2D m_targetPosition;

    bool targetPositionActive; //this was more generalized in the lucid chart and would be managed outside this class

    VelocityFuzzyLogicController m_FuzzyLogic;

    //local layered map
    mace::maps::LayeredMap* m_currentMapObject;

    mace::maps::Data2DGrid<VPF_ResultingForce>* m_staticRespulsiveMap;
    mace::maps::Data2DGrid<VPF_ResultingForce>* m_staticAttractionMap;

    std::shared_ptr<mace::state_space::SpaceInformation> spaceInfo;
    std::shared_ptr<mace::state_space::Cartesian2DSpace> space;

    std::ofstream myfile;

};

}
#endif // POTENTIAL_FIELDS_H
