#include "potential_fields.h"
#include "iterators/circle_map_iterator.h"
#include <cmath>
#include <iostream>
#include <fstream>


namespace mace
{
PotentialFields::PotentialFields(const state_space::SpaceInformationPtr &spaceInfo, const mace::maps::Data2DGrid<mace::maps::OccupiedResult>* staticMap):
    Planners(spaceInfo),
     targetPositionActive(false),
     m_repulsionRadius(3),
     m_linearAttractionRadius(2),
     m_planningRadius(2),
     m_repulsionGain(100),
     m_linearAttractionGain(2),
     m_radialInfluence(4),
     m_goalThreshold(1),
     m_goalRadialInfluence(0.5)
{
    m_conicalAttractionGain = m_linearAttractionGain/m_goalThreshold;
    m_currentMapObject = nullptr;
    m_totalForceGrid = nullptr;

    VPF_ResultingForce fillValue;
    m_staticRespulsiveMap = new mace::maps::Data2DGrid<VPF_ResultingForce>(&fillValue);
    m_totalForceGrid = new mace::maps::Data2DGrid<VPF_ResultingForce>(&fillValue);

    if(staticMap != nullptr)
        updateStaticObstacleGradient(staticMap);
}

void PotentialFields::setPlanningParameters(state_space::GoalState* begin, state_space::GoalState* end)
{
    //cast away the const
    mace::state_space::State* state = const_cast<mace::state_space::State*>(begin->getState());
    //lets cast the state pointer into the type we are expecting it to be
    const mace::pose::CartesianPosition_2D* cast = static_cast<const mace::pose::CartesianPosition_2D*>(state);

    m_agentPose.setXPosition(cast->getXPosition());
    m_agentPose.setYPosition(cast->getYPosition());

    //cast away the const
    state = const_cast<mace::state_space::State*>(end->getState());
    //lets cast the state pointer into the type we are expecting it to be
    cast = static_cast<const mace::pose::CartesianPosition_2D*>(state);

    m_targetPosition.setXPosition(cast->getXPosition());
    m_targetPosition.setYPosition(cast->getYPosition());
}

void PotentialFields::updateStaticObstacleGradient(const mace::maps::Data2DGrid<mace::maps::OccupiedResult>* staticMap)
{
    if(staticMap == nullptr)
        m_staticRespulsiveMap->clear();
    else
    {
        m_staticRespulsiveMap->updateGridSize(staticMap->getXMin(), staticMap->getXMax(),
                                 staticMap->getYMin(), staticMap->getYMax(),
                                 staticMap->getXResolution(),staticMap->getYResolution());

        m_totalForceGrid->updateGridSize(staticMap->getXMin(), staticMap->getXMax(),
                                         staticMap->getYMin(), staticMap->getYMax(),
                                         staticMap->getXResolution(), staticMap->getYResolution());


        mace::maps::GridMapIterator gridMapItr(staticMap);

        for(; !gridMapItr.isPastEnd(); ++gridMapItr)
        {
            const mace::maps::OccupiedResult* ptr = staticMap->getCellByIndex(*gridMapItr);

            switch (*ptr) {
            case mace::maps::OccupiedResult::OCCUPIED:
            case mace::maps::OccupiedResult::ENVIRONMENT_BOUNDARY:
            {
                double obstacleX,obstacleY;
                staticMap->getPositionFromIndex(*gridMapItr, obstacleX, obstacleY);
                mace::pose::CartesianPosition_2D obstacle(obstacleX,obstacleY);

              //generate circle iterator for the map layer of the influential radius an obstacle can have
                mace::maps::CircleMapIterator circleMapItr(staticMap,obstacle,m_radialInfluence);

                for(circleMapItr.begin();!circleMapItr.isPastEnd();++circleMapItr)
                {
                    //get position of current circleMapItr cell
                    double indexX, indexY;
                    staticMap->getPositionFromIndex(*circleMapItr, indexX, indexY);
                    mace::pose::CartesianPosition_2D currentPos(indexX,indexY);

                    //call updateRepulsiveGradient on current x,y position and obstacle position
                    double incidentAngle = 0.0;
//                    if(*ptr == mace::maps::OccupiedResult::OCCUPIED)
//                        incidentAngle = M_PI_2;

                    VPF_ResultingForce rf = computeRepulsiveGradient(&obstacle, &currentPos, incidentAngle);

                    //get correct cell in locally stored static grid
                    VPF_ResultingForce* currentStaticIndex = m_staticRespulsiveMap->getCellByIndex(*circleMapItr);
                    VPF_ResultingForce* currentTotalForceIndex = m_totalForceGrid->getCellByIndex(*circleMapItr);

                    *currentStaticIndex += rf;
                    *currentTotalForceIndex += rf;
                }
                break;
            }
            }
        }
    }
}

VPF_ResultingForce PotentialFields::retrieveRepulsiveSummation(const mace::pose::CartesianPosition_2D &currentPosition)
{
    //get correct cell in locally stored static grid
    return *(m_staticRespulsiveMap->getCellByPos(currentPosition.getXPosition(), currentPosition.getYPosition()));
}


VPF_ResultingForce PotentialFields::retrieveStaticObstacleGradient(int index)
{
    //get correct cell in locally stored static grid
    return *(m_staticRespulsiveMap->getCellByIndex(index));
}

VPF_ResultingForce PotentialFields::computeRepulsiveGradient(const Abstract_CartesianPosition *obstaclePosition, const Abstract_CartesianPosition *cellPosition, const double &incidentAngle)
{
    VPF_ResultingForce rf;

    double distance = obstaclePosition->distanceBetween2D(cellPosition);

    //calculate repulsive force magnitude for each axis
    if((distance <= m_radialInfluence) && (distance > std::numeric_limits<double>::epsilon())) //should be relative to some epsilon
    {
        Eigen::VectorXd normalizedVector = (obstaclePosition->getDataVector() - cellPosition->getDataVector()).normalized();
        Eigen::Vector2d dimensionVector = normalizedVector.head(2);
        Eigen::Matrix2d rotation = Eigen::Matrix2d::Zero(); rotation << cos(incidentAngle), -sin(incidentAngle), sin(incidentAngle), cos(incidentAngle); //[cos(theta) -sin(theta); sin(theta) cos(theta)]
        Eigen::Vector2d rotatedVector = rotation*dimensionVector;
        rf.setForceX((m_repulsionGain / pow(distance, 2)) * ((1/distance) - (1/m_radialInfluence)) * -rotatedVector.x());
        rf.setForceY((m_repulsionGain / pow(distance, 2)) * ((1/distance) - (1/m_radialInfluence)) * -rotatedVector.y());
    }
    return rf;
}

VPF_ResultingForce PotentialFields::computeAttractionGradient(const mace::pose::CartesianPosition_2D agentPose,  const mace::pose::CartesianPosition_2D targetPosition)
{

    VPF_ResultingForce vf;

    if(targetPosition.distanceBetween2D(agentPose) <= m_goalThreshold)
    {
        vf.setForceX(m_conicalAttractionGain * -agentPose.deltaX(targetPosition));
        vf.setForceY(m_conicalAttractionGain * -agentPose.deltaY(targetPosition));
    }
    else
    {
        double deltaX = targetPosition.deltaX(agentPose);
        double deltaY = targetPosition.deltaY(agentPose);
        std::cout<<"The delta positions are: "<<deltaX<<","<<deltaY<<std::endl;
        vf.setForceX(m_goalThreshold * m_linearAttractionGain * (targetPosition.deltaX(agentPose) / targetPosition.distanceBetween2D(agentPose)));
        vf.setForceY(m_goalThreshold * m_linearAttractionGain * (targetPosition.deltaY(agentPose) / targetPosition.distanceBetween2D(agentPose)));
    }
    return vf;
}

VPF_ResultingForce PotentialFields::computeArtificialForceVector(const mace::pose::Abstract_CartesianPosition* agentPosition,  const mace::pose::Abstract_CartesianPosition* targetPosition)
{
    VPF_ResultingForce rtnObj;

    const mace::pose::CartesianPosition_2D* current = nullptr;
    const mace::pose::CartesianPosition_2D* target = nullptr;

    if(agentPosition->isGreaterThan1D())
        current = agentPosition->positionAs<mace::pose::CartesianPosition_2D>();
    if(targetPosition->isGreaterThan1D())
        target = targetPosition->positionAs<mace::pose::CartesianPosition_2D>();

    if((current == nullptr) || (target == nullptr))
        return rtnObj;

    VPF_ResultingForce attraction = computeAttractionGradient(*current,*target);
    //VPF_ResultingForce repulsion = retrieveRepulsiveSummation(*current);

    rtnObj = attraction;
    return rtnObj;
}

void PotentialFields::computeVirtualPotentialField()
{

}

double PotentialFields::getRepulsionRadius() const
{
    return m_repulsionRadius;
}

void PotentialFields::setRepulsionRadius(double repulsionRadius)
{
    m_repulsionRadius = repulsionRadius;
}

double PotentialFields::getLinearAttractionRadius() const
{
    return m_linearAttractionRadius;
}

void PotentialFields::setLinearAttractionRadius(double linearAttractionRadius)
{
    m_linearAttractionRadius = linearAttractionRadius;
}

double PotentialFields::getPlanningRadius() const
{
    return m_planningRadius;
}

void PotentialFields::setPlanningRadius(double planningRadius)
{
    m_planningRadius = planningRadius;
}

double PotentialFields::getRepulsionGain() const
{
    return m_repulsionGain;
}

void PotentialFields::setRepulsionGain(double repulsionGain)
{
    m_repulsionGain = repulsionGain;
}

double PotentialFields::getAttractionGain() const
{
    return m_linearAttractionGain;
}

void PotentialFields::setAttractionGain(double attractionGain)
{
    m_linearAttractionGain = attractionGain;
}

double PotentialFields::getRadialInfluence() const
{
    return m_radialInfluence;
}

void PotentialFields::setRadialInfluence(double radialInfluence)
{
    m_radialInfluence = radialInfluence;
}

/*!
 * \brief used for debugging - will print entire frontier grid
 * \param none
 * \return void
 */
void PotentialFields::printGrid()
{
    std::ofstream myfile;
    std::cout<< " print grid " << std::endl;

    myfile.open("C:/Github/OpenMACE/potential_fields.csv");

    for(unsigned int index = 0; index < m_staticRespulsiveMap->getSize(); index++)
    {
        double x = 0.0, y = 0.0;

        m_staticRespulsiveMap->getPositionFromIndex(index, x, y);
        myfile << x << ",";
        myfile << y << ",";

        myfile << m_totalForceGrid->getCellByIndex(index)->getForceX() << ", " ;
        myfile << m_totalForceGrid->getCellByIndex(index)->getForceY() << "\n " ;
    }

    myfile.close();
    std::cout<<" ____________________ " <<std::endl;


}

double PotentialFields::getGoalThreshold() const
{
    return m_goalThreshold;
}

void PotentialFields::setGoalThreshold(double goalThreshold)
{
    m_goalThreshold = goalThreshold;
}

double PotentialFields::getGoalRadialInfluence() const
{
    return m_goalRadialInfluence;
}

void PotentialFields::setGoalRadialInfluence(double goalRadialInfluence)
{
    m_goalRadialInfluence = goalRadialInfluence;
}

double PotentialFields::getResultingSearchRadius() const
{
    return m_resultingSearchRadius;
}

void PotentialFields::setResultingSearchRadius(double radius)
{
    m_resultingSearchRadius = radius;
}

mace::pose::CartesianPosition_2D PotentialFields::getAgentPose() const
{
    return m_agentPose;
}

void PotentialFields::setAgentPose(const mace::pose::CartesianPosition_2D &value)
{
    m_agentPose = value;
}

mace::pose::CartesianPosition_2D PotentialFields::getTargetPosition() const
{
    return m_targetPosition;
}

void PotentialFields::setTargetPosition(const mace::pose::CartesianPosition_2D &targetPosition)
{
    m_targetPosition = targetPosition;
}
}
