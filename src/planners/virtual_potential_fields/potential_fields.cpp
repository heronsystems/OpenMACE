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
    m_repulsionRadius(6),
    m_linearAttractionRadius(2),
    m_planningRadius(2),
    m_repulsionGain(30),
    m_linearAttractionGain(0.5),
    m_radialInfluence(5),
    m_attractiveGoalThreshold(2),
    m_attractiveLinearGoalThreshold(5),
    m_goalRadialInfluence(3)
{

    m_repulsionGain = m_attractiveLinearGoalThreshold * m_linearAttractionGain * 2;
    m_conicalAttractionGain = m_linearAttractionGain/m_attractiveGoalThreshold;
    m_currentMapObject = nullptr;

    VPF_ResultingForce fillValue;
    m_staticRespulsiveMap = new mace::maps::Data2DGrid<VPF_ResultingForce>(&fillValue);
    m_staticAttractionMap = new mace::maps::Data2DGrid<VPF_ResultingForce>(&fillValue);

    if(staticMap != nullptr)
    {
        updateStaticObstacleGradient(staticMap);

        printGrid();
    }

    myfile.open("C:/Github/OpenMACE/potential_fields_logging.csv");

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

void PotentialFields::updateStaticAttractionGradient(const mace::maps::Data2DGrid<mace::maps::OccupiedResult> *staticMap)
{
    if(staticMap == nullptr)
        m_staticAttractionMap->clear();
    else
    {
        m_staticAttractionMap->updateGridSize(staticMap->getXMin(), staticMap->getXMax(),
                                              staticMap->getYMin(), staticMap->getYMax(),
                                              staticMap->getXResolution(),staticMap->getYResolution());

        mace::maps::GridMapIterator gridMapItr(staticMap);
        for(; !gridMapItr.isPastEnd(); ++gridMapItr)
        {
            double positionX,positionY;
            staticMap->getPositionFromIndex(*gridMapItr, positionX, positionY);
        } //end of for loop iterator
    }
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

                    *currentStaticIndex += rf;
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

VPF_ResultingForce PotentialFields::computeAttractionGradient(const mace::pose::CartesianPosition_2D agentPose, const mace::pose::Rotation_2D &agentRotation,
                                                              const mace::pose::CartesianPosition_2D targetPosition)
{
    VPF_ResultingForce vf;

    Eigen::Vector2d targetVector = targetPosition.getDataVector();
    Eigen::Vector2d agentVector = agentPose.getDataVector();

    double bearing = agentPose.polarBearingTo(&targetPosition);

    double deltaX = targetPosition.deltaX(agentPose);
    double deltaY = targetPosition.deltaY(agentPose);
    double distanceToGoal = agentPose.distanceBetween2D(targetPosition);
    std::cout<<"The delta positions are: "<<deltaX<<","<<deltaY<<std::endl;

    if(distanceToGoal <= m_attractiveGoalThreshold)
    {
        double gain_attraction = 2;
        double gain_zero = 0.005;
        double gain_transition = 0.8;
        double denomenator = 1 + std::exp(-gain_transition * distanceToGoal) / gain_zero;
        double f_att = gain_attraction / denomenator;
        vf.setForceX(f_att * (targetPosition.getXPosition() - agentPose.getXPosition()) / distanceToGoal);
        vf.setForceY(f_att * (targetPosition.getYPosition() - agentPose.getYPosition()) / distanceToGoal);
    }
    else if((distanceToGoal > m_attractiveGoalThreshold) && (distanceToGoal <= m_attractiveLinearGoalThreshold))
    {
        vf.setForceX(m_linearAttractionGain * (targetPosition.deltaX(agentPose) / targetPosition.distanceBetween2D(agentPose)));
        vf.setForceY(m_linearAttractionGain * (targetPosition.deltaY(agentPose) / targetPosition.distanceBetween2D(agentPose)));
    }
    else
    {
        vf.setForceX(m_linearAttractionGain * m_attractiveLinearGoalThreshold * (targetPosition.deltaX(agentPose) / targetPosition.distanceBetween2D(agentPose)));
        vf.setForceY(m_linearAttractionGain * m_attractiveLinearGoalThreshold * (targetPosition.deltaY(agentPose) / targetPosition.distanceBetween2D(agentPose)));
    }

    return vf;
}

VPF_ResultingForce PotentialFields::computeArtificialForceVector(const mace::pose::Abstract_CartesianPosition* agentPosition, const mace::pose::Velocity_Cartesian2D* agentVelocity,
                                                                 const mace::pose::Abstract_CartesianPosition* targetPosition, double &vResponse)
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

    mace::pose::CartesianPosition_2D evalPosition(*current);
    evalPosition.setXPosition(evalPosition.getXPosition() + agentVelocity->getXVelocity());
    evalPosition.setYPosition(evalPosition.getYPosition() + agentVelocity->getYVelocity());


    //VPF_ResultingForce attraction = computeAttractionGradient(*current,*target);
    VPF_ResultingForce attraction;
    VPF_ResultingForce repulsion = retrieveRepulsiveSummation(evalPosition);

    //apply fuzzy logic
    Eigen::Vector2d attractionVector(attraction.getForceX(), attraction.getForceY());
    Eigen::Vector2d repulsionVector(repulsion.getForceX(), repulsion.getForceY());
    Eigen::Vector2d rotatedRepulsionVector = repulsionVector;

    double angle = 0.0;

    if((fabs(repulsion.getForceX()) > std::numeric_limits<double>::epsilon()) || (fabs(repulsion.getForceY()) > std::numeric_limits<double>::epsilon()))
    {
        angle = acos(attractionVector.dot(repulsionVector) / (attractionVector.norm() * repulsionVector.norm()));
        angle = mace::math::wrapToPi(angle);
        if(fabs(angle) > mace::math::convertDegreesToRadians(90))
        {
            int signValue = mace::math::sgn<double>(angle);
            double incidentAngle = M_PI_4 * signValue;
            Eigen::Matrix2d rotation = Eigen::Matrix2d::Zero(); rotation << cos(incidentAngle), -sin(incidentAngle), sin(incidentAngle), cos(incidentAngle); //[cos(theta) -sin(theta); sin(theta) cos(theta)]
            rotatedRepulsionVector = rotation*repulsionVector + repulsionVector;
        }
//        else if (fabs(angle) > mace::math::convertDegreesToRadians(90)) {
//            int signValue = mace::math::sgn<double>(angle);
//            double incidentAngle = M_PI_4 * signValue;
//            Eigen::Matrix2d rotation = Eigen::Matrix2d::Zero(); rotation << cos(incidentAngle), -sin(incidentAngle), sin(incidentAngle), cos(incidentAngle); //[cos(theta) -sin(theta); sin(theta) cos(theta)]
//            rotatedRepulsionVector = rotation*repulsionVector;
//        }
    }

    repulsion.setForceX(rotatedRepulsionVector.x());
    repulsion.setForceY(rotatedRepulsionVector.y());


    rtnObj = attraction + repulsion;

    double forceFactor = sqrt(pow(repulsion.getForceX(),2) + pow(repulsion.getForceY(),2)) / 6;

    vResponse = 2/(1+forceFactor);

    myfile << current->getXPosition() << ",";
    myfile << current->getYPosition() << ",";
    myfile << evalPosition.getXPosition() << ",";
    myfile << evalPosition.getYPosition() << ",";
    myfile << attraction.getForceX() << ", " ;
    myfile << attraction.getForceY() << ", " ;
    myfile << repulsion.getForceX() << ", " ;
    myfile << repulsion.getForceY() << ", " ;
    myfile << vResponse << ", " ;
    myfile << angle << "\n " ;

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
    //    std::ofstream myfile;

    myfile.open("C:/Github/OpenMACE/potential_fields.csv");

    for(unsigned int index = 0; index < m_staticRespulsiveMap->getSize(); index++)
    {
        double x = 0.0, y = 0.0;

        m_staticRespulsiveMap->getPositionFromIndex(index, x, y);
        myfile << x << ",";
        myfile << y << ",";
    }

    myfile.close();
    std::cout<<" ____________________ " <<std::endl;


}

double PotentialFields::getGoalThreshold() const
{
    return m_attractiveGoalThreshold;
}

void PotentialFields::setGoalThreshold(double goalThreshold)
{
    m_attractiveGoalThreshold = goalThreshold;
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
