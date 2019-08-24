#include "potential_fields.h"
#include "iterators/circle_map_iterator.h"
#include <cmath>
#include <iostream>
#include <fstream>


namespace mace
{
PotentialFields::PotentialFields(const state_space::SpaceInformationPtr &spaceInfo, mace::maps::BaseGridMap* staticMap):
    Planners(spaceInfo),
     targetPositionActive(false),
     m_repulsionRadius(4),
     m_linearAttractionRadius(2),
     m_planningRadius(2),
     m_repulsionGain(2),
     m_attractionGain(0.5),
     m_radialInfluence(4),
     m_goalThreshold(1),
     m_goalRadialInfluence(0.5)
{
    m_currentMapObject = nullptr;
    m_totalForceGrid = nullptr;

    VPF_ResultingForce fillValue;
    m_staticGridRepulsiveFields = new mace::maps::Data2DGrid<VPF_ResultingForce>(&fillValue);
            m_totalForceGrid = new mace::maps::Data2DGrid<VPF_ResultingForce>(&fillValue);

    if(staticMap != nullptr)
    {
        updateStaticObstacleGradient(staticMap);
    }

}

PotentialFields::PotentialFields(const state_space::SpaceInformationPtr &spaceInfo):
    Planners(spaceInfo),
     targetPositionActive(false),
     m_repulsionRadius(0),
     m_linearAttractionRadius(0),
     m_planningRadius(0),
     m_repulsionGain(0),
     m_attractionGain(0),
     m_radialInfluence(0)
{
    m_currentMapObject = nullptr;

    VPF_ResultingForce fillValue;
    m_staticGridRepulsiveFields = new mace::maps::Data2DGrid<VPF_ResultingForce>(&fillValue);

}

void PotentialFields::setPlanningParameters(state_space::GoalState* begin, state_space::GoalState* end)
{
    //cast away the const
    mace::state_space::State* state = const_cast<mace::state_space::State*>(begin->getState());
    //lets cast the state pointer into the type we are expecting it to be
    const mace::pose::CartesianPosition_2D* cast = static_cast<const mace::pose::CartesianPosition_2D*>(state);

    m_agentPose.setXPosition(cast->getXPosition());
    m_agentPose.setYPosition(cast->getYPosition());
    m_agentPose.setCoordinateFrame(LocalFrameType::CF_LOCAL_ENU);

    //cast away the const
    state = const_cast<mace::state_space::State*>(end->getState());
    //lets cast the state pointer into the type we are expecting it to be
    cast = static_cast<const mace::pose::CartesianPosition_2D*>(state);

    m_targetPosition.setXPosition(cast->getXPosition());
    m_targetPosition.setYPosition(cast->getYPosition());
    m_targetPosition.setCoordinateFrame(LocalFrameType::CF_LOCAL_ENU);
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

            if(*ptr == mace::maps::OccupiedResult::OCCUPIED)
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
                    VPF_ResultingForce rf = computeRepulsiveGradient(currentPos, obstacle);

                    //get correct cell in locally stored static grid
                    VPF_ResultingForce* currentStaticIndex = m_staticRespulsiveMap->getCellByIndex(*circleMapItr);
                    VPF_ResultingForce* currentTotalForceIndex = m_totalForceGrid->getCellByIndex(*circleMapItr);

                    currentStaticIndex->addForceX(rf.getForceX());
                    currentTotalForceIndex->addForceX(rf.getForceX());
                    currentStaticIndex->addForceY(rf.getForceY());
                    currentTotalForceIndex->addForceY(rf.getForceY());

                   // currentStaticIndex->setDirection(rf->getDirection());
                }
            }
            else
            {
                std::cout << "m_targetPosition" << m_targetPosition.getXPosition() << m_targetPosition.getYPosition() << std::endl;

                staticMapLayer->getPositionFromIndex(*gridMapItr, indexX, indexY);
                std::cout<< "index X" << indexX << "index Y " << indexY << std::endl;
                mace::pose::CartesianPosition_2D currentPos(indexX, indexY);
                VPF_ResultingForce vf = computeAttractionGradient(currentPos, m_targetPosition);

                std::cout << " vf force x = " << vf.getForceX() << "vf force y = " << vf.getForceY() << std::endl;
                VPF_ResultingForce* currentTotalForceIndex = m_totalForceGrid->getCellByIndex(*gridMapItr);


                currentTotalForceIndex->addForceX(vf.getForceX());
                currentTotalForceIndex->addForceY(vf.getForceY());


            }
        }

       // printGrid();
    }
    else
    {
       throw std::runtime_error("Member object m_currentMapObject is a nullptr");
    }


}

VPF_ResultingForce PotentialFields::retrieveStaticObstacleGradient(int index)
{
    //get correct cell in locally stored static grid
    return *(m_staticRespulsiveMap->getCellByIndex(index));

}

VPF_ResultingForce PotentialFields::computeRepulsiveGradient(const Abstract_CartesianPosition *obstaclePosition, const Abstract_CartesianPosition *cellPosition)
{
    VPF_ResultingForce rf;
    bool fuzzyLogicApplied = false;

    double distance = cellPosition->distanceBetween2D(obstaclePosition);

    //calculate repulsive force magnitude for each axis
    if((distance <= m_radialInfluence) && (distance > 0)) //should be relative to some epsilon
    {
        rf.setForceX((m_repulsionGain / pow(distance, 2)) * ((1/distance) - (1/m_radialInfluence)) * (cellPosition.deltaX(obstaclePosition)/distance));
        rf.setForceY((m_repulsionGain / pow(distance, 2)) * ((1/distance) - (1/m_radialInfluence)) * (cellPosition.deltaY(obstaclePosition)/distance));

        //apply fuzzy logic
        //TODO actually apply fuzzy logic
        fuzzyLogicApplied = true; //we would want to compute this outside of the function
    }
    else
    {

    }

    return rf;
}

//you may want to consider passing in the map layer to the function and computing off of it
//would simplify and generalize the class better and reduce dependents
VPF_ResultingForce PotentialFields::computeDynamicObstacleGradient(const mace::pose::CartesianPosition_2D pose, mace::maps::BaseGridMap* bgm)
{

    std::cout << " computeDynamicObstacleGradient" << std::endl;

    VPF_ResultingForce rf;

    if(bgm != nullptr)
    {
        mace::maps::Data2DGrid<mace::maps::OccupiedResult>* occupiedGrid = dynamic_cast<mace::maps::Data2DGrid<mace::maps::OccupiedResult>*>(bgm);

        //generate circle iterator for the map layer of the planningRadius
        if(m_planningRadius > m_repulsionRadius)
        {
            m_resultingSearchRadius = m_planningRadius;
        }
        else
        {
            m_resultingSearchRadius = m_repulsionRadius;
        }
        mace::maps::CircleMapIterator circleMapItr(bgm,pose,m_resultingSearchRadius);

        double indexX, indexY;

        for(circleMapItr.begin();!circleMapItr.isPastEnd();++circleMapItr)
        {

            mace::maps::OccupiedResult* ptr = occupiedGrid->getCellByIndex(*circleMapItr);

            if(*ptr == mace::maps::OccupiedResult::OCCUPIED)
            {

                occupiedGrid->getPositionFromIndex(*circleMapItr, indexX, indexY);
                mace::pose::CartesianPosition_2D currentPos(indexX,indexY);

                rf = computeRepulsiveGradient(currentPos,pose);

                VPF_ResultingForce* currentTotalForceIndex = m_totalForceGrid->getCellByIndex(*circleMapItr);

                currentTotalForceIndex->addForceX(rf.getForceX());
                currentTotalForceIndex->addForceY(rf.getForceY());




            }

        }

        //rf.computeDirection();
    }
    else
    {
        throw std::runtime_error("Map layer is a nullptr");
    }

    std::cout<< "end" << std::endl;

    return rf;

}

VPF_ResultingForce PotentialFields::computeAttractionGradient(const mace::pose::CartesianPosition_2D agentPose,  const mace::pose::CartesianPosition_2D targetPosition)
{

    VPF_ResultingForce vf;

    if(targetPosition.distanceBetween2D(agentPose) < m_goalThreshold)
    {
        vf.setForceX(m_attractionGain * (targetPosition.deltaX(agentPose) - m_goalRadialInfluence));
        vf.setForceY(m_attractionGain * (targetPosition.deltaY(agentPose) - m_goalRadialInfluence));
    }
    else
    {
        vf.setForceX(m_goalThreshold * ((m_attractionGain * (targetPosition.deltaX(agentPose) - m_goalRadialInfluence)) / (targetPosition.distanceBetween2D(agentPose))));
        vf.setForceY(m_goalThreshold * ((m_attractionGain *  (targetPosition.deltaY(agentPose)- m_goalRadialInfluence)) / (targetPosition.distanceBetween2D(agentPose))));
    }
    return vf;
}

void PotentialFields::computeNewTargetState()
{

}

void PotentialFields::computeVirtualPotentialField()
{

}


TargetItem::Cartesian3DDynamicTarget PotentialFields::computeTotalGradient(mace::maps::BaseGridMap* dynamicMap,
                                                                            mace::pose::CartesianPosition_2D agentPose,
                                                                            mace::pose::CartesianPosition_2D targetPosition)
{

    std::cout<< " compute total gradient" << std::endl;

    VPF_ResultingForce vf1, vf2, vf3;

    int index = m_staticGridRepulsiveFields->indexFromPos(agentPose.getXPosition(), agentPose.getYPosition());
    vf1 = this->retrieveStaticObstacleGradient(index);

    if(dynamicMap != nullptr)
    {
        std::cout<< "dynamic map is not null" << std::endl;
        vf2 = this->computeDynamicObstacleGradient(agentPose, dynamicMap);
    }
    else
    {
        std::cout<< "dynamic map is null" << std::endl;
    }
    std::cout<< " dynamicObstacleGradient = " << vf2.getForceX() << ", " <<  vf2.getForceY() << std::endl;

    vf3 = this->computeAttractionGradient(agentPose, targetPosition);

    std::cout<< " attractionGradient = " << vf3.getForceX() << ", " <<  vf3.getForceY() << std::endl;

    VPF_ResultingForce totalForce;
    totalForce.addForceX(vf1.getForceX() + vf2.getForceX() + vf3.getForceX());
    totalForce.addForceY(vf1.getForceY() + vf2.getForceY() + vf3.getForceY());
    totalForce.updateDirection();

    std::cout<< "TOTAL FORCE:" << totalForce.getForceX() << " , " << totalForce.getForceY() << " , " << totalForce.getDirection() << std::endl;

    m_totalForceGrid->getCellByIndex(index)->setForceX(totalForce.getForceX());
    m_totalForceGrid->getCellByIndex(index)->setForceY(totalForce.getForceY());

    printGrid();

    TargetItem::Cartesian3DDynamicTarget target;
    CartesianVelocity_3D velocity;
    velocity.setXVelocity(totalForce.getForceX());
    velocity.setYVelocity(totalForce.getForceY());
    velocity.setZVelocity(0);

  //  target.setVelocity(velocity);

CartesianPosition_3D position;
position.setXPosition(8);
position.setYPosition(4);
position.setZPosition(10);
target.setPosition(position);

    std::cout << " leaving compute total gradient"<< std::endl;
    return target;


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
    return m_attractionGain;
}

void PotentialFields::setAttractionGain(double attractionGain)
{
    m_attractionGain = attractionGain;
}

double PotentialFields::getRadialInfluence() const
{
    return m_radialInfluence;
}

void PotentialFields::setRadialInfluence(double radialInfluence)
{
    m_radialInfluence = radialInfluence;
}

std::string PotentialFields::getStaticLayerName() const
{
    return m_staticLayerName;
}

void PotentialFields::setStaticLayerName(const std::string &staticLayerName)
{
    m_staticLayerName = staticLayerName;
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

    myfile.open("C:/mace/potential_fields.csv");

    for(int y = 0; y< m_staticGridRepulsiveFields->getYMax(); y++)
    {
        for(int x = 0; x<m_staticGridRepulsiveFields->getXMax(); x++)
        {
            myfile << x << ",";
            myfile << y << ",";

            std::cout << m_totalForceGrid->getCellByPos(x,y)->getForceX() << ", " ;
            myfile << m_totalForceGrid->getCellByPos(x,y)->getForceX() << ", " ;
            std::cout << m_totalForceGrid->getCellByPos(x,y)->getForceY() << ", " ;
            myfile << m_totalForceGrid->getCellByPos(x,y)->getForceY() << "\n " ;
            //std::cout << m_staticGridRepulsiveFields->getCellByPos(j,i)->getDirection() ;
            //myfile << m_staticGridRepulsiveFields->getCellByPos(j,i)->getDirection() << "\n";
            std::cout << " | " ;
        }
        std::cout<< " " << std::endl;
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
