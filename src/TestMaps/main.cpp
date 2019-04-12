#include <QCoreApplication>

#include "base/state_space/discrete_motion_validity_check.h"
#include "base/state_space/special_validity_check.h"
#include "base/state_space/cartesian_2D_space.h"
#include "maps/data_2d_grid.h"
#include "maps/occupancy_definition.h"

#include "task_generation_UMD/random_generation.h"

#include <iostream>
#include <QFile>
#include <QTextStream>
#include <QStringList>
#include <task_generation_UMD/frontiergeneration.h>
#include <module_task_generation/module_task_generation.h>
using namespace mace ;

const char kPathSeperator =
        #ifdef _WIN32
        '\\';
#else
        '/';
#endif

int main(int argc, char *argv[])
{

    std::cout << "HI" << std::endl;


    mace::maps::OccupiedResult fillData = mace::maps::OccupiedResult::NOT_OCCUPIED;

    mace::maps::Data2DGrid<mace::maps::OccupiedResult>* exampleMap = new mace::maps::Data2DGrid<mace::maps::OccupiedResult>(&fillData);
    exampleMap->updateGridSize(0,5,0,5,1,1);


    maps::OccupiedResult* value;

    for(int i =0; i<5; i++)
    {
        value= exampleMap->getCellByIndex(i);
        *value = maps::OccupiedResult::NOT_OCCUPIED;
    }

    value=exampleMap->getCellByIndex(5);
    *value=maps::OccupiedResult::OCCUPIED;

    value=exampleMap->getCellByIndex(6);
    *value=maps::OccupiedResult::OCCUPIED;

    value=exampleMap->getCellByIndex(7);
    *value=maps::OccupiedResult::UNKNOWN;

    for(int i =8; i<12; i++)
    {
        value= exampleMap->getCellByIndex(i);
        *value = maps::OccupiedResult::OCCUPIED;
    }

    value=exampleMap->getCellByIndex(12);
    *value=maps::OccupiedResult::NOT_OCCUPIED;

    for(int i =13; i<17; i++)
    {
        value= exampleMap->getCellByIndex(i);
        *value = maps::OccupiedResult::OCCUPIED;
    }

    value=exampleMap->getCellByIndex(17);
    *value=maps::OccupiedResult::UNKNOWN;

    value=exampleMap->getCellByIndex(18);
    *value=maps::OccupiedResult::OCCUPIED;

    value=exampleMap->getCellByIndex(19);
    *value=maps::OccupiedResult::OCCUPIED;

    for(int i =20; i<25; i++)
    {
        value= exampleMap->getCellByIndex(i);
        *value = maps::OccupiedResult::NOT_OCCUPIED;
    }


/*
    for(int i =0; i<5; i++)
    {
        maps::OccupiedResult* value= exampleMap->getCellByIndex(i);
        *value = maps::OccupiedResult::NOT_OCCUPIED;
    }

    for(int i=5; i< 9; i++)
    {
       maps::OccupiedResult* value2 = exampleMap->getCellByIndex(i);
       *value2 = maps::OccupiedResult::OCCUPIED;
    }

    maps::OccupiedResult* value7 = exampleMap->getCellByIndex(8);
    *value7 = maps::OccupiedResult::UNKNOWN;

    maps::OccupiedResult* value3 = exampleMap->getCellByIndex(9);
    *value3 = maps::OccupiedResult::UNKNOWN;


    for(int i=10; i< 15; i++)
    {
       maps::OccupiedResult* value4 = exampleMap->getCellByIndex(i);
       *value4 = maps::OccupiedResult::UNKNOWN;
    }

    for(int i =15; i<18; i++)
    {
        maps::OccupiedResult* value5 = exampleMap->getCellByIndex(i);
        *value5 = maps::OccupiedResult::OCCUPIED;
    }


    for(int i=18; i< 25; i++)
    {
       maps::OccupiedResult* value6 = exampleMap->getCellByIndex(i);
       *value6 = maps::OccupiedResult::UNKNOWN;

    }

    value3 = exampleMap->getCellByIndex(22);
    *value3 = maps::OccupiedResult::NOT_OCCUPIED;

    value3 = exampleMap->getCellByIndex(24);
    *value3 = maps::OccupiedResult::NOT_OCCUPIED;

    value3 = exampleMap->getCellByIndex(13);
    *value3 = maps::OccupiedResult::NOT_OCCUPIED;
*/

    std::unordered_map<std::string, mace::maps::BaseGridMap*> map;

    std::string example = "example";

    map.insert(std::make_pair(example, exampleMap));

    maps::LayeredMap* layeredMap = new maps::LayeredMap(map);

    frontierGeneration* frontierExample = new frontierGeneration();

    frontierExample->assignMapLayerObject(layeredMap);

    mace::maps::CartesianPosition_2D* pose = new mace::maps::CartesianPosition_2D();

    pose->setXPosition(2);
    pose->setYPosition(2);

    frontierExample->newlyUpdatedMapLayer(example, pose);

    frontierExample->generateFrontierWaypoint();


    /*
    int fillValue = 0;
    //maps::OccupiedResult fillValue = maps::OccupiedResult::EMPTY;
    maps::Data2DGrid<int>* m_Map = new maps::Data2DGrid<int>(&fillValue, 0, 5, 0, 5, 1, 1);

    std::vector<int*> neighbors = m_Map->getCellNeighbors(0, true);
    std::cout<< neighbors.size() << std::endl;

    const int* find = nullptr;
    int* cell = m_Map->getCellByIndex(10);
    *cell = 21;

    int foo = 21;
    const int *p = &foo;

    if(m_Map->cellValueEqualTo(10,p))
    {
        std::cout<< "CELL 10 IS EQUAL TO 21" << std::endl;

    }
    else
    {
        std::cout<<"CELL 10 IS NOT EQUAL TO 21" << std::endl;
    }
*/

//    for(int i=0; i<= neighbors.size(); i++)
//    {
//        std::cout<< neighbors.at(i).
//    }
    /*
    maps::OccupiedResult fillValue = maps::OccupiedResult::EMPTY;
    maps::Data2DGrid<mace::maps::OccupiedResult>* m_Map = new maps::Data2DGrid<maps::OccupiedResult>(&fillValue);

    TaskGeneration_RandomSpatial taskGenerator;
    taskGenerator.assignMapLayerObject(m_Map);
    state_space::State* randomState = taskGenerator.generateRandomWaypoint();
    //Ken Comment: The only reason we can do the following is because we are in control of the type
    pose::CartesianPosition_2D* newStatePosition = randomState->as<pose::CartesianPosition_2D>();
    std::cout<<"We have generated a new random state!"<<std::endl;
    std::cout << "Position X:" << newStatePosition->getXPosition() << " Position Y: " << newStatePosition->getYPosition()<<std::endl;
*/
    //    mace::maps::OctomapWrapper *wrapper = new mace::maps::OctomapWrapper();

//    std::list<TargetItem::DynamicTargetStorage> targetList;

//    TargetItem::DynamicTarget target;
//    mace::pose::CartesianPosition_3D targetPos(1000,1000,-10);
//    target.setPosition(targetPos);

//    TargetItem::DynamicTargetStorage targetNew(target,TargetItem::DynamicTargetStorage::INCOMPLETE);
//    targetList.push_back(targetNew);

//    MissionItem::MissionKey testKey(1,1,1,MissionItem::MISSIONTYPE::GUIDED);

//    TargetItem::DynamicMissionQueue availableQueue(testKey,1);



//    TargetItem::DynamicTargetList* newList = availableQueue.getDynamicTargetList();
//    newList->appendDynamicTarget(target);
//    std::cout<<"New queue available"<<std::endl;
    //availableQueue.getAssociatedMissionItem();
    //availableQueue.getDynamicTargetList()->appendDynamicTarget(target);

    /*
    char* MACEPath = getenv("MACE_ROOT");
    std::string rootPath(MACEPath);
    std::string btFile = rootPath + kPathSeperator + "load_303030.bt";
    mace::maps::OctomapWrapper octomap;

    octomap.loadOctreeFromBT(btFile);
    mace::maps::Data2DGrid<mace::maps::OccupiedResult>* compressedMap = octomap.get2DOccupancyMap();

    compressedMap->updateOriginPosition(mace::pose::CartesianPosition_2D(-15,-15));

    mace::state_space::Cartesian2DSpacePtr space = std::make_shared<mace::state_space::Cartesian2DSpace>();
    space->bounds.setBounds(-15,15,-15,15);

    mace::state_space::Cartesian2DSpace_SamplerPtr sampler = std::make_shared<mace::state_space::Cartesian2DSpace_Sampler>(space);
    mace::state_space::DiscreteMotionValidityCheckPtr motionCheck = std::make_shared<mace::state_space::DiscreteMotionValidityCheck>(space);
    mace::state_space::SpecialValidityCheckPtr stateCheck = std::make_shared<mace::state_space::SpecialValidityCheck>(space);
    auto stateValidityCheck = ([compressedMap](const mace::state_space::State *state){
        const mace::pose::CartesianPosition_2D* castState = state->as<const mace::pose::CartesianPosition_2D>();
        mace::maps::OccupiedResult* result = compressedMap->getCellByPos(castState->getXPosition(),castState->getYPosition());
        if(*result == mace::maps::OccupiedResult::NOT_OCCUPIED)
            return true;
        return false;
    });
    stateCheck->setLambda_Validity(stateValidityCheck);

    */

//    mace::state_space::Cartesian2DSpacePtr space = std::make_shared<mace::state_space::Cartesian2DSpace>();
//    space->bounds.setBounds(-15,15,-15,15);

//    state_space::State_TimeExpandedPtr test = std::make_shared<state_space::State_TimeExpanded>();
//    pose::Position<pose::CartesianPosition_2D>* point1 = new pose::Position<pose::CartesianPosition_2D>();
//    pose::Position<pose::CartesianPosition_2D>* point2 = new pose::Position<pose::CartesianPosition_2D>();

//    test->setState(point1);



//    point1->setXPosition(1);
//    point1->setYPosition(1);


//    point2->setXPosition(3);
//    point2->setYPosition(3);


//    double distance = space->distanceBetween(point1,point2);

//    std::cout<<"The distance here is: "<<std::endl;
    return 0;
}
