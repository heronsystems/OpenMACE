#ifndef FRONTIERGENERATION_H
#define FRONTIERGENERATION_H

#include "abstract_task_generation.h"

#include "base/state_space/space_information.h"
#include "base/state_space/cartesian_2D_space.h"
#include "maps/data_2d_grid.h"
#include "maps/occupancy_definition.h"
#include "queue"

MACE_CLASS_FORWARD(frontierGeneration);
class FrontierCell
{
    public:
        //enumeration to classify the state of each cell
        enum class Frontier_State
        {
            FRONTIER,
            NON_FRONTIER
        };

        //enumeration to classify the state of each cell
        enum class Visited_State
        {
            VISITED,
            NOT_VISITED
        };

        enum class Frontier_Status
        {
            NONE,
            EXPANDED,
            CLOSED
        };

        //start with these states automatically
        FrontierCell()
        {
            m_frontierState = Frontier_State::NON_FRONTIER;
            m_visitedState = Visited_State::NOT_VISITED;
            m_frontierStatus = Frontier_Status::NONE;

        }

        //getters and setters
        Frontier_State getFrontierState()
        {
            return m_frontierState;
        }

        void setFrontierState(Frontier_State fs)
        {
            m_frontierState = fs;
        }

        Visited_State getVisitedState()
        {
            return m_visitedState;
        }

        void setVisitedState(Visited_State vs)
        {
            m_visitedState = vs;
        }

        Frontier_Status getFrontierStatus()
        {
            return m_frontierStatus;
        }

        void setFrontierStatus(Frontier_Status fs)
        {
            m_frontierStatus = fs;
        }

    private:

        Frontier_State m_frontierState;
        Visited_State m_visitedState;
        Frontier_Status m_frontierStatus;
};

class frontierGeneration: public Abstract_TaskGeneration
{
public:

    // constructor
    frontierGeneration();

    // destructor
    ~frontierGeneration(){}

    // taskGeneration
     std::list<std::shared_ptr<TaskDescriptor>> generateFrontierWaypoint();

    // assign our local LayeredMap to this argument
    void assignMapLayerObject(mace::maps::LayeredMap* layeredMap) override;

    // when a map in the layeredMap object is updated, we search for new frontiers
    void newlyUpdatedMapLayer(const std::string &layerName, const mace::pose::CartesianPosition_2D* pose) override;

    void expandedFrontierSearch(mace::maps::OccupiedResult* occupiedGridCell, FrontierCell* frontierGridCell, int index);

    void fineFrontierSearch(int index, std::queue<int>*  frontierClusters);

    // check to see if this cell is a new frontier
    bool newFrontierCellCheck(mace::maps::OccupiedResult* occupiedGridCell, FrontierCell* frontierGridCell, int index);

    void calculateCentroids();

    void printGrid();
    char* enum_to_stringFRONTIER(FrontierCell::Frontier_State t);
    char* enum_to_stringVISITED(FrontierCell::Visited_State t);
    //idk
    virtual void newTaskAssignment(std::vector<TaskKey>) override;

private:

    std::shared_ptr<mace::state_space::SpaceInformation> spaceInfo;
    std::shared_ptr<mace::state_space::Cartesian2DSpace> space;
    std::shared_ptr<mace::state_space::Cartesian2DSpace_Sampler> sampler;

    //local frontier grid to keep track of visited cells and new frontiers
    mace::maps::Data2DGrid<FrontierCell>* frontierGrid;

    //occupied grid that we search over to find frontiers
    mace::maps::Data2DGrid<mace::maps::OccupiedResult>* occupiedGrid;

    std::queue<int> m_locationQueue;
    std::queue<int> m_frontiersQueue;

    std::queue<std::queue<int>> m_currentFrontiers;




    //robot's position
    int searchStartIndex;
};

#endif // FRONTIERGENERATION_H
