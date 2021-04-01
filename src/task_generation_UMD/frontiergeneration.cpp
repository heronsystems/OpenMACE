   #include "frontiergeneration.h"
#include "maps/data_2d_grid.h"
#include "data_tasks/task_loiter_descriptor.h"

/*!
 * \brief contructor
 * \param none
 * \return none
 */
frontierGeneration::frontierGeneration():
    Abstract_TaskGeneration()
{
    std::cout<< "FrontierGeneration contructor" << std::endl;

    m_algorithmType = AlgorithmTypes::FRONTIER;

    space = std::make_shared<mace::state_space::Cartesian2DSpace>();
    sampler = std::make_shared<mace::state_space::Cartesian2DSpace_Sampler>(space);
    spaceInfo = std::make_shared<mace::state_space::SpaceInformation>(space);

    const FrontierCell* fillValue = new FrontierCell();
    frontierGrid = new mace::maps::Data2DGrid<FrontierCell>(fillValue);

    // occupiedGrid will be null at first
    occupiedGrid = nullptr;

    // start at the zero index first
    // this will get overwritten by robot's position
    searchStartIndex = NULL;

}

/*!
 * \brief creates tasks from frontier locations
 * \param none
 * \return State space
 */
std::list<std::shared_ptr<TaskDescriptor>> frontierGeneration::generateFrontierWaypoint()
{
    //create a task list to send to callback function
    std::list<std::shared_ptr<TaskDescriptor>> taskList;

    mace::state_space::State* sampleState;

    //go through current frontier groupings, calculate the centroid, and assign it to a task
    while(m_currentFrontiers.size()>0)
    {
        sampleState = space->getNewState();

        unsigned int x = NULL;
        unsigned int y = NULL;

        //calculate centroids
        double Xsum = 0;
        double Ysum = 0;
        double size = m_currentFrontiers.front().size();

        while(m_currentFrontiers.front().size()>0)
        {
            occupiedGrid->getIndexDecomposed(m_currentFrontiers.front().front(), x, y);
            Xsum += x;
            Ysum += y;
            m_currentFrontiers.front().pop();
        }

        double Xaverage = Xsum/size;
        double Yaverage = Ysum/size;

        //no longer need this frontier grouping
        m_currentFrontiers.pop();

        //lets cast the state pointer into the type we are expecting it to be
        mace::pose::CartesianPosition_2D* cast = static_cast<mace::pose::CartesianPosition_2D*>(sampleState);

        //assign the centroid position information
        cast->setXPosition(Xaverage);
        cast->setYPosition(Yaverage);

        //create task with this location information
        std::shared_ptr<TaskLoiterDescriptor<mace::pose::CartesianPosition_2D>> task = std::make_shared<TaskLoiterDescriptor<mace::pose::CartesianPosition_2D>>(m_creatorID, taskCounter);
        task->setLoiterPosition(*cast);

        taskCounter++;

        Data::EnvironmentTime now;
        Data::EnvironmentTime::CurrentTime(Data::SYSTEMCLOCK, now);
        task->setTimeCreated(now);

        TaskKey key = task->getTaskKey();


        taskList.push_back(task);
    }

    //let the callback know that a task is available
    if(m_CBInterface)
        m_CBInterface->newlyAvailableTask(taskList);

     return taskList;
}

/*!
 * \brief assigns layeredMap to keep track of
 * \param pointer to layered map
 * \return void
 */
void frontierGeneration::assignMapLayerObject(mace::maps::LayeredMap* layeredMap)
{
    //call the abstract base class functionality here
    Abstract_TaskGeneration::assignMapLayerObject(layeredMap);

}

/*!
 * \brief update current map and search for frontiers
 * \param map layer name and entity position
 * \return void
 */
void frontierGeneration::newlyUpdatedMapLayer(const std::string &layerName, const mace::pose::CartesianPosition_2D* pose)
{
    std::cout<< "newlyUpdatedMapLayer" << std::endl;
    //grab appropriate layer from our layered map (should be an occupancy map)
    mace::maps::BaseGridMap* bgm = m_currentMapObject->getMapLayer(layerName);

    //TODO add in a check to make sure it actually returns an occupiedGrid
    occupiedGrid = dynamic_cast<mace::maps::Data2DGrid<mace::maps::OccupiedResult>*>(bgm);

    if(occupiedGrid->getSize() != frontierGrid->getSize())
    {
        frontierGrid->updateGridSize(occupiedGrid->getXMin(),
                                     occupiedGrid->getXMax(),
                                     occupiedGrid->getYMin(),
                                     occupiedGrid->getYMax(),
                                     occupiedGrid->getXResolution(),
                                     occupiedGrid->getYResolution());

        space->setBounds(mace::state_space::Cartesian2DSpaceBounds(occupiedGrid->getXMin(),
                                                                   occupiedGrid->getXMax(),
                                                                   occupiedGrid->getYMin(),
                                                                   occupiedGrid->getYMax()));
    }

    // determine robot's position
    const double x = pose->getXPosition();
    const double y = pose->getYPosition();

    //if this is the first robot position we are ever getting, make sure its starting in an open cell
    if(searchStartIndex == NULL)
    {
        //first time getting a entity's pose
         mace::maps::OccupiedResult* entityPose = occupiedGrid->getCellByPos(x,y);
         int entityIndex = occupiedGrid->indexFromPos(x,y);

         //if its not open, find the closest cell that is open
         if(*entityPose != mace::maps::OccupiedResult::NOT_OCCUPIED)
         {

             bool found = false;
             int currentIndex = NULL;

             std::queue<int> findOpenQueue;
             findOpenQueue.push(entityIndex);

             while(findOpenQueue.size()>0 && !found)
             {
                currentIndex = findOpenQueue.front();
                findOpenQueue.pop();

                std::vector<int> neighbors = occupiedGrid->getCellNeighbors(currentIndex, true);

                while(neighbors.size() > 0 && !found)
                {
                    if(!found)
                    {
                        currentIndex = neighbors.back();

                        neighbors.pop_back();

                        mace::maps::OccupiedResult* currentOccupiedGridCell = occupiedGrid->getCellByIndex(currentIndex);

                        if(*currentOccupiedGridCell == mace::maps::OccupiedResult::NOT_OCCUPIED)
                        {
                            //found the closest not_occupied and it's location will be stored in currentIndex
                            found = true;
                        }
                        else
                        {
                            findOpenQueue.push(currentIndex);
                        }
                    }
                }
            }
            // this is the new cell to start at
            searchStartIndex = currentIndex;
        }
        else
        {
            searchStartIndex = occupiedGrid->indexFromPos(x,y);
        }
    }
    else
    {
        // determine index based off position
        searchStartIndex = occupiedGrid->indexFromPos(x,y);
    }


    // enqueue and start the search
    m_locationQueue.push(searchStartIndex);
    FrontierCell* firstCell = frontierGrid->getCellByIndex(searchStartIndex);
    firstCell->setVisitedState(FrontierCell::Visited_State::VISITED);

    //start coarse outer breadth search
    while(m_locationQueue.size() > 0)
    {
        int location = m_locationQueue.front();
        m_locationQueue.pop();

        //Coarse Outer Breadth Search
        std::vector<int> neighbors = occupiedGrid->getCellNeighbors(location);

        std::cout<< neighbors.size() <<std::endl;

        //search through neighbors vector for frontiers
        while(neighbors.size()>0)
        {
            int currentIndex = neighbors.back();

            neighbors.pop_back();

            mace::maps::OccupiedResult* currentOccupiedGridCell = occupiedGrid->getCellByIndex(currentIndex);
            FrontierCell* currentFrontierGridCell = frontierGrid->getCellByIndex(currentIndex);

            //check to see if it is unoccupied
            if(*currentOccupiedGridCell == mace::maps::OccupiedResult::NOT_OCCUPIED  &&
               currentFrontierGridCell->getVisitedState() == FrontierCell::Visited_State::NOT_VISITED)
            {
                m_locationQueue.push(currentIndex);
                currentFrontierGridCell->setVisitedState(FrontierCell::Visited_State::VISITED);
            }
            else
            {
                // new frontier cell check
                bool isFrontier = newFrontierCellCheck(currentOccupiedGridCell, currentFrontierGridCell, currentIndex);

                if(isFrontier)
                {
                    //its a frontier, so check to see if theres any other frontiers attached to it
                    expandedFrontierSearch(currentOccupiedGridCell, currentFrontierGridCell, currentIndex);
                }
            }
        }
    }

  //debugging
  // printGrid();
  // calculateCentroids();
}

/*!
 * \brief search for frontiers attached to a known frontier
 * \param pointers to a cell in the occupied grid and frontier grid. current index of frontier
 * \return void
 */
void frontierGeneration::expandedFrontierSearch(mace::maps::OccupiedResult* occupiedGridCell, FrontierCell* frontierGridCell, int index)
{
    //expand frontier search
    frontierGridCell->setFrontierStatus(FrontierCell::Frontier_Status::EXPANDED);
    m_frontiersQueue.push(index);

    //initalize a new queue to store any linked frontiers
    std::queue<int>  frontierClusters;

    while(m_frontiersQueue.size() > 0)
    {
        int frontierIndex = m_frontiersQueue.front();

        m_frontiersQueue.pop();

        mace::maps::OccupiedResult* currentOccupiedGridCell = occupiedGrid->getCellByIndex(frontierIndex);
        FrontierCell* currentFrontierGridCell = frontierGrid->getCellByIndex(frontierIndex);

        if(currentFrontierGridCell->getFrontierStatus() != FrontierCell::Frontier_Status::CLOSED &&
                currentFrontierGridCell->getVisitedState() != FrontierCell::Visited_State::VISITED)
        {
            bool isFrontier = newFrontierCellCheck(currentOccupiedGridCell,currentFrontierGridCell, frontierIndex);

            if(isFrontier)
            {
                fineFrontierSearch(frontierIndex, &frontierClusters);
                currentFrontierGridCell->setFrontierStatus(FrontierCell::Frontier_Status::CLOSED);
            }
        }

    }

    //push this list of linked frontiers onto the queue
    m_currentFrontiers.push(frontierClusters);

}

/*!
 * \brief search 4 neighbor grid around a given index
 * \param index and queue to push frontiers into
 * \return void
 */
void frontierGeneration::fineFrontierSearch(int index, std::queue<int>*  frontierClusters)
{

    FrontierCell* currentFrontierGridCell = frontierGrid->getCellByIndex(index);
    currentFrontierGridCell->setFrontierState(FrontierCell::Frontier_State::FRONTIER);
    currentFrontierGridCell->setVisitedState(FrontierCell::Visited_State::VISITED);

    frontierClusters->push(index);

    std::vector<int> neighbors = occupiedGrid->getCellNeighbors(index, true);

    for(int i = 0; i<neighbors.size(); i++)
    {
        FrontierCell* cell = frontierGrid->getCellByIndex(neighbors.at(i));

        if(cell->getVisitedState() == FrontierCell::Visited_State::NOT_VISITED)
        {
            m_frontiersQueue.push(neighbors.at(i));
            cell->setFrontierStatus(FrontierCell::Frontier_Status::EXPANDED);
        }
    }

}

/*!
 * \brief checks if this specific cell is a frontier
 * \param cell in occupied grid and frontier grid, current index
 * \return true if frontier
 */
bool frontierGeneration::newFrontierCellCheck(mace::maps::OccupiedResult* occupiedGridCell, FrontierCell* frontierGridCell, int index)
{
    //check to see if we already know about this cell
    if(*occupiedGridCell != mace::maps::OccupiedResult::UNKNOWN ||
            frontierGridCell->getFrontierState() == FrontierCell::Frontier_State::FRONTIER )
    {
        //no new frontiers here
        return false;
    }
    else
    {
       //if any neighbors are not occupied, this is a frontier
        std::vector<int> neighbors = occupiedGrid->getCellNeighbors(index, true);

        for(int i = 0; i<neighbors.size(); i++)
        {
            if(*occupiedGrid->getCellByIndex(neighbors.at(i)) == mace::maps::OccupiedResult::NOT_OCCUPIED)
            {
                return true;
            }
        }
    }

    return false;

}

/*!
 * \brief used for debugging - calculate centroids of current frontier groups
 * \param none
 * \return void
 */
void frontierGeneration::calculateCentroids()
{
    std::cout << "currentFrontiers size = " << m_currentFrontiers.size() << std::endl;

    std::queue<std::queue<int>> tempQueue = m_currentFrontiers;

    while(tempQueue.size()>0)
    {
        double Xsum = 0;
        double Ysum = 0;
        double size = tempQueue.front().size();

        unsigned int x;
        unsigned int y;

        while(tempQueue.front().size()>0)
        {
        occupiedGrid->getIndexDecomposed(tempQueue.front().front(), x, y);
        Xsum += x;
        Ysum += y;
        tempQueue.front().pop();
        }

        double Xaverage = Xsum/size;
        double Yaverage = Ysum/size;

        std::cout<<"CENTROID OF FRONTIERS = " << "(" << Xaverage << "," << Yaverage<< ")" << std::endl;

        tempQueue.pop();
    }

}

/*!
 * \brief used for debugging - will print entire frontier grid
 * \param none
 * \return void
 */
void frontierGeneration::printGrid()
{

    for(int i = 0; i< frontierGrid->getYMax(); i++)
    {
        for(int j = 0; j<frontierGrid->getXMax(); j++)
        {
            std::cout << enum_to_stringFRONTIER(frontierGrid->getCellByPos(j,i)->getFrontierState()) << ", " ;
            std::cout << enum_to_stringVISITED(frontierGrid->getCellByPos(j,i)->getVisitedState());
            std::cout << " | " ;
        }
        std::cout<< " " << std::endl;
    }

    std::cout<<" ____________________ " <<std::endl;


}

/*!
 * \brief used for debugging - prints frontier enum
 * \param frontier state enum
 * \return enum as a printable string
 */
char* frontierGeneration::enum_to_stringFRONTIER(FrontierCell::Frontier_State t)
{
    switch(t)
    {
        case FrontierCell::Frontier_State::FRONTIER:
            return "FRONTIER";
        case FrontierCell::Frontier_State::NON_FRONTIER:
            return "NON_FRONTIER";
         default:
            return "ERROR";
    }
}

/*!
 * \brief used for debugging - prints visited enum
 * \param visited state enum
 * \return enum as a printable string
 */
char* frontierGeneration::enum_to_stringVISITED(FrontierCell::Visited_State t)
{
    switch(t)
    {
        case FrontierCell::Visited_State::VISITED:
            return "V";
        case FrontierCell::Visited_State::NOT_VISITED:
            return "NV";
         default:
            return "ERROR";
    }
}
void frontierGeneration::newTaskAssignment(std::vector<TaskKey>)
{

}
