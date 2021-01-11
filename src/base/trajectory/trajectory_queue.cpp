#include "trajectory_queue.h"

TrajectoryQueue::TrajectoryQueue(const double &samplingPeriod, const double &predictionHorizon, const unsigned int &predictionSteps)
{
    updateSamplingPeriod(samplingPeriod);
    updatePredictionHorizon(predictionHorizon);
    updatePredictionSteps(predictionSteps);
}

TrajectoryQueue::~TrajectoryQueue()
{
}

bool TrajectoryQueue::retrieveLeadingItem(TrajectoryPoint &obj)
{
    if (!_trajectoryQueue.empty())
    {
        obj = _trajectoryQueue.front();
        return true;
    }
    return false;
}

bool TrajectoryQueue::updateQueue()
{
    TrajectoryPoint tmpObj;
    return updateQueue(tmpObj);
}

bool TrajectoryQueue::updateQueue(TrajectoryPoint &obj)
{
    popFrontPoint();
    return retrieveLeadingItem(obj);
}

void TrajectoryQueue::updateQueueToIndex(const unsigned int &activeIndex)
{
    if(activeIndex > (_trajectoryQueue.size() - 1))
        _trajectoryQueue.clear();
    else
    {
        for (unsigned int index = 0; index < activeIndex; index++)
            popFrontPoint();
    }
    
}

void TrajectoryQueue::updateSamplingPeriod(const double &period, const bool &update)
{
    _samplingPeriod = period;
    if (update)
        updateMinimumQueueSize();
}

void TrajectoryQueue::updatePredictionHorizon(const double &duration, const bool &update)
{
    _predictionPeriod = duration;
    if (update)
        updateMinimumQueueSize();
}

void TrajectoryQueue::updatePredictionSteps(const unsigned int &steps, const bool &update)
{
    _predictionSteps = steps;
    if (update)
        updateMinimumQueueSize();
}

void TrajectoryQueue::initializeQueue()
{
    clearQueue();
}

void TrajectoryQueue::clearQueue()
{
    _trajectoryQueue.clear();
}

void TrajectoryQueue::shrinkQueueToMinimum()
{
    while (_trajectoryQueue.size() > _minQueueSize)
        popBackPoint();
}

void TrajectoryQueue::updateMinimumQueueSize()
{
    _minQueueSize = _predictionSteps * std::ceil(_predictionPeriod / _samplingPeriod);
}

void TrajectoryQueue::popFrontPoint()
{
    if (_trajectoryQueue.size() > 0)
    {
        _trajectoryQueue.pop_front();
    }
}

void TrajectoryQueue::popBackPoint()
{
    if (_trajectoryQueue.size() > 0)
    {
        _trajectoryQueue.pop_back();
    }
}

void TrajectoryQueue::getLastPoint(TrajectoryPoint &obj)
{
    if (!_trajectoryQueue.empty())
        obj = _trajectoryQueue.back();
}

void TrajectoryQueue::linearInterpolateTrajectory(const VectorStateQueue &input_queue,
                                                  VectorStateQueue &interpolated_queue)
{
    if (input_queue.size() < 2) //if the queue only consists of 1 point, there is nothing to interpolate
    {
        interpolated_queue = input_queue;
        return;
    }

    interpolated_queue = input_queue;
}

void TrajectoryQueue::pushBackPoint(const TrajectoryPoint &obj)
{
    if (_trajectoryQueue.size() < _maxQueueSize)
    {
        _trajectoryQueue.push_back(obj);
        on_QueueUpdate();
    }
}

void TrajectoryQueue::insertReferenceTrajectory(const VectorStateQueue &insertQueue)
{
    //VectorStateQueue interpolatedQueue;
    //linearInterpolateTrajectory(insertQueue, interpolatedQueue);

    /*
    if (!_trajectoryQueue.empty())
    {
        EnvironmentTime currentTime, queueStartTime;
        queueStartTime = _trajectoryQueue.front()._time;
        EnvironmentTime commandStart = interpolatedQueue.front()._time;



        if ((labs(commandStart.DiffMSecs(queueStartTime)) <= _samplingPeriod) || (labs(currentTime.DiffMSecs(commandStart)) <= 10)) //if the time this queue is less than current or 0 insert into beginning
        {
            std::cout<<"I am erasing"<<std::endl;
            clearQueue();
        }
        else //this is intended to be a splice
        {
            std::cout<<"I am splicing."<<std::endl;
            std::cout<<"The command time: "<<commandStart.EpochMSecs()<<std::endl;
            std::cout<<"The queue time: "<<queueStartTime.EpochMSecs()<<std::endl;

            //We cannot splice like this until the interpolation is valid
            // EnvironmentTime queueEndTime = _trajectoryQueue.back()._time;
            // if (commandStart < queueEndTime)
            // {
            //     size_t startIndex = std::round(commandStart.DiffMSecs(queueStartTime) / _samplingPeriod);
            //     _trajectoryQueue.erase(_trajectoryQueue.begin() + startIndex, _trajectoryQueue.end());
            // }
        }
    }
    */
    clearQueue();

    for (auto it = insertQueue.cbegin(); it != insertQueue.cend(); ++it)
    {
        _trajectoryQueue.push_back(*it);
    }

    on_QueueUpdate();
}

void TrajectoryQueue::retrieveImmediateHorizon(VectorStateQueue &obj) const
{
    obj.clear();

    unsigned int samples = _predictionSteps * std::ceil(_predictionPeriod / _samplingPeriod);
    if (samples > _trajectoryQueue.size())
        samples = _trajectoryQueue.size();

    for (unsigned int index = 0; index < samples; index++)
    {
        obj.push_back(_trajectoryQueue.at(index));
    }
}

void TrajectoryQueue::retrieveFullHorizon(VectorStateQueue &obj) const
{
    obj.clear();

    unsigned int samples = _trajectoryQueue.size();

    for (unsigned int index = 0; index < samples; index++)
    {
        obj.push_back(_trajectoryQueue.at(index));
    }
}

