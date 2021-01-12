/*!
  * @file trajectory_queue.h
  * 
  * @authors
  *     Kenneth Kroeger ken.kroeger@heronsystems.com
  *     Patrick Nolan pat.nolan@heronsystems.com
  *
  * @section PROJECT
  *     This is a part of Heron Systems participation within the APL's Skyborg Program exercising TACE with existing autonomy. 
  *
  * @section DESCRIPTION
  *
  * @date
  *     March 2020
  *
  * @copyright
  *     File and its related contents are subjected to a proprietary software license from
  *     Heron Systems Inc. The extent of the rights and further details are located in the
  *     top level of directory via LICENSE.md file.
  **/

#ifndef TRAJECTORY_QUEUE_H
#define TRAJECTORY_QUEUE_H

#include "common/common.h"

#include "base/trajectory/queue_interface.h"
#include "base/trajectory/trajectory_point.h"

class TrajectoryQueue: public QueueInterface
{
public:
  TrajectoryQueue(const double &samplingPeriod = 100.0, const double &predictionHorizon = 10000.0, const unsigned int &predictionSteps = 1); //times here are conveyed in milliseconds

  virtual ~TrajectoryQueue();

public:
  bool retrieveLeadingItem(TrajectoryPoint &obj);

  bool updateQueue();

  bool updateQueue(TrajectoryPoint &obj);

  void updateQueueToIndex(const unsigned int &index);

  void updateSamplingPeriod(const double &period, const bool &update = false);

  void updatePredictionHorizon(const double &duration, const bool &update = false);

  void updatePredictionSteps(const unsigned int &steps, const bool &update = false);

  void initializeQueue();

  void clearQueue();

  bool isQueueEmpty() const
  {
    return _trajectoryQueue.empty();
  }

private:
  void shrinkQueueToMinimum();

  void updateMinimumQueueSize();

  void popFrontPoint();

  void popBackPoint();

  void getLastPoint(TrajectoryPoint &obj);

  void linearInterpolateTrajectory(const VectorStateQueue &input_queue, VectorStateQueue &interpolated_queue);

public:
  void pushBackPoint(const TrajectoryPoint &obj);

  void insertReferenceTrajectory(const VectorStateQueue &insertQueue);

  void retrieveImmediateHorizon(VectorStateQueue &obj) const;

  void retrieveFullHorizon(VectorStateQueue &obj) const;

private:
  VectorStateQueue _trajectoryQueue;

  unsigned int _minQueueSize = 0;
  unsigned int _maxQueueSize = 1000; //this equates to 10Hz @ 10 second horizon

  double _samplingPeriod = 100.0; //the number of milliseconds between each iteration
  double _predictionPeriod = 10.0;
  unsigned int _predictionSteps = 1;
};

#endif // TRAJECTORY_QUEUE_H
