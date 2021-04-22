/** @file
 *
 * Multi human tracking node receiving human messages and providing those to the multi hypothesis tracker.
 *
 * @author Jan Razlaw
 */

#ifndef __MULTI_HUMAN_TRACKING_NODE_H__
#define __MULTI_HUMAN_TRACKING_NODE_H__

#include <chrono>
#include <fstream>
#include <iostream>

#include <person_msgs/PersonCovList.h>

#include <multi_hypothesis_tracking/definitions.h>
#include <multi_hypothesis_tracking/multi_hypothesis_tracking_base.h>

#include <ros/ros.h>
#include <ros/console.h>


namespace MultiHypothesisTracker
{

typedef person_msgs::PersonCovList HumanMsg;

/**
 * @brief Ros node to track multiple hypotheses simultaneously.
 */
class MultiHumanTrackingNode : public MultiHypothesisTrackingBase
{
public:
  /** @brief Constructor. */
  MultiHumanTrackingNode();
  /** @brief Destructor. */
  ~MultiHumanTrackingNode(){};

  /**
   * @brief Callback function for HumanMsg messages.
   *
   * Converts messages to detections.
   * Transforms detections to #m_world_frame and passes those to the tracking algorithm.
   *
   * @param [in] msg    detections.
   */
  void detectionCallback(const HumanMsg::ConstPtr& msg);

  /**
   * @brief Converts the human detections into the internal format.
   *
   * @param[in]     msg             message containing detections.
   * @param[out]    detections      detections in tracker format.
   */
  void convert(const HumanMsg::ConstPtr& msg,
               Detections& detections);

private:
  /** @brief Subscribes to detections. */
  ros::Subscriber m_human_detection_subscriber;
};

}

#endif
