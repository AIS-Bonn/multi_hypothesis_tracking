/** @file
 *
 * Multi human tracking node receiving human messages and providing those to the multi hypothesis tracker.
 *
 * @author Jan Razlaw
 */

#ifndef MULTI_HYPOTHESIS_TRACKING_MULTI_HYPOTHESIS_TRACKING_NODE_FOR_HUMANS_H
#define MULTI_HYPOTHESIS_TRACKING_MULTI_HYPOTHESIS_TRACKING_NODE_FOR_HUMANS_H

#include <person_msgs/PersonCovList.h>

#include <multi_hypothesis_tracking/definitions.h>
#include <multi_hypothesis_tracking/nodes/multi_hypothesis_tracking_base.h>

#include <ros/ros.h>
#include <ros/console.h>


namespace MultiHypothesisTracker
{

typedef person_msgs::PersonCovList HumanMsg;

class MultiHypothesisTrackingNodeForHumans : public MultiHypothesisTrackingBase
{
public:
  MultiHypothesisTrackingNodeForHumans();
  ~MultiHypothesisTrackingNodeForHumans(){};

  /**
   * @brief Callback function for HumanMsg messages.
   *
   * Converts messages to detections.
   * Transforms detections to #m_world_frame_id and passes those to the tracking algorithm.
   *
   * @param [in] detections_message    detections message.
   */
  void detectionCallback(const HumanMsg::ConstPtr& detections_message);

  /**
   * @brief Converts the human detections into the internal format.
   *
   * @param[in]     detections_message             message containing detections.
   * @param[out]    detections      detections in tracker format.
   */
  void convert(const HumanMsg::ConstPtr& detections_message,
               Detections& detections);

private:
  /** @brief Subscribes to detections. */
  ros::Subscriber m_human_detection_subscriber;
};

}

#endif
