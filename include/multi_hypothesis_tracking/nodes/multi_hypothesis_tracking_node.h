/** @file
 *
 * Multi hypothesis tracking node receiving detection messages and providing those to the multi hypothesis tracker.
 *
 * @author Jan Razlaw
 */

#ifndef MULTI_HYPOTHESIS_TRACKING_MULTI_HYPOTHESIS_TRACKING_NODE_H
#define MULTI_HYPOTHESIS_TRACKING_MULTI_HYPOTHESIS_TRACKING_NODE_H

#include <multi_hypothesis_tracking/definitions.h>
#include <multi_hypothesis_tracking/hypotheses/hypothesis_for_sparse_lidar_factory.h>
#include <multi_hypothesis_tracking/nodes/multi_hypothesis_tracking_base.h>

#include <multi_hypothesis_tracking_msgs/ObjectDetections.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>


namespace MultiHypothesisTracker
{

typedef multi_hypothesis_tracking_msgs::ObjectDetections DetectionsMsg;

class MultiHypothesisTrackingNode : public MultiHypothesisTrackingBase
{
public:
  MultiHypothesisTrackingNode();
  ~MultiHypothesisTrackingNode(){};

  void initializeHypothesisFactory(const ros::NodeHandle& private_node_handle) override;

  /**
   * @brief Callback function for ObjectDetection messages.
   *
   * Converts messages to detections.
   * Transforms detections to #m_world_frame_id and passes those to the tracking algorithm.
   *
   * @param [in] detections_message    detections.
   */
  void detectionCallback(const DetectionsMsg::ConstPtr& detections_message);

  /**
   * @brief Converts the detections from the laser into the internal format
   *
   * @param[in]     detections_message     detections.
   * @param[out]    detections             detections in tracker format.
   */
  void convert(const DetectionsMsg::ConstPtr& detections_message,
               Detections& detections);

public:
  /** @brief Subscribes to object detections. */
  ros::Subscriber m_object_detection_subscriber;
};

}

#endif
