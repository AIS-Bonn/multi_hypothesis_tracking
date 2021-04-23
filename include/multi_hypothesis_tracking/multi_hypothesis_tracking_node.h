/** @file
 *
 * Multi hypothesis tracking node receiving detection messages and providing those to the multi hypothesis tracker.
 *
 * @author Jan Razlaw
 */

#ifndef __MULTI_OBJECT_TRACKING_NODE_H__
#define __MULTI_OBJECT_TRACKING_NODE_H__

#include <geometry_msgs/PoseArray.h>

#include <multi_hypothesis_tracking/definitions.h>
#include <multi_hypothesis_tracking/multi_hypothesis_tracking_base.h>

#include <multi_hypothesis_tracking_msgs/ObjectDetections.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>


namespace MultiHypothesisTracker
{

/**
 * @brief Ros node to track multiple hypotheses simultaneously.
 */
class MultiHypothesisTrackingNode : public MultiHypothesisTrackingBase
{
public:
  /** @brief Constructor. */
  MultiHypothesisTrackingNode();
  /** @brief Destructor. */
  ~MultiHypothesisTrackingNode(){};

  /**
   * @brief Callback function for PoseArray messages.
   *
   * Converts messages to detections.
   * Transforms detections to #m_world_frame and passes those to the tracking algorithm.
   *
   * @param [in] msg    poses of the detections.
   */
  void detectionPosesCallback(const geometry_msgs::PoseArray::ConstPtr& msg);

  /**
   * @brief Callback function for ObjectDetection messages.
   *
   * Converts messages to detections.
   * Transforms detections to #m_world_frame and passes those to the tracking algorithm.
   *
   * @param [in] msg    detections.
   */
  void detectionCallback(const multi_hypothesis_tracking_msgs::ObjectDetections::ConstPtr& msg);

  /**
   * @brief Converts the detection's poses from the laser into the internal format
   *
   * @param[in]     msg             poses of the detections.
   * @param[out]    detections      detections in tracker format.
   */
  void convert(const geometry_msgs::PoseArray::ConstPtr& msg,
               Detections& detections);

  /**
   * @brief Converts the detections from the laser into the internal format
   *
   * @param[in]     msg             detections.
   * @param[out]    detections      detections in tracker format.
   */
  void convert(const multi_hypothesis_tracking_msgs::ObjectDetections::ConstPtr& msg,
               Detections& detections);

public:
  /** @brief Subscribes to detections. */
  ros::Subscriber m_laser_detection_subscriber;
};

}

#endif
