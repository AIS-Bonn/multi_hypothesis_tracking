/** @file
 *
 * Multi hypothesis tracking node receiving pose array messages and providing those to the multi hypothesis tracker.
 *
 * @author Jan Razlaw
 */

#ifndef MULTI_HYPOTHESIS_TRACKING_MULTI_HYPOTHESIS_TRACKING_NODE_FOR_POSE_ARRAYS_H
#define MULTI_HYPOTHESIS_TRACKING_MULTI_HYPOTHESIS_TRACKING_NODE_FOR_POSE_ARRAYS_H

#include <geometry_msgs/PoseArray.h>

#include <multi_hypothesis_tracking/definitions.h>
#include <multi_hypothesis_tracking/nodes/multi_hypothesis_tracking_base.h>

#include <ros/ros.h>
#include <ros/console.h>


namespace MultiHypothesisTracker
{

typedef geometry_msgs::PoseArray PoseArrayMsg;

class MultiHypothesisTrackingNodeForPoseArrays : public MultiHypothesisTrackingBase
{
public:
  MultiHypothesisTrackingNodeForPoseArrays();
  ~MultiHypothesisTrackingNodeForPoseArrays(){};

  void initializeHypothesisFactory(const ros::NodeHandle& private_node_handle) override;

  /**
   * @brief Callback function for PoseArray messages.
   *
   * Converts messages to detections.
   * Transforms detections to #m_world_frame and passes those to the tracking algorithm.
   *
   * @param [in] detections_message    poses of the detections.
   */
  void detectionPosesCallback(const PoseArrayMsg::ConstPtr& detections_message);

  /**
   * @brief Converts the detection's poses to the internal format.
   *
   * @param[in]     msg             poses of the detections.
   * @param[out]    detections      detections in tracker format.
   */
  void convert(const PoseArrayMsg::ConstPtr& msg,
               Detections& detections);

public:
  /** @brief Subscribes to pose arrays. */
  ros::Subscriber m_pose_array_subscriber;
};

}

#endif //MULTI_HYPOTHESIS_TRACKING_MULTI_HYPOTHESIS_TRACKING_NODE_FOR_POSE_ARRAYS_H
